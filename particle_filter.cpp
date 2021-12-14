#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
#include <common/angle_functions.hpp>
#include <common/grid_utils.hpp>
#include <common/point.hpp>
#include <chrono>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  kLocalizationConverged_ (false)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ////////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleWeight=1.0 / kNumParticles_;
    posteriorPose_ = pose;

    for(auto& p : posterior_){
        p.pose.x = posteriorPose_.x;
        p.pose.y = posteriorPose_.y;
        p.pose.theta = wrap_to_pi(posteriorPose_.theta);
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sampleWeight;
    }

}

void ParticleFilter::initializeFilterUniformly(const pose_xyt_t& pose, const OccupancyGrid& map)
{

    // std::cout << "HERE\n";
    resamplePose_ = pose;

    std::vector<particle_t> init_field;

    std::cout << "HERE2\n";

    for (int x = 0; x < map.widthInCells(); x+=3){
        for (int y = 0; y < map.heightInCells(); y+=3){
            if (map.logOdds(x, y) < -100 ){
                Point<double> map_cell = grid_position_to_global_position(Point<double>(x, y), map);
                for (double t = 0.0; t < 2 * M_PI; t+=M_PI_2){
                    particle_t p;
                    p.pose.x = map_cell.x;
                    p.pose.y = map_cell.y;
                    p.pose.theta = wrap_to_pi(t);
                    p.pose.utime = pose.utime;
                    p.parent_pose = p.pose;
                    init_field.push_back(p);
                }
            }
        }
    }

    std::cout << "HERE3\n";

    posterior_ = init_field;
    kNumParticles_ = posterior_.size();
    double sampleWeight=1.0 / kNumParticles_;

    for (particle_t& part: posterior_)
    {
        part.weight = sampleWeight;
    }
    std::cout << "HERE4\n";

}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    int i=0;
    if(hasRobotMoved) // checks to see if the robot has moved 
    {
        auto start =std::chrono::high_resolution_clock::now();
        auto prior = resamplePosteriorDistribution(); // resample and re weight --> more particles at high weights 
        auto proposal = computeProposalDistribution(prior); // applies action model *NO NEED TO CHANGE 
        posterior_ = computeNormalizedPosterior(proposal, laser, map); // apples sensor model and then normize * FIXED LIELY OOD SO GO 
        posteriorPose_ = estimatePosteriorPose(posterior_); //find final post to update the map * FINE GO TO GO
        auto finish = std::chrono::high_resolution_clock::now();
        auto delta_t = std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start);
        // std::cout << delta_t.count() <<  "\n" ;
        // i+=1;
        // if(i>100){
        //     std::exit(1);
        // }
    }
    
    // posteriorPose_ = odometry;

    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}

pose_xyt_t ParticleFilter::updateFilterLocalizationUnknown(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map,
                                        particles_converged_t *converged)
{
    
    // std::cout << "YIPPEE\n";
    // std::cout << "HERE5\n";
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    // std::cout << "HERE52\n";
    
    if(hasRobotMoved) // checks to see if the robot has moved 
    {
        std::vector<particle_t> prior;
        if (kLocalizationConverged_){
            prior = resamplePosteriorDistribution(); // resample and re weight --> more particles at high weights 
        }
        else{ // if not converged resample less frequently
            double dist = sqrt(pow(odometry.x - resamplePose_.x,2) + pow(odometry.y - resamplePose_.y,2));
            std::cout << "HERE6\n";
            if (dist >= 0.025){
                std::cout << "HERE61\n";
                prior = resamplePosteriorDistribution();
                resamplePose_ = pose_xyt_t({0, odometry.x, odometry.y, odometry.theta});
                std::cout << "HERE63\n";
            }
            else{
                std::cout << "HERE62\n";
                prior = posterior_;
                std::cout << "HERE64\n";
            }
        }
        std::cout << "HERE7\n";
        // std::cout << "HERE7\n";
        auto proposal = computeProposalDistribution(prior); // applies action model *NO NEED TO CHANGE 
        // std::cout << "HERE8\n";
        posterior_ = computeNormalizedPosterior(proposal, laser, map); // apples sensor model and then normize * FIXED LIELY OOD SO GO 

        // std::cout << "HERE9\n";
        if(kLocalizationConverged_ || localizationConverged(posterior_)){
            // std::cout << "HERE10\n";
            std::cout << "CONVERGED!\n";
            kLocalizationConverged_ = true;
            converged->converged = true;
            posteriorPose_ = estimatePosteriorPose(posterior_); //find final post to update the map * FINE GO TO GO
        }
        else{
            // std::cout << "HERE11\n";
            posteriorPose_ = odometry;
        }
    }
    
    converged->utime = odometry.utime;
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}

pose_xyt_t ParticleFilter::updateFilterActionOnly(const pose_xyt_t&      odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = proposal;
    }
    
    posteriorPose_ = odometry;
    
    return posteriorPose_;
}



pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////

    // std::cout << "HERE61\n";

    std::vector<particle_t> prior; // sets the prior to be used later in code
    //double sampleWeight = 1.0/kNumParticles_; 
    std::random_device rd;
    std::mt19937 generator(rd());

    std::normal_distribution<> rdist(0.0, 1/kNumParticles_);
    // std::normal_distribution<> dist(0.0, 0.04);
    // std::normal_distribution<> small_dist(0.0,0.005);

    // std::cout << "HERE62\n";
    // std::cout << kNumParticles_ << "\n";
    // std::cout << posterior_.size() << "\n";

    double r = rdist(generator);
    double c = posterior_[0].weight;
    long unsigned int i = 0;

    // std::cout << "HERE63\n";
    for (int m = 1; m <= kNumParticles_; m++)
    {
        double U = r + (m-1.0) * 1.0/kNumParticles_;
        while (U > c)
        {
            i = i + 1;
            c = c + posterior_[i].weight;
        }
        // std::cout << "HERE64 " << i << " \n";
        prior.push_back(posterior_[i]);
        // std::cout << "HERE65\n";
    }
    // std::cout << "HERE66\n";
    
    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;
    for(auto & p : prior){ // for each particle in the prior we want to apply the action
        proposal.push_back(actionModel_.applyAction(p));// appply action to all particles 
    }
    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;
    double sumWeights = 0.0;
    for(auto& p: proposal){
        particle_t weighted = p;
        weighted.weight = sensorModel_.likelihood(weighted, laser, map);
        sumWeights += weighted.weight;
        posterior.push_back(weighted);
    }

    for(auto& p: posterior){
        p.weight /= sumWeights;
    }

    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution

    //make a local copy of the particles
    pose_xyt_t pose;

    //make a local copy of the particles
    std::vector<particle_t> posterior_sorted = posterior;
    //Sort the posterior (weighted) distribution
    std::sort(posterior_sorted.begin(), posterior_sorted.end(),
    [](const particle_t& lhs, const particle_t& rhs) -> bool
    {
        return lhs.weight > rhs.weight;
    });

    double xMean = 0.0;
    double yMean = 0.0;
    double cosThetaMean = 0.0;
    double sinThetaMean = 0.0;
    double final_weight=0.0;

    int i = 0;
    for(auto& p: posterior_sorted){
        if (i > posterior_sorted.size()*0.1){
            break;
        }
        xMean += p.weight * p.pose.x;
        yMean += p.weight * p.pose.y;
        cosThetaMean += p.weight * std::cos(p.pose.theta);
        sinThetaMean += p.weight * std::sin(p.pose.theta);
        final_weight +=p.weight;
        i += 1;
    }

    pose.x = xMean/final_weight;
    pose.y = yMean/final_weight;
    pose.theta = std::atan2(sinThetaMean/final_weight, cosThetaMean/final_weight);

    return pose;
}

bool ParticleFilter::localizationConverged(const std::vector<particle_t>& posterior)
{

    std::vector<double> x_dist(posterior.size());
    std::vector<double> y_dist(posterior.size());
    std::vector<double> tsin_dist(posterior.size());
    std::vector<double> tcos_dist(posterior.size());

    int i =0;
    for (auto p: posterior){
        x_dist[i] = p.pose.x;
        y_dist[i] = p.pose.y;
        tsin_dist[i] = std::sin(p.pose.theta);
        tcos_dist[i] = std::cos(p.pose.theta);
        i++;
    }

    double sum_x = std::accumulate(x_dist.begin(), x_dist.end(), 0.0);
    double sum_y = std::accumulate(y_dist.begin(), y_dist.end(), 0.0);
    double sum_tsin = std::accumulate(tsin_dist.begin(), tsin_dist.end(), 0.0);
    double sum_tcos = std::accumulate(tcos_dist.begin(), tcos_dist.end(), 0.0);
    
    double mean_x = sum_x / x_dist.size();
    double mean_y = sum_y / y_dist.size();
    double mean_tsin = sum_tsin / tsin_dist.size();
    double mean_tcos = sum_tcos / tcos_dist.size();

    std::vector<double> diff_x(x_dist.size());
    std::vector<double> diff_y(y_dist.size());
    std::vector<double> diff_tsin(tsin_dist.size());
    std::vector<double> diff_tcos(tcos_dist.size());

    std::transform(x_dist.begin(), x_dist.end(), diff_x.begin(), [mean_x](double l) { return l - mean_x; });
    std::transform(y_dist.begin(), y_dist.end(), diff_y.begin(), [mean_y](double l) { return l - mean_y; });
    std::transform(tsin_dist.begin(), tsin_dist.end(), diff_tsin.begin(), [mean_tsin](double l) { return l - mean_tsin; });
    std::transform(tcos_dist.begin(), tcos_dist.end(), diff_tcos.begin(), [mean_tcos](double l) { return l - mean_tcos; });

    double sq_sum_x = std::inner_product(diff_x.begin(), diff_x.end(), diff_x.begin(), 0.0);
    double sq_sum_y = std::inner_product(diff_y.begin(), diff_y.end(), diff_y.begin(), 0.0);
    double sq_sum_tsin = std::inner_product(diff_tsin.begin(), diff_tsin.end(), diff_tsin.begin(), 0.0);
    double sq_sum_tcos = std::inner_product(diff_tcos.begin(), diff_tcos.end(), diff_tcos.begin(), 0.0);
    
    double stdev_x = std::sqrt(sq_sum_x / x_dist.size());
    double stdev_y = std::sqrt(sq_sum_y / y_dist.size());
    double stdev_tsin = std::sqrt(sq_sum_tsin / tsin_dist.size());
    double stdev_tcos = std::sqrt(sq_sum_tcos / tcos_dist.size());

    double stdev_t = abs(std::atan2(stdev_tsin, stdev_tcos));

    std::cout << stdev_x << ", " << stdev_y << ", " << stdev_t << "\n";

    if (stdev_x < 0.1 && stdev_y < 0.1 && stdev_t < 1) {

        std::vector<particle_t> posterior_sorted = posterior;
        //Sort the posterior (weighted) distribution
        std::sort(posterior_sorted.begin(), posterior_sorted.end(),
        [](const particle_t& lhs, const particle_t& rhs) -> bool
        {
            return lhs.weight > rhs.weight;
        });

        std::vector<particle_t> reduced_posterior;

        int i = 0;
        for(auto& p: posterior_sorted){
            if (i > 200){
                break;
            }
            reduced_posterior.push_back(p);
            i++;
        }

        posterior_ = reduced_posterior;
        kNumParticles_ = 200;

        return true;
    }

    return false;
        
}
