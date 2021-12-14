#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{

    // look for max range and min range of the lazor scan aka if statement 
    // need to make more rebust 
    
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);// moves scan to particles
    double scanScore = 0.0; // sets up score for particle 

    for(auto& ray : movingScan){// go through every ray to check to see where the lazor hits?
        if(ray.range<6.0){ // check to see if the scan is valid
            Point<double> endpoint(ray.origin.x + ray.range * std::cos(ray.theta),
               ray.origin.y + ray.range * std::sin(ray.theta));    
            Point<double> startpoint(ray.origin.x, ray.origin.y);                
            auto rayEnd = global_position_to_grid_position(endpoint, map); // find global x y laser end

            double ray_actual=ray.range;

            if(map.logOdds(rayEnd.x, rayEnd.y) > 2.0){ //hit
                scanScore += map.logOdds(rayEnd.x, rayEnd.y);
            }
            else{ 

                double cell_size=0.05;
                

            // check far 
                double ray_long=ray_actual+cell_size/(std::sin(ray.theta)); // increase ray length

                Point<double> endpoint_far(ray.origin.x + ray_long * std::cos(ray.theta),
                ray.origin.y + ray_long * std::sin(ray.theta));                      
                rayEnd = global_position_to_grid_position(endpoint_far, map); // find global x y laser 
                if(map.logOdds(rayEnd.x, rayEnd.y) > 2.0){
                    scanScore += 0.30*map.logOdds(rayEnd.x, rayEnd.y);
                }

                double ray_long_2=ray_actual+2*cell_size/(std::sin(ray.theta)); // increase ray length
                 Point<double> endpoint_far2(ray.origin.x + ray_long_2 * std::cos(ray.theta),
                ray.origin.y + ray_long_2 * std::sin(ray.theta));                      
                rayEnd = global_position_to_grid_position(endpoint_far2, map); // find global x y laser 
                if(map.logOdds(rayEnd.x, rayEnd.y) > 2.0){
                    scanScore += 0.12*map.logOdds(rayEnd.x, rayEnd.y);
                }


                double ray_short=ray_actual-cell_size/(std::sin(ray.theta)); // decrease ray length
                Point<double> endpoint_close (ray.origin.x + ray_short * std::cos(ray.theta),
                    ray.origin.y + ray_short * std::sin(ray.theta));                      
                rayEnd = global_position_to_grid_position(endpoint_close, map); // find global x y laser 
                if(map.logOdds(rayEnd.x, rayEnd.y) > 2.0){
                    scanScore += 0.30*map.logOdds(rayEnd.x, rayEnd.y);
                }

                double ray_short_2=ray_actual-2*cell_size/(std::sin(ray.theta)); // decrease ray length
                 Point<double> endpoint_close2 (ray.origin.x + ray_short_2 * std::cos(ray.theta),
                    ray.origin.y + ray_short_2 * std::sin(ray.theta));                      
                rayEnd = global_position_to_grid_position(endpoint_close2, map); // find global x y laser 
                if(map.logOdds(rayEnd.x, rayEnd.y) > 2.0){
                    scanScore += 0.13*map.logOdds(rayEnd.x, rayEnd.y);
                }


            }

        }
    }
    return scanScore;

}