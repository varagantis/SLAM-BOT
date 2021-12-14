#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    if(!initialized_){
        previousPose_ = pose;
        initialized_ = true;
    }

    MovingLaserScan movingscan(scan, previousPose_, pose);

    for(auto& ray : movingscan){
        scoreEndpoint(ray, map);
        scoreRay(ray, map);
    }

    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map){
    if(ray.range < kMaxLaserDistance_){
        Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
        Point<int> rayCell;

        rayCell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
        rayCell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);
    
        if(map.isCellInGrid(rayCell.x, rayCell.y)){
            increaseCellOdds(rayCell.x, rayCell.y, map);
        }
    }
}
void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map){
    Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
    Point<int> rayEnd;

    rayEnd.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
    rayEnd.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

    int dx=abs(rayEnd.x-static_cast<int>(rayStart.x));
    int dy=abs(rayEnd.y-static_cast<int>(rayStart.y));
    int sx=static_cast<int>(rayStart.x)<rayEnd.x ? 1:-1;
    int sy=static_cast<int>(rayStart.y)<rayEnd.y ? 1:-1;
    float error= dx-dy;
    int x=static_cast<int>(rayStart.x);
    int y=static_cast<int>(rayStart.y);

    while(x != rayEnd.x || y != rayEnd.y ){
        if(map.isCellInGrid(x, y)){
            decreaseCellOdds(x, y, map);
        }
        int e2=2*error;
        if(e2 >= -dy){
            error -= dy;
            x +=sx;
        }
        if(e2 <= dx){
            error += dx;
            y +=sy;
        }
    }

}
void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map){
    
    if(std::numeric_limits<CellOdds>::min() - map(x,y) < -1*kMissOdds_){
        map(x,y) -= kMissOdds_;
    }
    else {
        map(x,y) = std::numeric_limits<CellOdds>::min();
    }

}
void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map){
    
    if(std::numeric_limits<CellOdds>::max() - map(x,y) > kHitOdds_){
        map(x,y) += kHitOdds_;
    }
    else {
        map(x,y) = std::numeric_limits<CellOdds>::max();
    }
}
