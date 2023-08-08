#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pedsim_msgs/msg/obstacle_point.hpp"
#include "pedsim_msgs/msg/obstacle_point_array.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MapToWorld: public rclcpp::Node {
    public:
        MapToWorld(): Node("MapToWorld"){
                map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",3, std::bind(&MapToWorld::map_callback, this, _1));
                obstacle_point_pub_ = this->create_publisher<pedsim_msgs::msg::ObstaclePointArray>("sfm_obstacle_cloud", 20);
        }

        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map){
            resolution_ = map->info.resolution;
            width_ = map->info.width;
            height_ = map->info.height;
            origin_ = map->info.origin;
            map_data_ = map->data;//.data(); //std::vector<int>(map->data, map->data+num_cells);
            // map_ = *map;
            num_cells = width_ * height_;
        }

        void construct_obstacle_point(){
            pedsim_msgs::msg::ObstaclePointArray points;
            for (int i=0; i<width_; i++){
                for (int j=0; j<height_; j++){
                    if (map_data_.at(j*width_+i)==1){
                        pedsim_msgs::msg::ObstaclePoint point = mapToWorld(i, j);
                        points.points.push_back(point);
                    }
                    
                }
            }
        }

        pedsim_msgs::msg::ObstaclePoint mapToWorld(unsigned int mx, unsigned int my) const{
            pedsim_msgs::msg::ObstaclePoint point;
            point.x = origin_.position.x + (mx + 0.5) * resolution_;
            point.y = origin_.position.y + (my + 0.5) * resolution_;
        }

        bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const {
            if (wx < origin_.position.x || wy < origin_.position.y) {
                return false;
            }

            mx = static_cast<unsigned int>((wx - origin_.position.x) / resolution_);
            my = static_cast<unsigned int>((wy - origin_.position.y) / resolution_);

            if (mx < width_ && my < height_) {
                return true;
            }
            return false;
        }

    private:
        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

        // Publishers
        rclcpp::Publisher<pedsim_msgs::msg::ObstaclePointArray>::SharedPtr obstacle_point_pub_;

        // variables
        float resolution_;
        int width_;
        int height_;
        int num_cells;
        geometry_msgs::msg::Pose origin_;
        // nav_msgs::msg::OccupancyGrid map_;
        std::vector<signed char> map_data_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin( std::make_shared<MapToWorld>() );
    rclcpp::shutdown();
    return 0;
}