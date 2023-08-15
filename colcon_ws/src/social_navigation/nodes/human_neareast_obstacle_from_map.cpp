#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pedsim_msgs/msg/obstacle_point.hpp"
#include "pedsim_msgs/msg/obstacle_point_array.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"

#include "nav_msgs/msg/get_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "social_navigation/msg/human_state/hpp"
#include "social_navigation/msg/human_states/hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class HumanNearestObstacle: public rclcpp::Node {
    public:
        HumanNearestObstacle(): Node("HumanNearestObstacle"){
            
            // Get the map and store it
            rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr client = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");
            auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
            while (!client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return 0;
                }
                RCLCPP_INFO(this->get_logger, "service not available, waiting again...");
            }

            auto result = client->async_send_request(request);
            // Wait for the result.
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
                }

            map_ = result.get()->map;

            //Publishers


            //Subscribers
            human_sub_ = this->create_subscription<social_navigation::msg::HumanStates>( "/human_states", 2, std::bind(&HumanNearestObstacle::human_callback, this, _1) );

            // Timer callbacks
            timer_ = this->create_wall_timer(100ms, std::bind(&HumanNearestObstacle::run, this));

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

        void human_callback(const social_navigation::msg::HumanStates::SharedPtr msg){
            human_states = msg->states;
        }

        void run(){
            
            for (int i=0; i<num_humans; i++){
                geometry_msgs::msg::Vector3 state = human_states[i];
                unsigned int mx, my;
                worldToMap( state.x, state.y, mx, my );

                // now start moving out and look for nearest cell
                for (int j=0; j<width_; j++){
                    for (int k=0; k<height_; k++){
                        
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
        nav_msgs::msg::OccupancyGrid map_;
        std::vector<social_navigation::msg::HumanState> human_states;

        rclcpp::Subscription<social_navigation::msg::HumanStates>::SharedPtr human_sub_;
        rclcpp:TimerBase::SharedPtr timer_;
        int num_humans = 4;

        float resolution_;
        int width_;
        int height_;
        int num_cells;
        geometry_msgs::msg::Pose origin_;
};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin( std::make_shared<HumanNearestObstacle>() );
    rclcpp::shutdown();
    return 0;
}