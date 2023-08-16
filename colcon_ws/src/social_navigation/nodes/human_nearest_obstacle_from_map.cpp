#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
// #include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "social_navigation_msgs/msg/human_state.hpp"
#include "social_navigation_msgs/msg/human_states.hpp"
#include "social_navigation_msgs/msg/human_closest_obstacles.hpp"

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
                // return 0;
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }

            auto result = client->async_send_request(request);
            // Wait for the result.
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS){
                    RCLCPP_INFO(this->get_logger(), "Sum: ");//%ld", result.get()->sum);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to call service add_two_ints");
                }

            map_ = result.get()->map;
            resolution_ = map_.info.width;
            width_ = map_.info.width;
            height_ = map_.info.height;
            origin_ = map_.info.origin;
            num_cells = width_ * height_;

            //Publishers


            //Subscribers
            human_sub_ = this->create_subscription<social_navigation_msgs::msg::HumanStates>( "/human_states", 2, std::bind(&HumanNearestObstacle::human_callback, this, _1) );

            // Timer callbacks
            timer_ = this->create_wall_timer(100ms, std::bind(&HumanNearestObstacle::run, this));

        }

        void human_callback(const social_navigation_msgs::msg::HumanStates::SharedPtr msg){
            human_states = msg->states;
        }

        void run(){
            
            social_navigation_msgs::msg::HumanClosestObstacles human_obstacle_distances;
            for (int i=0; i<num_humans; i++){
                social_navigation_msgs::msg::HumanClosestObstacle obstacle_points;
                obstacle_points.num_obstacles = 0;
                geometry_msgs::msg::Point state = human_states[i];
                unsigned int mx, my;
                worldToMap( state.x, state.y, mx, my );

                // search: 
                    // put a limit on distance: maybe 3 meters?
                    int max_depth = 3.0 / resolution_;

                    for (int d=0; d<max_depth; d++){
                        // fopr depth d, size of square is 2*d+1
                        for (int j=-d; j<d; j++){
                            for (int k=-d; k<d; k++){
                                int index_x = mx - j;
                                int index_y = my - k;
                                if ((index_x<mx) & (index_y<my) & (index_x>=0) & (index_y>=0)){
                                    if (map_.data.at( index_y*width_+index_x )>0.5){  // found nearest obstacle
                                        obstacle_points.num_obstacles += 1 ;
                                        geometry_msgs::msg::Point point = mapToWorld( index_x, index_y );  // convert to world
                                        obstacle_points.obstacle_locations.push_back(point);
 
                                        goto search_end;
                                    }
                                }
                            }
                        }

                        // if reach here, then ignore collision avoidance with this human
                    }

                    search_end:
                        human_obstacle_distances.obstacle_info.push_back(obstacle_points);
                        // do nothing

            }

            // publish results

        }          

        geometry_msgs::msg::Point mapToWorld(unsigned int mx, unsigned int my) const{
            geometry_msgs::msg::Point point;
            point.x = origin_.position.x + (mx + 0.5) * resolution_;
            point.y = origin_.position.y + (my + 0.5) * resolution_;
            point.z = 0;
            return point;
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
        std::vector<geometry_msgs::msg::Point> human_states;

        rclcpp::Subscription<social_navigation_msgs::msg::HumanStates>::SharedPtr human_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
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