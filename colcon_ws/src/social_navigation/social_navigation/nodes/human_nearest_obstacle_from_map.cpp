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
#include "sensor_msgs/msg/point_cloud.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class HumanNearestObstacle: public rclcpp::Node {
    public:
        HumanNearestObstacle(nav_msgs::msg::OccupancyGrid map): Node("HumanNearestObstacle"){
            
            map_ = map;
            resolution_ = map_.info.width;
            width_ = map_.info.width;
            height_ = map_.info.height;
            origin_ = map_.info.origin;
            num_cells = width_ * height_;
            std::cout << "hello" << std::endl;
            //Publishers
            human_obstacle_pub_ = this->create_publisher<social_navigation_msgs::msg::HumanClosestObstacles>("/human_closest_obstacles", 10);
            obstacle_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/closest_obstacles_cloud", 10);

            //Subscribers
            human_sub_ = this->create_subscription<social_navigation_msgs::msg::HumanStates>( "/human_states", 2, std::bind(&HumanNearestObstacle::human_callback, this, _1) );
            std::cout << "hello1" << std::endl;
            // Timer callbacks
            timer_ = this->create_wall_timer(100ms, std::bind(&HumanNearestObstacle::run, this));

        }

        void human_callback(const social_navigation_msgs::msg::HumanStates::SharedPtr msg){
            human_states = msg->states;
            human_states_init = true;
        }

        void run(){
            if (human_states_init==false)
                return;

            std::cout << "hello2" << std::endl;
            social_navigation_msgs::msg::HumanClosestObstacles human_obstacle_distances;
            sensor_msgs::msg::PointCloud obstacle_cloud;

            social_navigation_msgs::msg::HumanClosestObstacle obstacle_points;
            obstacle_points.num_obstacles = 0;

            for (int i=0; i<num_humans; i++){
                
                geometry_msgs::msg::Pose state = human_states[i];
                unsigned int mx, my;
                worldToMap( state.position.x, state.position.y, mx, my );

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
                                        geometry_msgs::msg::Point32 point32;
                                        point32.x = point.x; point32.y = point.y; point32.z = point.z;
                                        obstacle_points.obstacle_locations.push_back(point);
                                        obstacle_cloud.points.push_back(point32);
 
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
            std::cout << "hello3" << std::endl;
            // publish results
            obstacle_cloud.header.frame_id = "world";
            obstacle_cloud.header.stamp = this->get_clock()->now();
            obstacle_cloud_pub_->publish(obstacle_cloud);
            human_obstacle_pub_->publish(human_obstacle_distances);
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
        std::vector<geometry_msgs::msg::Pose> human_states;
        bool human_states_init = false;

        rclcpp::Subscription<social_navigation_msgs::msg::HumanStates>::SharedPtr human_sub_;
        rclcpp::Publisher<social_navigation_msgs::msg::HumanClosestObstacles>::SharedPtr human_obstacle_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr obstacle_cloud_pub_;

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

    // Get the map and store it
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_map_client");
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr client = node->create_client<nav_msgs::srv::GetMap>("/map_server/map");
    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        // return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: ");//%ld", result.get()->sum);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
        }
    nav_msgs::msg::OccupancyGrid map_ = result.get()->map;

    rclcpp::spin( std::make_shared<HumanNearestObstacle>(map_) );
    rclcpp::shutdown();
    return 0;
}



 // Get the map and store it
// rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr client = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");
// auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
// while (!client->wait_for_service(1s)) {
//     if (!rclcpp::ok()) {
//     RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
//     // return 0;
//     }
//     RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
// }

// auto result = client->async_send_request(request);
// // Wait for the result.
// if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS){
//         RCLCPP_INFO(this->get_logger(), "Sum: ");//%ld", result.get()->sum);
//     } else {
//         RCLCPP_ERROR(this->get_logger(), "Failed to call service add_two_ints");
//     }