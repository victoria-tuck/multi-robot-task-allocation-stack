#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
// #include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "social_navigation_msgs/msg/human_states.hpp"
#include "social_navigation_msgs/msg/human_closest_obstacle.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

#define M_PI 3.14157

using namespace std::chrono_literals;
using std::placeholders::_1;

class HumanNearestObstacle: public rclcpp::Node {
    public:
        HumanNearestObstacle(nav_msgs::msg::OccupancyGrid map): Node("HumanNearestObstacle"){
            
            map_ = map;
            resolution_ = map_.info.resolution;
            width_ = map_.info.width;
            height_ = map_.info.height;
            origin_ = map_.info.origin;
            num_cells = width_ * height_;
            std::cout << "width: " << width_  << " height:  " << height_ << "Origin: x: " << origin_.position.x << " y: " << origin_.position.y << std::endl;


            // read parameters
            this->declare_parameter<int>("num_humans",25);
            this->get_parameter("num_humans", num_humans);

            //Publishers
            human_obstacle_pub_ = this->create_publisher<social_navigation_msgs::msg::HumanClosestObstacle>("/human_closest_obstacles", 10);
            human_single_obstacle_pub_ = this->create_publisher<social_navigation_msgs::msg::HumanClosestObstacle>("/human_single_closest_obstacles", 10);
            obstacle_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/human_closest_obstacles_cloud", 10);

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

            // std::cout << "hello2" << std::endl;
            // social_navigation_msgs::msg::HumanClosestObstacles human_obstacle_points;
            sensor_msgs::msg::PointCloud obstacle_cloud;

            social_navigation_msgs::msg::HumanClosestObstacle obstacle_points;
            social_navigation_msgs::msg::HumanClosestObstacle single_obstacle_points;
            obstacle_points.num_obstacles = 0;

            for (int i=0; i<num_humans; i++){
                
                geometry_msgs::msg::Pose state = human_states[i];
                unsigned int mx, my;
                worldToMap( state.position.x, state.position.y, mx, my );

                // search: 
                    // search: 
                int max_depth = 10.0; //10.0;

                float angle_increment = M_PI/6;
                float depth_increment = resolution_+0.01;
                // std::cout << "pi " << M_PI << "icrement: " << angle_increment << std::endl;
                geometry_msgs::msg::Point single_loc;
                single_loc.x = 100.0;
                single_loc.y = 100.0;
                single_loc.z = 0.0;
                float d_prev = 100.0;
                for (float angle=0; angle<2*M_PI; angle=angle+angle_increment){
                    // std::cout << "hello2.5 " << angle << " " << std::endl;
                    for (float d=0; d<max_depth; d=d+depth_increment){
                        // std::cout << "hello3 " << d << " " << std::endl;
                        geometry_msgs::msg::Point loc;
                        loc.x = state.position.x + cosf(angle) * d;
                        loc.y = state.position.y + sinf(angle) * d;
                        loc.z = 0.008; //state.position.z;
                        unsigned int loc_mx, loc_my;
                        if (worldToMap( loc.x, loc.y, loc_mx, loc_my )){         
                            // std::cout << "hello4 " << d << " " << std::endl;               
                            if (map_.data.at( loc_my*width_+loc_mx )>0.5){
                                // std::cout << "hello5 " << d << " " << std::endl;
                                obstacle_points.num_obstacles += 1 ;
                                geometry_msgs::msg::Point32 loc32;
                                loc32.x = loc.x; loc32.y = loc.y; loc32.z = loc.z;
                                obstacle_points.obstacle_locations.push_back(loc);
                                obstacle_cloud.points.push_back(loc32);
                                sensor_msgs::msg::ChannelFloat32 channel;
                                channel.name = "distance";
                                channel.values.push_back( d );
                                obstacle_cloud.channels.push_back(channel);

                                if (d_prev>d){
                                    d_prev = d;
                                    single_loc.x = loc.x;
                                    single_loc.y = loc.y;
                                }
                                // std::cout << "found point" << std::endl;
                                break; // if found an obstacle at this angle, then no need to search more. move on to next angle
                            }
                        }
                        else{ // out of map bounds
                            continue;
                        } 
                    }
                }

                single_obstacle_points.num_obstacles += 1;
                single_obstacle_points.obstacle_locations.push_back(single_loc);                

            }
            // std::cout << "hello3" << std::endl;
            // publish results
            obstacle_cloud.header.frame_id = "map";
            obstacle_cloud.header.stamp = this->get_clock()->now();
            obstacle_cloud_pub_->publish(obstacle_cloud);
            human_obstacle_pub_->publish(obstacle_points);
            human_single_obstacle_pub_->publish(single_obstacle_points);
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
        rclcpp::Publisher<social_navigation_msgs::msg::HumanClosestObstacle>::SharedPtr human_obstacle_pub_;
        rclcpp::Publisher<social_navigation_msgs::msg::HumanClosestObstacle>::SharedPtr human_single_obstacle_pub_;
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
