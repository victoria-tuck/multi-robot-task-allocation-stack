#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
// #include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
// #include "social_navigation_msgs/msg/robot_closest_obstacles.hpp"
#include "social_navigation_msgs/msg/robot_closest_obstacle.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#define M_PI 3.14157

using namespace std::chrono_literals;
using std::placeholders::_1;

class RobotNearestObstacle: public rclcpp::Node {
    public:
        RobotNearestObstacle(nav_msgs::msg::OccupancyGrid map, std::string name=""): Node("RobotNearestObstacleSectors"){
            std::string prefix;
            if (name != "") {
                prefix = "/" + name;
            } else {
                prefix = "";
            }

            map_ = map;
            
            resolution_ = map_.info.resolution;
            width_ = map_.info.width;
            height_ = map_.info.height;
            origin_ = map_.info.origin;
            num_cells = width_ * height_;
            std::cout << "width: " << width_  << " height:  " << height_ << "Origin: x: " << origin_.position.x << " y: " << origin_.position.y << std::endl;

            //Publishers
            // robot_obstacle_pub_ = this->create_publisher<social_navigation_msgs::msg::RobotClosestObstacle>("/robot1/robot_closest_obstacles", 10);
            // obstacle_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/robot1/robot_closest_obstacles_cloud", 10);
            // human_pose_init_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/robot1/initialpose", 10);

            // //Subscribers
            // robot_sub_ = this->create_subscription<nav_msgs::msg::Odometry>( "/robot1/odom", 2, std::bind(&RobotNearestObstacle::robot_callback, this, _1) );
            robot_obstacle_pub_ = this->create_publisher<social_navigation_msgs::msg::RobotClosestObstacle>(prefix + "/robot_closest_obstacles", 10);
            obstacle_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>(prefix + "/robot_closest_obstacles_cloud", 10);
            human_pose_init_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

            //Subscribers
            robot_sub_ = this->create_subscription<nav_msgs::msg::Odometry>( prefix + "/odom", 2, std::bind(&RobotNearestObstacle::robot_callback, this, _1) );

            // Timer callbacks
            timer_ = this->create_wall_timer(100ms, std::bind(&RobotNearestObstacle::run, this));

        }

        void robot_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
            robot_state = msg->pose.pose;
            robot_state_init = true;


            if (!pose_init) {
                geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
                initial_pose.header.frame_id = "map";
                initial_pose.header.stamp = this->get_clock()->now();
                initial_pose.pose.pose.position.x = robot_state.position.x;
                initial_pose.pose.pose.position.y = robot_state.position.y;
                initial_pose.pose.pose.orientation.z = robot_state.orientation.x;
                initial_pose.pose.pose.orientation.w = robot_state.orientation.w;
                human_pose_init_pub_->publish(initial_pose);
                pose_init = true;
            }            
        }

        void run(){
            if (robot_state_init==false)
                return;

            // std::cout << "hello2" << std::endl;
            // social_navigation_msgs::msg::RobotClosestObstacles robot_obstacle_distances;
            sensor_msgs::msg::PointCloud obstacle_cloud;

            social_navigation_msgs::msg::RobotClosestObstacle obstacle_points;
            obstacle_points.num_obstacles = 0;

            // for (int i=0; i<num_humans; i++){
                
            geometry_msgs::msg::Pose state = robot_state;
            unsigned int mx, my;
            worldToMap( robot_state.position.x, robot_state.position.y, mx, my );

            // std::cout << " robot position " << robot_state.position.x << "\t" << robot_state.position.y << " map pos: " << mx << "\t"  << my << std::endl;

            // search: 
            int max_depth = 10.0;

            float angle_increment = M_PI/6;
            float depth_increment = resolution_+0.01;
            for (float angle=0; angle<2*M_PI; angle=angle+angle_increment){
                for (float d=0; d<max_depth; d=d+depth_increment){
                    geometry_msgs::msg::Point loc;
                    loc.x = robot_state.position.x + cosf(angle) * d;
                    loc.y = robot_state.position.y + sinf(angle) * d;
                    loc.z = robot_state.position.z;
                    unsigned int loc_mx, loc_my;
                    if (worldToMap( loc.x, loc.y, loc_mx, loc_my )){                        
                        if (map_.data.at( loc_my*width_+loc_mx )>0.5){
                            obstacle_points.num_obstacles += 1 ;
                            geometry_msgs::msg::Point32 loc32;
                            loc32.x = loc.x; loc32.y = loc.y; loc32.z = loc.z;
                            obstacle_points.obstacle_locations.push_back(loc);
                            obstacle_cloud.points.push_back(loc32);
                            sensor_msgs::msg::ChannelFloat32 channel;
                            channel.name = "distance";
                            channel.values.push_back( d );
                            obstacle_cloud.channels.push_back(channel);
                            break; // if found an obstacle at this angle, then no need to search more. move on to next angle
                        }
                    }
                    else{ // out of map bounds
                        continue;
                    } 
                }
            }

            obstacle_cloud.header.frame_id = "map";
            obstacle_cloud.header.stamp = this->get_clock()->now();
            obstacle_cloud_pub_->publish(obstacle_cloud);
            robot_obstacle_pub_->publish(obstacle_points);
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

            // std::cout << "mx: " << mx << " my: " << my << " width: " <<  width_ << std::endl;

            if (mx < width_ && my < height_) {
                return true;
            }
            return false;
        }



    private:
        nav_msgs::msg::OccupancyGrid map_;
        geometry_msgs::msg::Pose robot_state;
        bool robot_state_init = false;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_sub_;
        rclcpp::Publisher<social_navigation_msgs::msg::RobotClosestObstacle>::SharedPtr robot_obstacle_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr obstacle_cloud_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr human_pose_init_pub_;

        rclcpp::TimerBase::SharedPtr timer_;
        int num_humans = 4;
        int max_obstacle_points = 10;
        bool pose_init = false;

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



    while (!client->wait_for_service(10s)) {
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

    std::string name1 = "robot1";
    // std::string name1 = "";
    auto node1 = std::make_shared<RobotNearestObstacle>(map_, name1);
    std::string name2 = "robot2";
    auto node2 = std::make_shared<RobotNearestObstacle>(map_, name2);
    std::string name3 = "robot3";
    // std::string name1 = "";
    auto node3 = std::make_shared<RobotNearestObstacle>(map_, name3);
    std::string name4 = "robot4";
    auto node4 = std::make_shared<RobotNearestObstacle>(map_, name4);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node1);
    executor.add_node(node2);
    executor.add_node(node3);
    executor.add_node(node4);

    executor.spin();

    // rclcpp::spin( std::make_shared<RobotNearestObstacle>(map_) );
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