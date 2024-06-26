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


using namespace std::chrono_literals;
using std::placeholders::_1;

class RobotNearestObstacle: public rclcpp::Node {
    public:
        RobotNearestObstacle(nav_msgs::msg::OccupancyGrid map, std::string name = ""): Node("RobotNearestObstacle"){
            std::string prefix;
            if (name == "") {
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
            std::cout << "hello" << std::endl;
            //Publishers
            // robot_obstacle_pub_ = this->create_publisher<social_navigation_msgs::msg::RobotClosestObstacle>("/robot1/robot_closest_obstacles", 10);
            // obstacle_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/robot1/robot_closest_obstacles_cloud", 10);

            // //Subscribers
            // robot_sub_ = this->create_subscription<nav_msgs::msg::Odometry>( "/robot1/odom", 2, std::bind(&RobotNearestObstacle::robot_callback, this, _1) );
            robot_obstacle_pub_ = this->create_publisher<social_navigation_msgs::msg::RobotClosestObstacle>(prefix + "/robot_closest_obstacles", 10);
            obstacle_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>(prefix + "/robot_closest_obstacles_cloud", 10);

            //Subscribers
            robot_sub_ = this->create_subscription<nav_msgs::msg::Odometry>( prefix + "/odom", 2, std::bind(&RobotNearestObstacle::robot_callback, this, _1) );
            std::cout << "hello1" << std::endl;
            // Timer callbacks
            timer_ = this->create_wall_timer(100ms, std::bind(&RobotNearestObstacle::run, this));

        }

        void robot_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
            robot_state = msg->pose.pose;
            robot_state_init = true;
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
            int max_depth = 3.0 / resolution_;

            for (int d=0; d<max_depth; d++){                // for depth d, size of square is 2*d+1
                for (int j=-d; j<d; j++){
                    for (int k=-d; k<d; k++){
                        int index_x = mx - j;
                        int index_y = my - k;
                        // std::cout << "hello3" << std::endl;
                        if ((index_x<width_) & (index_y<height_) & (index_x>=0) & (index_y>=0)){
                            // std::cout << "in1" << std::endl;
                            if (map_.data.at( index_y*width_+index_x )>0.5){  // found nearest obstacle
                                obstacle_points.num_obstacles += 1 ;
                                geometry_msgs::msg::Point point = mapToWorld( index_x, index_y );  // convert to world
                                geometry_msgs::msg::Point32 point32;
                                point32.x = point.x; point32.y = point.y; point32.z = point.z;
                                obstacle_points.obstacle_locations.push_back(point);
                                obstacle_cloud.points.push_back(point32);
                                sensor_msgs::msg::ChannelFloat32 channel;
                                channel.name = "distance";
                                channel.values.push_back( sqrt( powf(point.x-robot_state.position.x,2)+powf(point.y-robot_state.position.y,2) ) );
                                obstacle_cloud.channels.push_back(channel);

                                // goto search_end;
                            }

                            if (obstacle_points.num_obstacles>max_obstacle_points)
                                break;
                        }

                        if (obstacle_points.num_obstacles>max_obstacle_points)
                                break;
                    }

                    if (obstacle_points.num_obstacles>max_obstacle_points)
                                break;
                }

                if (obstacle_points.num_obstacles>max_obstacle_points)
                                break;
            }

            // }
            // std::cout << "hello3" << std::endl;
            // publish results
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

            std::cout << "mx: " << mx << " my: " << my << std::endl;

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
        rclcpp::TimerBase::SharedPtr timer_;
        int num_humans = 4;
        int max_obstacle_points = 10;

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

    rclcpp::executors::MultiThreadedExecutor executor;
    for (int i = 1; i <= 1; i++) {
        std::string name = "robot" + std::to_string(i);
        auto node = std::make_shared<RobotNearestObstacle>(map_, name);
        executor.add_node(node);
    }

    executor.spin();

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