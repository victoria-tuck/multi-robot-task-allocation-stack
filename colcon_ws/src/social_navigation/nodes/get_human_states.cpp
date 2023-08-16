#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include  "gazebo_msgs/srv/get_entity_state.hpp"
#include "social_navigation_msgs/msg/human_states.hpp"
// #include "social_navigation_msgs/msg/human_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

// #include "rclcpp/rclcpp.hpp"
// #include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

class HumanStatePublihser : public rclcpp::Node{
    public:
        HumanStatePublihser(): Node("get_human_state_node"){

            this->declare_parameter<int>("num_humans",4);
            this->get_parameter("num_humans", num_humans);
            this->declare_parameter<float>("update_frequency",10);
            this->get_parameter("update_frequency", update_frequency);

            // client = this->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state");
            for (int i=0; i<num_humans; i++){

                geometry_msgs::msg::Pose pose;
                geometry_msgs::msg::Twist velocity;
                bool valid = false;
                human_states_.states.push_back(pose);
                human_states_.velocities.push_back(velocity);
                human_states_.valid.push_back(valid);
            }

            for (int i=0; i<num_humans; i++){
                std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> temp_callback = std::bind(&HumanStatePublihser::human_callback, this, _1, i);
                sub_humans.push_back(this->create_subscription<nav_msgs::msg::Odometry>("/gazebo/human"+std::to_string(i+1), 10, temp_callback ));
                // temp_callbacks.push_back( std::bind(&HumanStatePublihser::human_callback, this, _1, i) );                
                // sub_humans.push_back(this->create_subscription<nav_msgs::msg::Odometry>("/gazebo/human"+std::to_string(i+1), [this, i](const nav_msgs::msg::Odometry::SharedPtr msg){ this->human_callback(msg, i); }, 10) );
            }
           
            pub_humans = this->create_publisher<social_navigation_msgs::msg::HumanStates>("/human_states",10);
            timer_ = this->create_wall_timer( std::chrono::duration<double>(1.0/update_frequency), std::bind(&HumanStatePublihser::HumanStatePublihserCallback, this) );
        }


    private:

        void human_callback(const nav_msgs::msg::Odometry::SharedPtr msg, const int index){
            // int index = 0;
            // std::cout << "Human " << index << std::endl;
            human_states_.states[index] = msg->pose.pose;
            human_states_.velocities[index] = msg->twist.twist;
            human_states_.valid[index] = true;
        }

        void HumanStatePublihserCallback(){
            social_navigation_msgs::msg::HumanStates human_states_local;

            for (int i=0; i<num_humans; i++){
                if (human_states_.valid[i] == false)
                    return;
                human_states_local.states.push_back(human_states_.states[i]);
                human_states_local.velocities.push_back(human_states_.velocities[i]);
                human_states_local.valid.push_back(human_states_.valid[i]);
            }
            pub_humans->publish(human_states_local);
        }

        rclcpp::Publisher<social_navigation_msgs::msg::HumanStates>::SharedPtr pub_humans;
        std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> sub_humans;
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_human;
        int num_humans = 4;
        float update_frequency = 10;
        rclcpp::TimerBase::SharedPtr timer_;
        social_navigation_msgs::msg::HumanStates human_states_;
        // std::vector<std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)>> temp_callbacks;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HumanStatePublihser>());
  rclcpp::shutdown();
  return 0;
}


// void HumanStatePublihserCallback(){
//             social_navigation_msgs::msg::HumanStates human_states;

//             auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();

//             int num_humans = 4;
//             std::cout << "hello2" << std::endl;
//             for (int i=0; i<num_humans; i++){
//                 std::cout << "hello2_1" << std::endl;
//                     request->name="actor"+std::to_string(i+1);
//                     request->reference_frame = "world";
//                     while (!client->wait_for_service(1s)) {
//                                 if (!rclcpp::ok()) {
//                                 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
//                                 // return 0;
//                                 }
//                                 RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
//                             }
//                     auto result = client->async_send_request(request);
//                     std::cout << "hello3" << std::endl;
//                     // std::wait
//                     // try{
//                     //     std::cout << "hello4" << std::endl;
//                     //     int succ = 0;
//                     //     if (result.get()->success){
//                     //         std::cout << "hello5" << std::endl;
//                     //         succ = 1;
//                     //         RCLCPP_INFO(this->get_logger(), "Gazebo data received");
//                     //     }
//                     //     std::cout << "hello6" << std::endl;
//                     //     // std::cout <<  "Gazebo data received: " << succ << "\n";
//                     // }
//                     // catch (const std::exception &e) {
//                     //     std::cout << "hello7" << std::endl;
//                     //     RCLCPP_ERROR(this->get_logger(), "Failed to call service /gazebo/get_entity_state");
//                     // }


//                     if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
//                     {
//                         std::cout << "Sent: " << result.get()->success << "\n";
//                         // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sent " << result.get()->success);
//                     } 
//                     else {
//                         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
//                     }



//                     std::cout << "hello8" << std::endl;
//                     geometry_msgs::msg::Pose pose = result.get()->state.pose;
//                     geometry_msgs::msg::Point pos;
//                     std::cout << pose.position.x << std::endl;
//                     pos.x = pose.position.x; pos.y = pose.position.y; pos.z = pose.position.z;
//                     std::cout << "hello9" << std::endl;
//                     // pos.state.push_back(pose.position.x); pos.state.push_back(pose.position.y); pos.state.push_back(pose.position);
//                     human_states.states.push_back( pos );
//             }
//             pub_humans->publish(human_states);
//         }