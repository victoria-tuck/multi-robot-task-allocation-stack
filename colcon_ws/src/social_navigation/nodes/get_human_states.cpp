#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "social_navigation_msgs/msg/human_states.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

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