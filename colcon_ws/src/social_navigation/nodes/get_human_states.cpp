#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include  "gazebo_msgs/srv/get_entity_state.hpp"
#include "social_navigation_msgs/msg/human_states.hpp"
// #include "social_navigation_msgs/msg/human_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

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

            client = this->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state");
            pub_humans = this->create_publisher<social_navigation_msgs::msg::HumanStates>("/human_states",10);
            std::cout << "hello0" << std::endl;
            // while (!client->wait_for_service(1s)) {
            //     if (!rclcpp::ok()) {
            //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            //     // return 0;
            //     }
            //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            // }
            std::cout << "hello1" << std::endl;
            timer_ = this->create_wall_timer( std::chrono::duration<double>(1.0/update_frequency), std::bind(&HumanStatePublihser::HumanStatePublihserCallback, this) );
        }


    private:
        void HumanStatePublihserCallback(){
            social_navigation_msgs::msg::HumanStates human_states;

            auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();

            int num_humans = 4;
            std::cout << "hello2" << std::endl;
            for (int i=0; i<num_humans; i++){
                std::cout << "hello2_1" << std::endl;
                    request->name="actor"+std::to_string(i+1);
                    request->reference_frame = "world";
                    while (!client->wait_for_service(1s)) {
                                if (!rclcpp::ok()) {
                                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                                // return 0;
                                }
                                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
                            }
                    auto result = client->async_send_request(request);
                    std::cout << "hello3" << std::endl;
                    // std::wait
                    // try{
                    //     std::cout << "hello4" << std::endl;
                    //     int succ = 0;
                    //     if (result.get()->success){
                    //         std::cout << "hello5" << std::endl;
                    //         succ = 1;
                    //         RCLCPP_INFO(this->get_logger(), "Gazebo data received");
                    //     }
                    //     std::cout << "hello6" << std::endl;
                    //     // std::cout <<  "Gazebo data received: " << succ << "\n";
                    // }
                    // catch (const std::exception &e) {
                    //     std::cout << "hello7" << std::endl;
                    //     RCLCPP_ERROR(this->get_logger(), "Failed to call service /gazebo/get_entity_state");
                    // }


                    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
                    {
                        std::cout << "Sent: " << result.get()->success << "\n";
                        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sent " << result.get()->success);
                    } 
                    else {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
                    }



                    std::cout << "hello8" << std::endl;
                    geometry_msgs::msg::Pose pose = result.get()->state.pose;
                    geometry_msgs::msg::Point pos;
                    std::cout << pose.position.x << std::endl;
                    pos.x = pose.position.x; pos.y = pose.position.y; pos.z = pose.position.z;
                    std::cout << "hello9" << std::endl;
                    // pos.state.push_back(pose.position.x); pos.state.push_back(pose.position.y); pos.state.push_back(pose.position);
                    human_states.states.push_back( pos );
            }
            pub_humans->publish(human_states);
        }

        rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client;
        rclcpp::Publisher<social_navigation_msgs::msg::HumanStates>::SharedPtr pub_humans;
        int num_humans = 10;
        float update_frequency = 10;
        rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HumanStatePublihser>());
// //   std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_human_states_node");
//   rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client =
//     node->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/get_entity_state");

//   rclcpp::Publihser<social_navigation::msg::HumanStates>::SharedPtr pub_humans = node->create_publihser<social_navigation::msg::HumanStates>("/human_states",10);
//   rate = node->create_rate(10);

//   while (!client->wait_for_service(1s)) {
//     if (!rclcpp::ok()) {
//       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
//       return 0;
//     }
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
//   }

//   social_navigation::msg::HumanStates human_states;
//   auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
//   int num_humans = 4;
//   for (unsigned int i=0; i<num_humans; i++){
//         request->name="human"+std::to_string(i);
//         request->reference_frame = "world";
//         auto result = client->async_send_request(request);
//         // Wait for the result.
//         if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
//         {
//             std::cout << "Sent: " << result.get()->success << "\n";
//             // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sent " << result.get()->success);
//         } 
//         else {
//             RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
//         }

//         geometry_msgs::msg::Pose pose = result.get().state.pose;
//         std::vector<float> pos{ pose.position.x, pose.position.y, pose.positionz };
//         human_states.push_back( pos );
//   }

  rclcpp::shutdown();
  return 0;
}