#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include  "gazebo_msgs/srv/set_entity_state.hpp"
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

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_human_states_node");
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client =
    node->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/get_entity_state");


  int num_humans = 4;
  for (unsigned int i=0; i<num_humans; i++){
    
  }


  auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
  request->state.name="animated_box";
    request->state.pose.position.x=4.0;
    request->state.pose.position.y=13.0;
    request->state.pose.position.z=0.0;
    request->state.pose.orientation.x=0.00;
    request->state.pose.orientation.y=0.00;
    request->state.pose.orientation.z=0.00;
    request->state.pose.orientation.w=1.00;
    request->state.twist.linear.x=0.00;
    request->state.twist.linear.y=0.00;
    request->state.twist.linear.z=0.00;
    request->state.twist.angular.x=0.00;
    request->state.twist.angular.y=0.00;
    request->state.twist.angular.z=0.00; 
    request->state.reference_frame="world";

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    std::cout << "Sent: " << result.get()->success << "\n";
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sent " << result.get()->success);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}