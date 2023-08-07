#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

#include "geometry_msgs/msg/transform_stamped.hpp"

// TODO
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>



/// Simulates robot motion of a differential-drive robot with translational and
/// rotational velocities as input
/// These are provided in the form of a geometry_msgs::Twist, e.g. by
/// turtlebot_teleop/turtlebot_teleop_key.
/// The resulting robot position is published as a TF transform from world -->
/// base_footprint frame.
class robotUpdate: public rclcpp::Node
{
  public:

    double g_updateRate, g_simulationFactor;
    std::string g_worldFrame, g_robotFrame;
    geometry_msgs::msg::Twist g_currentTwist;
    geometry_msgs::msg::TransformStamped g_currentPose;
    std::unique_ptr<tf2_ros::TransformBroadcaster> g_transformBroadcaster;
    boost::mutex mutex;


    robotUpdate(): Node("simulate_diff_drive_robot" ){
      // do nothing

      // Process parameters
      this->declare_parameter<std::string>("world_frame", "odom");
      this->get_parameter("world_frame", g_worldFrame);
      this->declare_parameter<std::string>("robot_frame", "base_footprint");
      this->get_parameter("robot_frame", g_robotFrame);

      this->declare_parameter<double>("/pedsim_simulator/simulation_factor",1.0);  // set to e.g. 2.0 for 2x speed
      this->get_parameter("/pedsim_simulator/simulation_factor", g_simulationFactor);
      this->declare_parameter<double>("/pedsim_simulator/update_rate", 25.0);  // in Hz
      this->get_parameter("/pedsim_simulator/update_rate", g_updateRate);

      double initialX = 0.0, initialY = 0.0, initialTheta = 0.0;
      this->declare_parameter<double>("pose_initial_x", 0.0);
      this->get_parameter("pose_initial_x", initialX);
      this->declare_parameter<double>("pose_initial_y", 0.0);
      this->get_parameter("pose_initial_y", initialY);
      this->declare_parameter<double>("pose_initial_theta", 0.0);
      this->get_parameter("pose_initial_theta", initialTheta);

      g_currentPose.transform.translation.x = initialX;
      g_currentPose.transform.translation.y = initialY;
      tf2::Quaternion q;
      q.setRPY(0, 0, initialTheta);
      g_currentPose.transform.rotation = tf2::toMsg(q);

      g_transformBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twistSubscriber = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 3, std::bind(&robotUpdate::onTwistReceived, this, std::placeholders::_1));

      // Run
      // boost::thread updateThread(updateLoop);

      rclcpp::TimerBase::SharedPtr timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0/g_updateRate), std::bind(&robotUpdate::updateLoop, this));


    }

  private:
   

    void updateLoop() {
        const double dt = g_simulationFactor / g_updateRate;

        double x = g_currentPose.transform.translation.x;
        double y = g_currentPose.transform.translation.y;
        double theta = tf2::getYaw(g_currentPose.transform.rotation);

        // Get requested translational and rotational velocity
        double v, omega;
        {
          boost::mutex::scoped_lock lock(mutex);
          v = g_currentTwist.linear.x;
          omega = g_currentTwist.angular.z;
        }

        // Simulate robot movement
        x += cos(theta) * v * dt;
        y += sin(theta) * v * dt;
        theta += omega * dt;

        // Update pose
        g_currentPose.transform.translation.x = x;
        g_currentPose.transform.translation.y = y;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        g_currentPose.transform.rotation = tf2::toMsg(q); //createQuaternionMsgFromRPY(0, 0, theta));

        g_currentPose.header.stamp = this->get_clock()->now();
        g_currentPose.header.frame_id = g_worldFrame;
        g_currentPose.child_frame_id = g_robotFrame;
        g_transformBroadcaster->sendTransform(g_currentPose);
    }

    void onTwistReceived(const geometry_msgs::msg::Twist::SharedPtr twist);
  
};

void robotUpdate::onTwistReceived(const geometry_msgs::msg::Twist::SharedPtr twist) {
      boost::mutex::scoped_lock lock(mutex);
      g_currentTwist = *twist;
}






int main(int argc, char** argv) {
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<robotUpdate>());
}
