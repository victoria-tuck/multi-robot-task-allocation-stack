/**
* Copyright 2014- Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
*/

#include <pedsim_sensors/obstacle_point_cloud.h>
#include <pedsim_utils/geometry.h>

#include <random>

namespace pedsim_ros {

ObstaclePointCloud::ObstaclePointCloud(std::string node_name) //, //const double rate, const FoVPtr& fov)
    : PedsimSensor(node_name) {

      pub_signals_local_ = this->create_publisher<sensor_msgs::PointCloud>("point_cloud_local", 1);
      pub_signals_global_ = this->create_publisher<sensor_msgs::PointCloud>("point_cloud_global", 1);
      sub_simulated_obstacles_ = this->create_subscription<pedsim_msgs::LineObstacles>("/pedsim_simulator/simulated_walls", 1, &ObstaclePointCloud::obstaclesCallBack, this);

     
      RCLCPP_INFO_STREAM(this->get_logger(), "Initialized obstacle PCD sensor with center: (" << init_x << ", " << init_y << ") and range: " << fov_range);

      rclcpp::TimerBase::SharedPtr timer_ = this->create_wall_timer( rclcpp::Duration(1/rate_), std::bind(&ObstaclePointCloud::run, this) );
}

void ObstaclePointCloud::broadcast() {
  if (q_obstacles_.size() < 1) {
    return;
  }

  const auto sim_obstacles = q_obstacles_.front();

  using Cell = std::pair<float, float>;
  std::vector<Cell> all_cells;
  for (const auto& line : sim_obstacles->obstacles) {
    const auto cells = pedsim::LineObstacleToCells(line.start.x, line.start.y,
                                                   line.end.x, line.end.y);
    std::copy(cells.begin(), cells.end(), std::back_inserter(all_cells));
  }

  constexpr int point_density = 100;
  const int num_points = all_cells.size() * point_density;

  std::default_random_engine generator;

  // \todo - Read params from config file.
  std::uniform_int_distribution<int> color_distribution(1, 255);
  std::uniform_real_distribution<float> height_distribution(0, 1);
  std::uniform_real_distribution<float> width_distribution(-0.5, 0.5);

  sensor_msgs::PointCloud pcd_global;
  pcd_global.header.stamp = this->get_clock()->now();
  pcd_global.header.frame_id = sim_obstacles->header.frame_id;
  pcd_global.points.resize(num_points);
  pcd_global.channels.resize(1);
  pcd_global.channels[0].name = "intensities";
  pcd_global.channels[0].values.resize(num_points);

  sensor_msgs::PointCloud pcd_local;
  pcd_local.header.stamp = this->get_clock()->now();
  pcd_local.header.frame_id = robot_odom_.header.frame_id;
  pcd_local.points.resize(num_points);
  pcd_local.channels.resize(1);
  pcd_local.channels[0].name = "intensities";
  pcd_local.channels[0].values.resize(num_points);

  // prepare the transform to robot odom frame.
  geometry_msgs::msg::TransformStamped robot_transform;
  try {
      robot_transform = tf_buffer_->lookupTransform(
        robot_odom_.header.frame_id, sim_obstacles->header.frame_id,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        sim_obstacles->header.frame_id, robot_odom_.header.frame_id, ex.what());
      return;
  }

  size_t index = 0;
  for (const auto& cell : all_cells) {
    const int cell_color = color_distribution(generator);

    for (size_t j = 0; j < point_density; ++j) {
      if (fov_->inside(cell.first, cell.second)) {

        const geometry_msgs::Vector3 point(cell.first + width_distribution(generator),
                                cell.second + width_distribution(generator),
                                0.);
        geometry_msgs::Vector3 transformed_point;
        tf2::do_transform( point, transformed_point, robot_transform )

        pcd_local.points[index].x = transformed_point.x;
        pcd_local.points[index].y = transformed_point.y;
        pcd_local.points[index].z = height_distribution(generator);
        pcd_local.channels[0].values[index] = cell_color;

        // Global observations.
        pcd_global.points[index].x = cell.first + width_distribution(generator);
        pcd_global.points[index].y =
            cell.second + width_distribution(generator);
        pcd_global.points[index].z = height_distribution(generator);
        pcd_global.channels[0].values[index] = cell_color;
      }

      index++;
    }
  }

  if (pcd_local.channels[0].values.size() > 1) {
    pub_signals_local_.publish(pcd_local);
  }
  if (pcd_global.channels[0].values.size() > 1) {
    pub_signals_global_.publish(pcd_global);
  }

  q_obstacles_.pop();
};

void ObstaclePointCloud::run() {
  broadcast();
}

void ObstaclePointCloud::obstaclesCallBack(
    const pedsim_msgs::LineObstaclesConstPtr& obstacles) {
  q_obstacles_.emplace(obstacles);
}

}  // namespace

// --------------------------------------------------------------

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);//, "pedsim_obstacle_sensor");
  // ros::NodeHandle node("~");
  node_name = "pedsim_obstacle_sensor";

  rclcpp::spin(pedsim_ros::ObstaclePointCloud pcd_sensor(node_name));
  rclcpp::shutdown();

  return 0;
}
