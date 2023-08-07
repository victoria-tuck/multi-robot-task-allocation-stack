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

#include <pedsim_sensors/occlusion_point_cloud.h>
#include <pedsim_utils/geometry.h>
#include <math.h>
#include <random>
#define INF 100000000
using namespace std::chrono_literals;
using std::placeholders::_1;

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

namespace pedsim_ros {

using Cell = std::complex<float>;

PointCloud::PointCloud(std::string node_name)//,const double rate, const int resol, const FoVPtr& fov)
    : PedsimSensor(node_name) { //, rate, fov) {
  
  int sensor_resol = 360;
  this->declare_parameter<int>("resol", 360);
  this->get_parameter("resol", sensor_resol);
  resol_ = sensor_resol;

  pub_signals_local_ = this->create_publisher<sensor_msgs::msg::PointCloud>("point_cloud_local", 1);
  pub_signals_global_ = this->create_publisher<sensor_msgs::msg::PointCloud>("point_cloud_global", 1);

  sub_simulated_obstacles_ = this->create_subscription<pedsim_msgs::msg::LineObstacles>("/pedsim_simulator/simulated_walls", 1, std::bind(&PointCloud::obstaclesCallBack, this, _1));
  sub_simulated_agents_ = this->create_subscription<pedsim_msgs::msg::AgentStates>("/pedsim_simulator/simulated_agents", 1, std::bind(&PointCloud::agentStatesCallBack, this, _1));

  RCLCPP_INFO_STREAM(this->get_logger(), "Initialized occlusion PCD sensor with center: (" << init_x << ", " << init_y << ") , range: " << fov_range << ", resolution: " << sensor_resol);

  rclcpp::TimerBase::SharedPtr timer_ = this->create_wall_timer( std::chrono::duration<double>(1/rate_), std::bind(&PointCloud::run, this) );
}

uint PointCloud::rad_to_index(float rad_){
  auto rad = atan2(sin(rad_), cos(rad_));
  return (rad + M_PI) * resol_ / (2 * M_PI);
}

float PointCloud::index_to_rad(uint index){
  return -M_PI + 2 * M_PI / resol_ * index;
}

uint PointCloud::fit_index(int index) {
  if (index < 0) {
    return index % resol_ + resol_;
  }
  else{
    return index % resol_; 
  }
}

void PointCloud::fillDetectedObss(std::vector<Cell>& detected_obss, Cell cell, float width){
  Cell obs = cell - Cell(fov_->origin_x, fov_->origin_y);
  float edge_x = -width * sgn(obs.real());
  float edge_y = -width * sgn(obs.imag());
  for (float dw = -width; dw <= width; dw += width / 10) {
    for (auto new_obs : {obs + Cell(edge_x, dw), obs + Cell(dw, edge_y)}){
      auto r = std::abs(new_obs);
      auto theta = std::arg(new_obs);
      uint rad_index = rad_to_index(theta);
      // fill adj indexes with same values
      for (int i : {-2, -1, 0, 1, 2}){
        uint index = fit_index(rad_index + i);
        if (r < std::abs(detected_obss[index])) {
          auto polar = std::polar(r, theta);
          detected_obss[index] = polar;
        }
      }
    }
  }
}

void PointCloud::broadcast() {

  std::vector<Cell> detected_obss(resol_, Cell(INF, INF));

  // obstacles 
  if (q_obstacles_.size() < 1 || q_agents_.size() < 1) {
    return;
  }

  // fill by cells
  std::vector<std::pair<float, float>> all_cells;
  const auto sim_obstacles = q_obstacles_.front();
  for (const auto& line : sim_obstacles->obstacles) {
    const auto cells = pedsim::LineObstacleToCells(line.start.x, line.start.y,
                                                   line.end.x, line.end.y);
    std::copy(cells.begin(), cells.end(), std::back_inserter(all_cells));
  }
  for (const auto& pair_cell : all_cells) {
    // convert pair to complex
    auto cell = Cell(pair_cell.first, pair_cell.second);
    fillDetectedObss(detected_obss, cell, obs_width);
  }

  // fill by agents
  const auto people_signal = q_agents_.front();
  for (const auto& person : people_signal->agent_states) {
    Cell cell(person.pose.position.x, person.pose.position.y);
    fillDetectedObss(detected_obss, cell, human_width);
  }

  constexpr int point_density = 10;
  const int num_points = detected_obss.size() * point_density;

  std::default_random_engine generator;

  // \todo - Read params from config file.
  std::uniform_int_distribution<int> color_distribution(1, 255);
  std::uniform_real_distribution<float> height_distribution(0, 1);
  std::uniform_real_distribution<float> width_distribution(-0.05, 0.05);

  sensor_msgs::msg::PointCloud pcd_global;
  pcd_global.header.stamp = this->get_clock()->now();
  pcd_global.header.frame_id = sim_obstacles->header.frame_id;
  pcd_global.points.resize(num_points);
  pcd_global.channels.resize(1);
  pcd_global.channels[0].name = "intensities";
  pcd_global.channels[0].values.resize(num_points);

  sensor_msgs::msg::PointCloud pcd_local;
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
  for (const auto& obs : detected_obss) {
    Cell cell = obs + Cell(fov_->origin_x, fov_->origin_y);
    const int cell_color = color_distribution(generator);

    for (size_t j = 0; j < point_density; ++j) {
      if (fov_->inside(cell.real(), cell.imag())) {
        geometry_msgs::msg::Vector3 point;
            point.x = cell.real() + width_distribution(generator);
            point.y = cell.imag() + width_distribution(generator);
            point.z = 0.;
        // const auto transformed_point = transformPoint(robot_transform, point);
        geometry_msgs::msg::Vector3 transformed_point;
        // tf2::doTransform( point, transformed_point, robot_transform );

        pcd_local.points[index].x = transformed_point.x;
        pcd_local.points[index].y = transformed_point.y;
        pcd_local.points[index].z = height_distribution(generator);
        pcd_local.channels[0].values[index] = cell_color;

        // Global observations.
        pcd_global.points[index].x = cell.real() + width_distribution(generator);
        pcd_global.points[index].y =
            cell.imag() + width_distribution(generator);
        pcd_global.points[index].z = height_distribution(generator);
        pcd_global.channels[0].values[index] = cell_color;
      }

      index++;
    }
  }

  if (pcd_local.channels[0].values.size() > 1) {
    pub_signals_local_->publish(pcd_local);
  }
  if (pcd_global.channels[0].values.size() > 1) {
    pub_signals_global_->publish(pcd_global);
  }

  q_obstacles_.pop();
  q_agents_.pop();
};

void PointCloud::run() {
    broadcast();
  }

void PointCloud::obstaclesCallBack(
    const pedsim_msgs::msg::LineObstacles::SharedPtr obstacles) {
  q_obstacles_.emplace(obstacles);
}

void PointCloud::agentStatesCallBack(
    const pedsim_msgs::msg::AgentStates::SharedPtr agents) {
  q_agents_.emplace(agents);
}

}  // namespace pedsim_ros

// --------------------------------------------------------------

int main(int argc, char** argv) {
  rclcpp::init(argc, argv); //, "pedsim_occlusion_sensor");
  std::string node_name = "pedsim_occlusion_sensor";
  rclcpp::spin(std::make_shared<pedsim_ros::PointCloud>(node_name));
  
  return 0;
}
