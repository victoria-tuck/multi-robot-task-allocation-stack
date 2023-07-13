#include <hateb_local_planner/hateb_local_planner_ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>


int main(int argc, char** argv){
  ros::init(argc, argv, "hateb_local_planner");
  ros::NodeHandle n("~");

  tf2_ros::Buffer tf2(ros::Duration(10),true);
  tf2_ros::TransformListener tfListener(tf2);
  costmap_2d::Costmap2DROS costmap("local_costmap", tf2);

  hateb_local_planner::HATebLocalPlannerROS tp;
  tp.initialize("hateb_local_planner_test", &tf2, &costmap);

  ros::spin();

  return 0;
}
