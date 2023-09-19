#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "social_navigation_msgs/msg/human_states.hpp"
#include "social_navigation_msgs/msg/human_closest_obstacles.hpp"
#include "builtin_interfaces/msg/time.hpp"
// Social Force Model
#include <lightsfm/sfm.hpp>
#define PI 3.14167

using namespace std::chrono_literals;
using std::placeholders::_1;

struct waypoint{
    float x;
    float y;
};

struct human{
    std::string name;
    int id;
    // geometry_msgs::msg::Pose pose;
    // geometry_msgs::msg::Twist;
    std::vector<int> in_group_ids;
    std::vector<waypoint> waypoints;
    int curr_waypoint_id;
    float speed;
};

class HumansSFM: public rclcpp::Node {
    public:
        HumansSFM(): Node("HumansSFM"){

            //variables
            t = this->get_clock()->now();

            // Suscribers: get humans and their nearest obstacles. get robot too
            human_states_sub_ = this->create_subscription<social_navigation_msgs::msg::HumanStates>("/human_states", 10, std::bind(&HumansSFM::human_pose_callback, this, _1));
            human_closest_obstacles_sub_ = this->create_subscription<social_navigation_msgs::msg::HumanClosestObstacles>("/human_closest_obstacle", 10, std::bind(&HumansSFM::closest_obstacle_callback, this, _1));

            //Publishers: control humans


            // Read configuration from a config file. human goals, groups, sfm weights for cohesion, replusion

            // Initialize actors
            for (int i=0; i<num_humans; i++){
                sfm::Agent actor;
                actor.desiredVelocity = 0.5;
                actor.params.forceFactorDesired = 0.5;
                actor.params.forceFactorObstacle = 0.5;
                actor.params.forceFactorSocial = 0.5;
                actor.params.forceFactorGroupGaze = 0.5;
                actor.params.forceFactorGroupCoherence = 0.5;
                actor.params.forceFactorGroupRepulsion = 0.5;
                int max_distance = 5.0;
                

                human config;
                config.name = "actor" + std::to_string(i);
                config.id = i;
                // config.in_group_ids.push_back(-1);
                // config.waypoints.push_back();
                config.curr_waypoint_id = 0;

                sfm::Goal goal;
                // goal.center.set(g.X(), g.Y());
                goal.radius = 0.3;
                actor.goals.push_back(goal);

                sfmActors.push_back(actor);
            }
        }


    private:

        void closest_obstacle_callback(const social_navigation_msgs::msg::HumanClosestObstacles::SharedPtr msg){
            human_closest_obstacles = *msg;
        }

        // assume geometry_msgs Pose from now on
        void human_pose_callback(const social_navigation_msgs::msg::HumanStates::SharedPtr msg){
            for (int i=0; i<num_humans; i++){
                sfmActors[i].position.set(msg->states[i].x, msg->states[i].y);
                sfmActors[i].yaw  = utils::Angle::fromRadian(0.0); // TODO: get actual yaw angle
                sfmActors[i].velocity.set( msg->velocities[i].linear.x, msg->velocities[i].linear.y );
                sfmActors[i].linearVelocity = sqrt(msg->velocities[i].linear.x * msg->velocities[i].linear.x + msg->velocities[i].linear.y * msg->velocities[i].linear.y);
                sfmActors[i].angularVelocity = msg->velocities[i].angular.z; // Length()
            }

            actors_initialized = true;

            for (int i=0; i<num_humans; i++){

                if (obstacles_initialized)
                    HandleObstacles(i);
                
                std::vector<sfm::Agent> otherActors = HandlePedestrians(i);
                // Compute Social Forces
                sfm::SFM.computeForces(sfmActors[i], otherActors);

                // Update model
                t_prev = t;
                t = this->get_clock()->now();
                float dt = (t.nanoseconds() - t_prev.nanoseconds())/1e9;  //(t-t_prev).seconds();
                sfm::SFM.updatePosition(sfmActors[i], dt);
                double yaw = sfmActors[i].yaw.toRadian();
                sfmActors[i].yaw = utils::Angle::fromRadian(wrap_angle(yaw));

            }

           
        }

        double wrap_angle(double& yaw){
            if (yaw > PI){
                return yaw - 2*PI;
            }
            else if (yaw < -PI){
                return yaw + 2*PI;
            }

        }

        void HandleObstacles(int ego_index){
            // get closest obstacle
            for (int i=0; i<human_closest_obstacles.obstacle_info[ego_index].num_obstacles; i++){
                utils::Vector2d ob(human_closest_obstacles.obstacle_info[ego_index].obstacle_locations[0].x, human_closest_obstacles.obstacle_info[ego_index].obstacle_locations[0].x);
                this->sfmActor.obstacles1.push_back(ob);
            }          
        }

        std::vector<sfm::Agent> HandlePedestrians(int ego_index){
            std::vector<sfm::Agent> otherActors;
            for (int i=0; i<num_humans; i++){
                if (i==ego_index)
                    continue;
                if (sqrt(std::pow(sfmActors[i].position.getX()-sfmActors[ego_index].position.getX(),2)+std::pow(sfmActors[i].position.getY()-sfmActors[ego_index].position.getY(),2))<max_human_distance){
                    sfm::Agent ped;
                    ped.id = i;
                    ped.position.set(sfmActors[i].position.getX(), sfmActors[i].position.getY());
                    ped.yaw = utils::Angle::fromRadian(sfmActors[i].yaw.toRadian());

                    ped.radius = sfmActors[i].radius;
                    ped.velocity.set(sfmActors[i].velocity.getX(), sfmActors[i].velocity.getY());
                    ped.linearVelocity = sfmActors[i].linearVelocity;
                    ped.angularVelocity = sfmActors[i].angularVelocity; // Length()

                    // TODO: check if the ped belongs to my group
                    // if (sfmActor[ego_index].groupId != -1) {
                    // std::vector<std::string>::iterator it;
                    // it = find(groupNames.begin(), groupNames.end(), model->GetName());
                    // if (it != groupNames.end())
                    //     ped.groupId = this->sfmActor.groupId;
                    // else
                    //     ped.groupId = -1;
                    // }
                    ped.groupId = -1;
                    otherActors.push_back(ped);
                }     
                
            }
            return otherActors;
        }
 
        sfm::Agent sfmActor;
        std::vector<human> humans_config;
        std::vector<sfm::Agent> sfmActors;
        int num_humans = 4;
        bool actors_initialized = false;
        bool obstacles_initialized = false;
        rclcpp::Time t;
        rclcpp::Time t_prev;
        float max_human_distance = 5.0;
        social_navigation_msgs::msg::HumanClosestObstacles human_closest_obstacles;


        rclcpp::Subscription<social_navigation_msgs::msg::HumanStates>::SharedPtr human_states_sub_;
        rclcpp::Subscription<social_navigation_msgs::msg::HumanClosestObstacles>::SharedPtr human_closest_obstacles_sub_;

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumansSFM>());
    rclcpp::shutdown();
    return 0;
}