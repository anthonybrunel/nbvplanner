/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nbvplanner/nbvp_srv.h>
#include "../../../oscar/utils_headers/poly_trajectory_rpg.h"
#include <tf2/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include "../../../oscar/utils_headers/plannerlogging.h"

enum State{
    NOTREADY,
    TAKEOFF,
    TRAJ,
    IDLE
};
State state = NOTREADY;
PlannerLogging planner_logger_;

bool odomReady = false;

Eigen::Affine3d pose;
Eigen::Vector3d vel;

ros::Publisher saved_paths_pub_;

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> saved_path_ompl_;

void uavOdomCallback(const nav_msgs::OdometryConstPtr &odom_i){
    odomReady =true;
    tf::poseMsgToEigen(odom_i->pose.pose, pose);
    tf::vectorMsgToEigen(odom_i->twist.twist.linear, vel);
    double speed = vel.norm();

    planner_logger_.log_dist_speed(speed,pose.translation());
}

void sendGoal(PolyTrajInterface &traj_interface,const Eigen::Vector4d &start, const Eigen::Vector4d &vel_start, const Eigen::Vector4d &goal){
    mav_trajectory_generation::Trajectory traj;
    traj_interface.computeTrajectoryAndYaw(start,vel_start,goal,Eigen::Vector4d::Zero(),std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >()
                                           ,&traj);
    traj_interface.setTrajectory(traj);
    traj_interface.startTrajectory();
    state=TRAJ;
}
void sendGoal(PolyTrajInterface &traj_interface,const Eigen::Vector4d &start, const Eigen::Vector4d &vel_start, const Eigen::Vector4d &goal,
              const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > &wps){
    mav_trajectory_generation::Trajectory traj;
    traj_interface.computeTrajectoryAndYaw(start,vel_start,goal,Eigen::Vector4d::Zero(),wps,&traj);
    traj_interface.setTrajectory(traj);
    traj_interface.startTrajectory();
    state=TRAJ;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "exploration");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Subscriber sub_odom_     =
            nh.subscribe<nav_msgs::Odometry>("odometry", 1, uavOdomCallback);

    ROS_INFO("Started exploration");

    std_srvs::Empty srv;
    std::string log_path_ ="";
    nh_private.param("planner/save_log_folder", log_path_, log_path_);

    std::string ns = ros::this_node::getName();


    static int n_seq = 0;
    PolyTrajInterface traj_interface(nh,nh_private);


    trajectory_msgs::MultiDOFJointTrajectory samples_array;
    mav_msgs::EigenTrajectoryPoint trajectory_point;
    trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;

    // Wait for 10 seconds to let the Gazebo GUI show up.
    ros::Duration(6.0).sleep();

    ros::Rate loop(100);
    // This is the initialization motion, necessary that the known free space allows the planning
    // of initial paths.

    ros::ServiceClient nbvpClient = nh.serviceClient<nbvplanner::nbvp_srv>("/nbvplanner",true);

    int iteration = 0;

    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > way_pts;
    Eigen::Vector4d start,goal;

    while(ros::ok()){
        switch(state){
        case NOTREADY:
            std::cout << "Planner waiting for odometry" << std::endl;
            if(odomReady)
                state = TAKEOFF;
            break;
        case TAKEOFF:
            ROS_INFO("Starting the planner: Performing initialization motion");
            start << pose.translation(),mav_msgs::yawFromQuaternion(
                        (Eigen::Quaterniond)pose.rotation());
            way_pts.push_back(Eigen::Vector4d(start.x(),start.y(),start.z(),M_PI*.9));
            way_pts.push_back(Eigen::Vector4d(start.x(),start.y(),start.z(),M_PI*1.8));
            sendGoal(traj_interface,start,Eigen::Vector4d::Zero(),Eigen::Vector4d(start.x()+0.5,start.y(),start.z(),M_PI*1.8),way_pts);

            way_pts.clear();

        break;
        case TRAJ:
            if(!traj_interface.isRunning()){
                state = IDLE;
            }
            break;

        case IDLE:
            ROS_INFO_THROTTLE(0.5, "Planning iteration %i", iteration);
            nbvplanner::nbvp_srv planSrv;
            planSrv.request.header.stamp = ros::Time::now();
            planSrv.request.header.seq = iteration;
            planSrv.request.header.frame_id = "world";
            Timer t;
            if (nbvpClient.call(planSrv)) {
                n_seq++;
                if (planSrv.response.path.size() == 0) {
//                    ros::Duration(0.5).sleep();
                }
                else if(planSrv.response.path.size() >= 2){
                    planner_logger_.addTime(t.elapsed_ms());

                    mav_trajectory_generation::Trajectory trajectory;
                    tf::Pose pose_start;
                    tf::Pose pose_end;
                    tf::poseMsgToTF(planSrv.response.path[0], pose_start);
                    tf::poseMsgToTF(planSrv.response.path[planSrv.response.path.size()-1], pose_end);
                    double yaw_start = tf::getYaw(pose_start.getRotation());
                    double yaw_end = tf::getYaw(pose_end.getRotation());

                    start << pose.translation(),yaw_start;
//                    Eigen::Vector4d(planSrv.response.path[0].position.x,
//                            planSrv.response.path[0].position.y,
//                            planSrv.response.path[0].position.z,yaw_start);
                    goal = Eigen::Vector4d(planSrv.response.path[planSrv.response.path.size()-1].position.x,
                            planSrv.response.path[planSrv.response.path.size()-1].position.y,
                            planSrv.response.path[planSrv.response.path.size()-1].position.z,yaw_end);
                    sendGoal(traj_interface,start,Eigen::Vector4d::Zero(),goal);


                }

            }else {
                ROS_WARN_THROTTLE(1, "Planner not reachable");
                ros::Duration(1.0).sleep();
            }
            iteration++;
            break;
        }
        ros::spinOnce();

        //executing trajectory


        //            for (int i = 0; i < planSrv.response.path.size(); i++) {
        //                samples_array.header.seq = n_seq;
        //                samples_array.header.stamp = ros::Time::now();
        //                samples_array.header.frame_id = "world";
        //                samples_array.points.clear();
        //                tf::Pose pose;
        //                tf::poseMsgToTF(planSrv.response.path[i], pose);
        //                double yaw = tf::getYaw(pose.getRotation());
        //                trajectory_point.position_W.x() = planSrv.response.path[i].position.x;
        //                trajectory_point.position_W.y() = planSrv.response.path[i].position.y;
        //                // Add offset to account for constant tracking error of controller
        //                trajectory_point.position_W.z() = planSrv.response.path[i].position.z;
        //                tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), yaw);
        //                trajectory_point.setFromYaw(tf::getYaw(quat));
        //                mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
        //                samples_array.points.push_back(trajectory_point_msg);
        //                trajectory_pub.publish(samples_array);
        //            }

    }

    planner_logger_.saveData(log_path_);



    return 0;
}
