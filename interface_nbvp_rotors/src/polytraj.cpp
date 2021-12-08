#include "polytraj.h"

PolyTrajInterface::PolyTrajInterface(const ros::NodeHandle &nh,
                                     const ros::NodeHandle& nh_private
                                     )
{
    publish_timer_ = nh_.createTimer(ros::Duration(dt_),
                                     &PolyTrajInterface::commandTimerCallback,
                                     this, false, false);

    std::cout << "[PolyTrajInterface] Initialized" <<std::endl;

    pub_markers_ =
            nh_private_.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 1000);

    //    typedef mav_trajectory_generation::InputConstraintType ICT;
    //    mav_trajectory_generation::InputConstraints input_constraints;
    //    input_constraints.addConstraint(
    //                ICT::kFMin, 0.5 * 9.81); // minimum acceleration in [m/s/s].
    //    input_constraints.addConstraint(
    //                ICT::kFMax, constraint_->max_a_ * 9.81); // maximum acceleration in [m/s/s].
    //    input_constraints.addConstraint(
    //                ICT::kVMax, constraint_->max_v_); // maximum velocity in [m/s].
    //    input_constraints.addConstraint(
    //                ICT::kOmegaXYMax, M_PI / 2.0); // maximum roll/pitch rates in [rad/s].
    //    input_constraints.addConstraint(
    //                ICT::kOmegaZMax,
    //                constraint_->max_yaw_vel_); // max yaw rate [rad/s].
    //    input_constraints.addConstraint(
    //                ICT::kOmegaZDotMax,
    //                constraint_->max_yaw_acc_); // maximum yaw acceleration in [rad/s/s]..
    //    feasibility_check_ =
    //            mav_trajectory_generation::FeasibilityAnalytic(input_constraints);
    //    feasibility_check_.settings_.setMinSectionTimeS(0.01);


    dt_ = (0.01);

    nh_private_.param("dt", dt_, dt_);

    command_pub_ = nh_private_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
                mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);


}



bool PolyTrajInterface::computeTrajectoryAndYaw(const StateConstraint &constraint_i, const Eigen::Vector4d &start_pos, const Eigen::Vector4d &start_vel, const Eigen::Vector4d &goal_pos,
                                                const Eigen::Vector4d &goal_vel,
                                                const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > &wps, mav_trajectory_generation::Trajectory *trajectory)
{
    std::cout << "New Trajectory 3D with waypoints" << start_pos.transpose() << " "<< start_vel.transpose() << " " << goal_pos .transpose()<< " " << goal_vel.transpose() <<std::endl;

    mav_trajectory_generation::Segment::Vector segment_xyz;

    //    if((start_pos.head(3)-goal_pos.head(3)).squaredNorm()> 0.0001)
    //    {
    const int dimension = 3;
    mav_trajectory_generation::Vertex::Vector vertices;

    const int derivative_to_optimize =
            mav_trajectory_generation::derivative_order::ACCELERATION;


    //    optimalTraj.computeTrajPoints(0.01,pts);
    mav_trajectory_generation::Vertex start(dimension), end(dimension);
    start.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(start_pos.head<3>()));
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        start_vel.head(3));
    vertices.push_back(start);

    mav_trajectory_generation::Vertex p(dimension);
    for(size_t i = 0; i < wps.size(); ++i){
        const Eigen::Vector4d &next_pts = wps[i];
        p.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(next_pts.head<3>()));
        p.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        Eigen::Vector3d::Zero());
        p.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
                        Eigen::Vector3d(0,0,0));
        vertices.push_back(p);
    }


    end.addConstraint(mav_trajectory_generation::derivative_order::POSITION,goal_pos.head<3>());
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      goal_vel.head(3));
    end.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
                      Eigen::Vector3d(0,0,0));

    vertices.push_back(end);
    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, constraint_i.vmax, constraint_i.amax);

    for(size_t i =0; i < segment_times.size(); ++i){
        if(segment_times[i] < 0.001)
            segment_times[i] = 0.1;
    }

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    parameters.max_iterations = 1000;
    parameters.f_rel = 0.05;
    parameters.x_rel = 0.1;
    parameters.time_penalty = 2000.0;
    parameters.initial_stepsize_rel = 0.1;
    parameters.inequality_constraint_tolerance = 0.1;

    // set up optimization problem
    const int N = 6;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);


    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, constraint_i.vmax);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, constraint_i.amax);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::JERK, 20.);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::SNAP, 20.);

    // solve trajectory
    opt.optimize();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));
    trajectory->scaleSegmentTimesToMeetConstraints(constraint_i.vmax, constraint_i.amax);



    trajectory->getSegments(&segment_xyz);
    //    }

    std::vector<Quintic> segments_yaw;
    float prev_yaw = start_pos.w();
    float prev_vel_yaw = start_vel.w();
    double prev_time = 0;
    if(wps.size() > 0){
        for(size_t i = 0; i < wps.size(); ++i){
            const float next_yaw = wps[i].w();
            Eigen::Matrix<float,6,1> state;state << prev_yaw,prev_vel_yaw,0,next_yaw,0,0;
            double t = fabs(prev_yaw-next_yaw)/constraint_i.vmax;
            Quintic q(state,t);
            //applyconstraint
            q.applyConstraint(constraint_i.yaw_rate,constraint_i.yaw_acc);
            segments_yaw.push_back(q);

            prev_yaw = next_yaw;

            prev_vel_yaw = 0;

        }
    }

    Eigen::Matrix<float,6,1> state;state << prev_yaw,prev_vel_yaw,0,goal_pos.w(),0,0;
    Quintic q(state,fabs(prev_yaw-goal_pos.w())/constraint_i.yaw_rate);
    //apply constraint
    q.applyConstraint(constraint_i.yaw_rate,constraint_i.yaw_acc);
    segments_yaw.push_back(q);
    std::cout << goal_pos.w() <<std::endl;

    std::cout << q.t_ <<std::endl;
    trajectory_4D_ = SegmentManager4D(start_pos.head(3),segment_xyz,segments_yaw);
    return true;

}




void PolyTrajInterface::commandTimerCallback(const ros::TimerEvent &)
{




    if(setup_start){
        current_sample_time_ = 0.0;
        start_time_ = ros::Time::now();
        setup_start = false;
    }
    trajectory_mtx_.lock();
    current_sample_time_ = ros::Duration(ros::Time::now()-start_time_).toSec();
    if (current_sample_time_ <= trajectory_4D_.getMaxTime()) {
        trajectory_msgs::MultiDOFJointTrajectory msg;
        mav_msgs::EigenTrajectoryPoint trajectory_point;
        bool success = trajectory_4D_.sampleTrajectoryAtTime(
                    current_sample_time_, &trajectory_point);
        if (!success) {
            publish_timer_.stop();
        }

        mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
        msg.points[0].time_from_start = ros::Duration(current_sample_time_);
        command_pub_.publish(msg);
    } else if(current_sample_time_ <= trajectory_4D_.getMaxTime() + 0.5){//time shift to go at goal
        current_sample_time_ = trajectory_4D_.getMaxTime();
        trajectory_msgs::MultiDOFJointTrajectory msg;
        mav_msgs::EigenTrajectoryPoint trajectory_point;
        bool success = trajectory_4D_.sampleTrajectoryAtTime(
                    current_sample_time_, &trajectory_point);
        if (!success) {
            publish_timer_.stop();
        }
        mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
        msg.points[0].time_from_start = ros::Duration(current_sample_time_);
    }else {
        publish_timer_.stop();
        //        trajectory_.clear();
    }
    trajectory_mtx_.unlock();
}

void PolyTrajInterface::yawStrategy(mav_msgs::EigenTrajectoryPoint &trajectory_point)
{
    if(useYaw & trajectory_point.velocity_W.norm() > 0)
        return;

    float goal_yaw =std::atan2(trajectory_point.velocity_W.x(),
                               -trajectory_point.velocity_W.y()) - M_PI/2.;

    if (goal_yaw > M_PI)        { goal_yaw -= 2 * M_PI; }
    else if (goal_yaw <= -M_PI) { goal_yaw += 2 * M_PI; }

    trajectory_point.setFromYaw(goal_yaw);
    trajectory_point.timestamp_ns = ros::Time::now().toNSec();

}

void PolyTrajInterface::startTrajectory()
{
    //    if(trajectory_.getMaxTime() <= 0)
    //        return;
    setup_start = true;
    trajectory_mtx_.lock();
    publish_timer_.start();
    trajectory_mtx_.unlock();

}

void PolyTrajInterface::stopTrajectory()
{
    current_sample_time_ = trajectory_4D_.getMaxTime()+10;
    publish_timer_.stop();
}

void PolyTrajInterface::setTrajectory(const mav_trajectory_generation::Trajectory &trajectory)
{
    trajectory_mtx_.lock();
    publish_timer_.stop();
    trajectory_.clear();
    trajectory_ = trajectory;
    trajectory_mtx_.unlock();

}




void PolyTrajInterface::publishVizualization(const mav_trajectory_generation::Trajectory &trajectory)
{
    if(trajectory.empty())
        return;
    // send trajectory as markers to display them in RVIZ
    visualization_msgs::MarkerArray markers;
    double distance =
            0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";


    mav_trajectory_generation::drawMavTrajectory(trajectory,
                                                 distance,
                                                 frame_id,
                                                 &markers);
    pub_markers_.publish(markers);
}
