#ifndef POLYTRAJ_H
#define POLYTRAJ_H

#pragma once
#include <Eigen/Core>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/feasibility_sampling.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>


#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <mav_trajectory_generation/polynomial.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

#include <mav_trajectory_generation_ros/feasibility_sampling.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>

#include <mutex>
#include <atomic>

struct StateConstraint{
    double vmax = 1.5;
    double amax = 2.5;
    double yaw_rate=1.57;
    double yaw_acc=1.57;
};

class Quintic{
public:
    Eigen::Matrix<float,6,1> coeffs_;
    Eigen::Matrix<float,5,1> coeffs_d_;
    Eigen::Matrix<float,4,1> coeffs_dd_;
    Eigen::Matrix<float,3,1> coeffs_ddd_;
    float t_=0;
    Quintic(){

    }
    Quintic(Eigen::Matrix<float,6,1> coeffs , float t){

        t_ = t;
        coeffs(2) *=0.5f;

        coeffs_ = coeffs;
        if(fabs(coeffs(3)-coeffs(0))<0.0001){
            t_ = 0;
            return;
        }
        Eigen::Matrix<float,3,3> A;

        float t_sq = t*t;
        float t_th = t_sq*t;
        float t_fo = t_th*t;
        float t_fi = t_fo*t;
        A << t_th,t_fo,t_fi,
                3*t_sq,4*t_th,5*t_fo,
                6*t,12*t_sq,20*t_th;

        Eigen::Matrix<float,3,1> b;
        b<< coeffs(3)-coeffs(0)-coeffs(1)*t-coeffs(2)*t_sq,
                coeffs(4)-coeffs(1)-2*coeffs_(2)*t,
                coeffs(5) - 2*coeffs_(2);
        coeffs_.tail(3) = A.inverse() * b;

        coeffs_d_(2) = coeffs_(3) *3;
        coeffs_d_(3) = coeffs_(4) * 4;
        coeffs_d_(4) = coeffs_(5) * 5;

        coeffs_dd_(1) =coeffs_(3) * 6;
        coeffs_dd_(2) =coeffs_(4) * 12;
        coeffs_dd_(3) =coeffs_(5) * 20;

        coeffs_ddd_(0) = coeffs_dd_(1);
        coeffs_ddd_(1) =coeffs_(4)* 24;
        coeffs_ddd_(2) =coeffs_(5)* 60;
    }

    float ft(float t){

        return coeffs_(0)+coeffs_(1)*t+coeffs_(2)*t*t+coeffs_(3)*t*t*t + coeffs_(4)*t*t*t*t+coeffs_(5)*t*t*t*t*t;
    }

    float ft_d(float t){
        return coeffs_(1) + 2 * coeffs_(2) * t +
                3 * coeffs_(3) * t *t + 4 * coeffs_(4) * t *t*t + 5 * coeffs_(5) * t *t*t*t;
    }

    float ft_dd(float t){
        //        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3
        return  2 * coeffs_(2) +
                6 * coeffs_(3) * t  + 12 * coeffs_(4) * t *t + 20 * coeffs_(5) * t *t*t;

    }

    double evaluate(double t, int der){
        if(t_ < 0.0001){
            return coeffs_(der);
        }
        switch(der){
        case 0:
            return ft(t);
        case 1:
            return ft_d(t);
        case 2:
            return ft_dd(t);
        }

        return 0;
    }

    float ft_ddd(float t){
        Eigen::Matrix<float,3,1> t_v; t_v << 1,t;
        return coeffs_ddd_.dot(t_v);
    }

    void applyConstraint(double constraint_v,double constraint_a){
        if(t_ == 0){
            return;
        }
        double delta_t = 0.02;
        constexpr double kTolerance = 1e-3;
        constexpr size_t kMaxCounter = 20;

        bool within_range = false;
        std::cout << t_<<std::endl;
        for(size_t i = 0; i < kMaxCounter; ++i){
            double maxv=0,maxa=0;
            for(double t = 0; t <= t_; t+=delta_t){
                double v = ft_d(t);
                double a = ft_dd(t);
                v*=v;
                a*=a;
                if(v > maxv){
                    maxv = v;
                }

                if(a > maxa){
                    maxa = a;
                }
            }
            maxv = sqrt(maxv);
            maxa = sqrt(maxa);
            std::cout << maxa <<" " << maxv << " " << t_ <<std::endl;
            double velocity_violation = maxv / constraint_v;
            double acceleration_violation = maxa / constraint_a;

            within_range = velocity_violation <= 1.0 + kTolerance &&
                    acceleration_violation <= 1.0 + kTolerance;
            if (within_range) {
                break;
            }
            double violation_scaling = std::max(
                        1.0, std::max(velocity_violation, sqrt(acceleration_violation)));
            if(violation_scaling>1.8)
                violation_scaling*=0.6;
            double violation_scaling_inverse = 1.0 / violation_scaling;
            scaleQuintic(violation_scaling_inverse);

            t_*=violation_scaling;
        }

    }

    void scaleQuintic(double scale_factor){
        double scale =1.0;
        for(int i = 0; i<coeffs_.size();++i){
            coeffs_[i] *= scale;
            scale *=scale_factor;
        }
    }

};


class VelProfile{
public:

    float T_ = 1.;
    Eigen::Vector3f start_pos_ = Eigen::Vector3f::Zero();

    Quintic qx_;
    Quintic qy_;
    Quintic qz_;

    void init(const Eigen::Vector3f &vel_start , const Eigen::Vector3f &acc_start, const Eigen::Vector3f &jerk_start,
              const Eigen::Vector3f &vel_goal , const Eigen::Vector3f &acc_goal, const Eigen::Vector3f &jerk_goal,float t){

        T_ = t;
        Eigen::Matrix<float,6,1> state;

        state <<  vel_start(0),acc_start(0),jerk_start(0),vel_goal(0),acc_goal(0),jerk_goal(0);
        qx_ = Quintic(state,t);
        state <<  vel_start(1),acc_start(1),jerk_start(1),vel_goal(1),acc_goal(1),jerk_goal(1);
        qy_ = Quintic(state,t);

        state <<  vel_start(2),acc_start(2),jerk_start(2),vel_goal(2),acc_goal(2),jerk_goal(2);
        qz_ = Quintic(state,t);
    }


    void getVelAcc(float t, Eigen::Vector3f& vel, Eigen::Vector3f& acc){
        vel << qx_.ft(t),qy_.ft(t),qz_.ft(t);
        acc << qx_.ft_d(t),qy_.ft_d(t),qz_.ft_d(t);
    }

};


class SegmentManager4D{
public:
    SegmentManager4D(){

    }
    SegmentManager4D(const Eigen::Vector3d &start_position, mav_trajectory_generation::Segment::Vector xyz_segment, std::vector<Quintic> yaw_segments){
        segments_xyz_=xyz_segment;
        segments_yaw_=yaw_segments;
        size_t size = std::max(xyz_segment.size(),yaw_segments.size());
        start_position_ = start_position;
        //set max time, set maxtime segment
        segments_max_time_.resize(size);
        segments_xyz_time_.resize(size);
        segments_yaw_time_.resize(size);
        for(int i = 0;i < size;++i){
            segments_max_time_[i] = std::fmax(xyz_segment[i].getTime(),segments_yaw_[i].t_);
            segments_xyz_time_[i] = xyz_segment[i].getTime();
            segments_yaw_time_[i] = segments_yaw_[i].t_;
            max_time_ += segments_max_time_[i];
        }

        std::cout << max_time_<<std::endl;
    }

    bool sampleTrajectoryAtTime(double t, mav_msgs::EigenTrajectoryPoint *p){
        if(t > max_time_)
            return false;
        Eigen::Vector4d pos = evaluate(t,0);
        Eigen::Vector4d vel = evaluate(t,1);
        Eigen::Vector4d acc = evaluate(t,2);
        p->setFromYaw(pos.w());
        p->setFromYawRate(vel.w());
        p->setFromYawAcc(acc.w());


        p->position_W = pos.head(3);
        p->velocity_W = vel.head(3);
        p->acceleration_W = acc.head(3);

        p->degrees_of_freedom = mav_msgs::MavActuation::DOF4;
        return true;
    }

    Eigen::Vector4d evaluate(double t,int der){
        Eigen::Vector4d res;
        double accumulated_time = 0;
        size_t i;
        for(i = 0; i < segments_max_time_.size();++i){
            accumulated_time += segments_max_time_[i];
            if (accumulated_time > t) {
                break;
            }

        }

        if (i >= segments_max_time_.size()) {
            i = segments_max_time_.size() - 1;
        }
        float relative_t = accumulated_time;
        relative_t -= segments_max_time_[i];
        relative_t = t - relative_t;
        if(segments_xyz_.empty()){
            res.head(3) = start_position_;
        }else{
            if(relative_t > segments_xyz_time_[i]){
                res.head(3) = segments_xyz_[i].evaluate(segments_xyz_[i].getTime()-0.0001,der).head(3);
            }else{
                res.head(3) = segments_xyz_[i].evaluate(relative_t,der).head(3);
            }
        }

        if(relative_t > segments_yaw_time_[i]){
            res(3) = segments_yaw_[i].evaluate(segments_yaw_time_[i],der);
        }else{
            res(3) = segments_yaw_[i].evaluate(relative_t,der);
        }
        return res;
    }

    void sample3D(double start,double dt, std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> points){
        points.reserve((int) max_time_/dt+1);
        double t = start;
        while(t < max_time_){

            points.push_back(evaluate(t,0).head(3));
            t+=dt;
        }

    }

    double getMaxTime() const{
        return max_time_;
    }
    double max_time_=0;

    mav_trajectory_generation::Segment::Vector segments_xyz_;
    std::vector<Quintic> segments_yaw_;

    std::vector<double> segments_max_time_;
    std::vector<double> segments_xyz_time_;
    std::vector<double> segments_yaw_time_;
    Eigen::Vector3d start_position_;
};

class PolyTrajInterface
{
public:
    PolyTrajInterface(const ros::NodeHandle& nh,
                      const ros::NodeHandle& nh_private);


    bool computeTrajectoryAndYaw(const StateConstraint & contraint_i, const Eigen::Vector4d &start_pos, const Eigen::Vector4d &start_vel,
                                 const Eigen::Vector4d &goal_pos, const Eigen::Vector4d &goal_vel, const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > &wps,
                                 mav_trajectory_generation::Trajectory *trajectory);


    void computeYawTraj();

    void scaleTimeTrajectory();

    void commandTimerCallback(const ros::TimerEvent&);
    void yawStrategy(mav_msgs::EigenTrajectoryPoint &trajectory_point);


    void publishVizualization(const mav_trajectory_generation::Trajectory &trajectory);

    void startTrajectory();
    void stopTrajectory();

    float remainingTime(){
        trajectory_mtx_.lock();

        current_sample_time_ = ros::Duration(ros::Time::now()-start_time_).toSec();
        float t = current_sample_time_ - trajectory_.getMaxTime();
        trajectory_mtx_.unlock();
        return t;

    }



    bool inputFeasability(const mav_trajectory_generation::Trajectory *trajectory)
    {
        for(const auto &segment: trajectory->segments()){
            if (feasibility_check_.checkInputFeasibility(segment) !=
                    mav_trajectory_generation::InputFeasibilityResult::kInputFeasible) {
                return false;
            }
        }
        return true;
    }



    bool getPredictedPose(Eigen::Vector4d & pos_o, Eigen::Vector4d & vel_o);


    void setTrajectory(const mav_trajectory_generation::Trajectory &trajectory);


    void get3DTraj(double start, double dt, std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>& points){
        std::unique_lock<std::mutex>  m(trajectory_mtx_);
        trajectory_4D_.sample3D(start,dt,points);
    }


    std::vector<Quintic> yaw_traj_;

    mav_trajectory_generation::Trajectory trajectory_;
    ros::Publisher pub_markers_;


    mav_trajectory_generation::Trajectory current_trajectory_;
    mav_trajectory_generation::FeasibilityAnalytic feasibility_check_;

    bool useYaw = true;

    std::mutex trajectory_mtx_;

    std::atomic_bool setup_start;
    ros::Time start_time_;
    double current_sample_time_;
    Eigen::Vector3d  predicted_velocity_;
    Eigen::Vector3d predicted_acc_;
    Eigen::Vector3d predicted_pos_;

    SegmentManager4D trajectory_4D_;


    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher command_pub_;

    ros::Timer publish_timer_;


    double dt_ = 0.01;


    StateConstraint constraint_;


};


#endif // POLYTRAJ_H
