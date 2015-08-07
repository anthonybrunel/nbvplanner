#ifndef NBVP_H_
#define NBVP_H_

#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeNode.h>
#include <octomap/OcTree.h>
#include <octomap_world/octomap_manager.h>
#include <kdtree/kdtree.h>
#include <nbvplanner/nbvp_srv.h>
#include <nbvplanner/mesh_structure.h>

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace nbvInspection {
  
  template<typename stateVec>
  class Node {
    static int counter_;
  public:
    Node();
    ~Node();
    Node * minDist(stateVec);
    static int getCounter();
    void printToFile(std::fstream& file);

    stateVec state_;
    Node * parent_;
    std::vector<Node*> children_;
    double informationGain_;
    double distance_;
    static double bestInformationGain_;
    static Node * bestNode_;
    static const double ZERO_INFORMATION_GAIN_;
  };

  template<typename stateVec>
  class nbvPlanner {
    static double v_max_;
    static double dyaw_max_;
    static double dv_max_;
    static double ddyaw_max_;
    static double camPitch_;
    static double camHorizontal_;
    static double camVertical_;
    
    static double igProbabilistic_;
    static double igFree_;
    static double igOccupied_;
    static double igUnmapped_;
    static double igArea_;
    static double informationGainRange_;
    static double degressiveCoeff_;
    static double extensionRange_;
    static int initIterations_;
    static double dt_;
    
    static double minX_;
    static double minY_;
    static double minZ_;
    static double maxX_;
    static double maxY_;
    static double maxZ_;
    static bool softBounds_;
    static double probability_mean_clamp_;
    static double probability_deviation_clamp_;
    std::vector<std::stack<stateVec> > history_;
    static bool use_history_;
  
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    std::vector<stateVec *> root_;
    std::vector<stateVec *> g_stateOld_;
    
    ros::Publisher inspectionPath_;
    ros::Publisher treePub_;
    ros::ServiceClient octomapClient_;
    ros::Subscriber posClient0_;
    ros::Subscriber posClient1_;
    ros::Subscriber posClient2_;
    ros::Subscriber posClient3_;
    ros::Subscriber posClient4_;
    ros::ServiceServer plannerService0_;
    ros::ServiceServer plannerService1_;
    ros::ServiceServer plannerService2_;
    ros::ServiceServer plannerService3_;
    ros::ServiceServer plannerService4_;
    ros::Subscriber pointcloud_sub0_;
    ros::Subscriber pointcloud_sub1_;
    ros::Subscriber pointcloud_sub2_;
    ros::Subscriber pointcloud_sub3_;
    ros::Subscriber pointcloud_sub4_;
    
    std::vector<ros::Time> g_timeOld_;
    double average_computation_duration_;
    int g_ID_;
    std::string pkgPath_;
    int iteration_;
    std::vector<Eigen::Vector3d> camBoundNormals_;
    Node<stateVec> * rootNode_;
    kdtree * kdTree_;
    std::vector<std::vector<stateVec> > bestBranchOld_;
    static volumetric_mapping::OctomapManager * manager_;
    static mesh::StlMesh * mesh_;
    static Eigen::Vector3d boundingBox_;

    std::vector<double> mappedOccupied_;
    std::vector<double> mappedFree_;
    std::vector<double> unMapped_;
  public:
    typedef std::vector<stateVec> vector_t;
    typedef octomap::OcTree octomap_t;
    
    nbvPlanner(const ros::NodeHandle& nh,
               const ros::NodeHandle& nh_private);
    ~nbvPlanner();
    vector_t buildTree(nbvPlanner<stateVec>& instance, int I, stateVec s, double& IGout,
                              double (nbvPlanner<stateVec>::*informationGain)(stateVec), int agentID);
    vector_t sampleEuler(stateVec s);
    double informationGain(stateVec s);
    static bool setParams();
    static bool getRRTextension();
    static int getInitIterations();
    static bool extensionRangeSet();

    void posCallback0(const geometry_msgs::PoseStamped& pose);
    void posCallback1(const geometry_msgs::PoseStamped& pose);
    void posCallback2(const geometry_msgs::PoseStamped& pose);
    void posCallback3(const geometry_msgs::PoseStamped& pose);
    void posCallback4(const geometry_msgs::PoseStamped& pose);
    void posCallback(const geometry_msgs::PoseStamped& pose, int agentID);
    bool plannerCallback0(nbvplanner::nbvp_srv::Request& req,
                         nbvplanner::nbvp_srv::Response& res);
    bool plannerCallback1(nbvplanner::nbvp_srv::Request& req,
                         nbvplanner::nbvp_srv::Response& res);
    bool plannerCallback2(nbvplanner::nbvp_srv::Request& req,
                         nbvplanner::nbvp_srv::Response& res);
    bool plannerCallback3(nbvplanner::nbvp_srv::Request& req,
                         nbvplanner::nbvp_srv::Response& res);
    bool plannerCallback4(nbvplanner::nbvp_srv::Request& req,
                         nbvplanner::nbvp_srv::Response& res);
    bool plannerCallback(nbvplanner::nbvp_srv::Request& req,
                         nbvplanner::nbvp_srv::Response& res, int agentID);
                         
  };
}

#endif // NBVP_H_
