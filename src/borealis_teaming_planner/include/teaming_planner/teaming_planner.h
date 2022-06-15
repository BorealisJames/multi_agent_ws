#ifndef TEAMING_PLANNER_H
#define TEAMING_PLANNER_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <unordered_map>
#include <string>

#include "../../../distributed_multi_robot_formation/src/Common/Common.h"
#include "../../../distributed_multi_robot_formation/src/DistributedMultiRobotFormation.h"
#include "../../../distributed_multi_robot_formation/src/DistributedMultiRobotFormationHandler.h"
#include "../../../distributed_multi_robot_formation/src/ProcessPointCloud/ProcessPointCloud.h"

#include "../../../distributed_global_path_planner/src/Common/Common.h"

#include "../../../distributed_global_path_planner/src/DistributedGlobalPathPlanner.h"
#include "../../../distributed_global_path_planner/src/DistributedGlobalPathPlannerHandler.h"

#include "../../../Common/ConstantsEnum.h"
#include "../../../Common/Config/ConfigFileReader.h"
#include "teaming_planner_constants.h"

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Int8.h>
#include <mt_msgs/pose.h>
#include <mt_msgs/mtTask.h>
#include <mt_msgs/angleIndexAndUtility.h>
#include <mt_msgs/convexRegion2D.h>
#include <mt_msgs/convexRegion3D.h>
#include <mt_msgs/phaseAndTime.h>
#include <mt_msgs/position.h>
#include <mt_msgs/posevector.h>
#include <borealis_hri_msgs/Borealis_HRI_Output.h>

#include <tf/transform_listener.h>

class TeamingPlanner
{
    private:
        ros::NodeHandle mNh;
        ros::NodeHandle mNhPrivate;

        Common::Utils::ConfigFileReader mConfigFileReader;

        DistributedFormation::DistributedMultiRobotFormationHandler::Ptr mFormationHandlerPtr;
        DistributedFormation::DistributedMultiRobotFormation mDistributedFormation;

        // DistributedGlobalPathPlanner::DistributedGlobalPathPlannerHandler::Ptr mConsensusGlobalPathHandlerPtr;
        // DistributedGlobalPathPlanner::DistributedGlobalPathPlanner mConsensusGlobalPath;

        // Configurable Variables
        uint32_t mSourceSegmentId;
        double mModulePeriod;
        int mNumOfAgents;
        bool mDebugVerbose;
        double mIntervalDistance;
        double mPlanningHorizon;
        double mDesiredHeight;
        
        // Variables
        TeamingPlannerConstants::ModuleState mModuleState;
        std::vector<DistributedFormation::Common::Pose> mHistoryOfHumanPoses;
        DistributedFormation::Common::Pose mHumanSystemPose;
        Common::Entity::MTTaskBundle mTask;
        DistributedFormation::Common::Pose mSelfSystemPose;
        sensor_msgs::PointCloud mSystemPointCloud;
        geometry_msgs::PoseStamped mSelft265SystemPose;
        sensor_msgs::PointCloud2 mSystemPointCloud2;

        sensor_msgs::PointCloud mSystemDepthCamera;
        std::unordered_map<int32_t, DistributedFormation::Common::PhaseAndTime> mAgentsPhaseAndTimeMap;
        std::unordered_map<int32_t, DistributedFormation::Common::Pose> mAgentsPoseMap;
        std::unordered_map<int32_t, DistributedFormation::Common::DirectionUtility> mAgentsDirectionUtilityMap;
        std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion2D> mAgentsConvexRegion2DMap;
        std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion3D> mAgentsConvexRegion3DMap;
        std::unordered_map<int32_t, std::unordered_map<int32_t, DistributedFormation::Common::Pose>> mAgentsAssignedVirtualPoseMap;
        tf::TransformListener mPoseTransformListener;
        tf::TransformListener mPointCloudTransformListener;
        tf::TransformListener mPointCloud2TransformListener;

        bool mHistoryOfHumanPosesReceived;
        bool useUWB;

        std_msgs::Bool mBoolActivatePlanner; 
        std::string mHRIMode;
        geometry_msgs::PoseStamped mHRIPoseStamped;
        int mNumberOfAgentsInFormation;

        // There is a probably better way to implement this but htis should do
        geometry_msgs::PoseWithCovarianceStamped mGunTargetPose;
        bool mgunTargetPoseRecieved;
    
        // Verbose Variables
        bool mModuleStateVerbose;
        bool mModuleTaskVerbose;

        // Publisher
        ros::Publisher mPhaseAndTimePublisher;
        ros::Publisher mPosePublisher;
        ros::Publisher mDirectionUtilityPublisher;
        ros::Publisher mConvexRegion2DPublisher;
        ros::Publisher mConvexRegion3DPublisher;
        ros::Publisher mAssignedVirtualPosePublisher;
        ros::Publisher mAssignedt265VirtualPosePublisher;
        ros::Publisher mAssignedVirtualPoseMapPublisher;

        ros::Publisher mVoxel_filter_cloudPublisher;

        // Subscribers 
        ros::Subscriber mGoalSubscriber;
        ros::Subscriber mTaskSubscriber;
        ros::Subscriber mHumanSystemPoseSubscriber;
        ros::Subscriber mSelfSystemPoseSubscriber;
        ros::Subscriber mSelft265SystemPoseSubscriber;
        ros::Subscriber mSystemPointCloudSubscriber;
        ros::Subscriber mSystemPointCloud2Subscriber;
        ros::Subscriber mSystemDepthCameraSubscriber;
        ros::Subscriber mGunTargetPoseSubscriber;
        ros::Subscriber mActivatePlannerSubscriber;

        ros::Subscriber mHRIModeSubscriber;

        // Hardcoded for now
        std::vector<ros::Subscriber> mUAVSystemPoseSubscriberVector;
        std::vector<ros::Subscriber> mUAVPhaseAndTimeSubscriberVector;
        std::vector<ros::Subscriber> mUAVDirectionUtilitySubscriberVector;
        std::vector<ros::Subscriber> mUAVConvexRegion2DSubscriberVector;
        std::vector<ros::Subscriber> mUAVConvexRegion3DSubscriberVector;
        std::vector<ros::Subscriber> mUAVAssignedVirtualPoseMapSubscriberVector;
        
        // Timers
        ros::Timer mModuleLoopTimer;

        // Publisher Functions
        bool pubPhaseAndTime_mrf(const int32_t aAgentId, const DistributedFormation::Common::PhaseAndTime aPhaseAndTime);
        bool pubPose_mrf(const int32_t aAgentId, const DistributedFormation::Common::Pose aPose);
        bool pubDirectionUtility_mrf(const int32_t aAgentId, const DistributedFormation::Common::DirectionUtility aDirectionUtility);
        bool pubConvexRegion2D_mrf(const int32_t aAgentId, const DistributedFormation::Common::ConvexRegion2D aConvexRegion2D);
        bool pubConvexRegion3D_mrf(const int32_t aAgentId, const DistributedFormation::Common::ConvexRegion3D aConvexRegion3D);
        bool pubAssignedPose_mrf(const int32_t aAgentId, const DistributedFormation::Common::Pose aAssignedVirtualPose);
        bool pubAssignedPoseMap_mrf(const int32_t aAgentId, const std::unordered_map<int32_t, DistributedFormation::Common::Pose> aAssignedVirtualPoseMap);
        bool switchToGunTargetPose(const int32_t aAgentId);

        // Subscriber Functions
        void goalCallback(const mt_msgs::pose::ConstPtr& aGoal);
        void taskCallback(const mt_msgs::mtTask::ConstPtr& aTask);
        void activatePlannerCallback(const std_msgs::Bool::ConstPtr& aBoolActivatePlanner);

        void humanSystemPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& aHumanSystemPose);
        // void selfSystemPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& aSelfSystemPose);
        void selfSystemPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& aSelfSystemPose);
        void selfSystemPoseCallbackUWB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& aSelfSystemPose);
        void selft265SystemPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& aSelfSystemPose);
        void systemPoseCallback(const mt_msgs::pose::ConstPtr& aSystemPose);
        void systemPointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& aSystemPointCloud);

        void systemPointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& aSystemPointCloud2);

        void systemDepthCameraCallback(const sensor_msgs::PointCloud2::ConstPtr& aSystemDepthCamera);
        void phaseTimeCallback(const mt_msgs::phaseAndTime::ConstPtr& aPhaseAndTime);
        void directionUtilityCallback(const mt_msgs::angleIndexAndUtility::ConstPtr& aDirectionUtility);
        void convexRegion2DCallback(const mt_msgs::convexRegion2D::ConstPtr& aConvexRegion2D);
        void convexRegion3DCallback(const mt_msgs::convexRegion3D::ConstPtr& aConvexRegion3D);
        void assignedVirtualPoseMapCallback(const mt_msgs::posevector::ConstPtr& aAssignedVirtualPoseMap);

        void gunCommandPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& aPoseWithCovarianceStamped);
        void incomingHRIModeCallback(const borealis_hri_msgs::Borealis_HRI_Output::ConstPtr& aHRImsg);

        // Timer Functions 
        void moduleLoopCallback(const ros::TimerEvent& event);

        // Get Functions
        bool getOwnAgentId(int32_t& ownAgentID);
        bool getNumberOfAgentsInTeam(int32_t& numberOfAgentsInTeam);

        bool getPosesForFormationToTrack_mrf(std::vector<DistributedFormation::Common::Pose>& historyOfHumanPoses);
        bool getPhaseAndTimeMap_mrf(std::unordered_map<int32_t, DistributedFormation::Common::PhaseAndTime>& phaseAndTimeMap);
        bool getPoseMap_mrf(std::unordered_map<int32_t, DistributedFormation::Common::Pose>& poseMap);
        bool getDirectionUtilityMap_mrf(std::unordered_map<int32_t, DistributedFormation::Common::DirectionUtility>& directionUtilityMap);
        bool getOwnAgentLidarPointCloud(sensor_msgs::PointCloud& cloud);
        bool getOwnAgentDepthCamera(sensor_msgs::PointCloud& depthCamera);
        bool getConvexRegion2DMap_mrf(std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion2D>& convexRegion2DMap);
        bool getConvexRegion3DMap_mrf(std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion3D>& convexRegion3DMap);
        bool getAssignedVirtualPoseMap_mrf(std::unordered_map<int32_t, std::unordered_map<int32_t, DistributedFormation::Common::Pose>>& assignedVirtualPoseMap);
        bool getHumanSystemPose_mrf(DistributedFormation::Common::Pose& aHumanSystemPose);
        bool getOwnUAVSystemPose_mrf(DistributedFormation::Common::Pose& aUAVSystemPose);

        // distributed path planner

    // std::function<bool(std::vector<Common::Pose>& goTherePath)> m_getGoTherePath;
    // std::function<void()>  m_clearAgentsPoseBuffer;
    // std::function<bool(std::unordered_map<int32_t, Common::PhaseAndTime>& phasesAndTimeRecordOfAgents)>  m_getPhasesAndTimeRecordOfAgents;

    // std::function<bool(const int32_t, const Common::PhaseAndTime& ownAgentPhaseAndTime)>  m_pubOwnPhaseAndTime;

    // std::function<bool(const int32_t ownAgentID, const Common::Pose& ownAgentPose)>  m_pubOwnPoseFunc;
    // std::function<bool(Common::Pose& ownAgentPose)>  m_getOwnAgentPose;
    // std::function<bool(std::unordered_map<int32_t, Common::Pose>& agentsPose)>  m_getAgentsPose;

    // std::function<void()>m_clearAgentsPathAndWaypointProgressBuffer;
    // std::function<bool(std::unordered_map<int32_t, Common::PathAndWaypointProgress>& agentsGoTherePathAndWaypointProgress)> m_getAgentsPathAndWaypointProgress;
    // std::function<bool(const int32_t, const Common::PathAndWaypointProgress& goTherePathAndWaypointProgress)> m_pubOwnPathAndWaypointProgress;

    // std::function<bool(sensor_msgs::PointCloud& cloud)>  m_getOwnAgentLidarPointCloud;
    // std::function<bool(sensor_msgs::PointCloud& cloud)>  m_getOwnAgentCameraPointCloud;

    // std::function<void()>  m_clearAgentsPlannedPathBuffer;
    // std::function<bool(std::unordered_map<int32_t, std::vector<Eigen::Vector3d>>& agentsPlannedPath)> m_getAgentsPlannedPath;
    // std::function<bool(const int32_t, const std::vector<Eigen::Vector3d>& ownPlannedPath)> m_pubOwnPlannedPath;

    // std::function<void()>  m_clearAgentsProcessedPathOfAgentsBuffer;
    // std::function<bool(std::unordered_map<int32_t, std::unordered_map<int32_t, Common::PathAndCost>>& agentsProcessedPathOfAgents)> m_getAgentsProcessedPathOfAgents;
    // std::function<bool(const int32_t, const std::unordered_map<int32_t, Common::PathAndCost>& ownProcessedPathOfAgents)> m_pubOwnProcessedPathOfAgents;

    // std::function<void()> m_clearAgentsBestProcessedPathBuffer;
    // std::function<bool(std::unordered_map<int32_t, std::vector<Eigen::Vector3d>>& agentsBestProcessedPath)> m_getAgentsBestProcessedPath;
    // std::function<bool(const int32_t, const std::vector<Eigen::Vector3d>& ownBestProcessedPath)> m_pubOwnBestProcessedPath;

    // std::function<bool(const int32_t, const std::vector<Common::Pose>& processedGoTherePath)>  m_pubProcessedGoTherePath;

        bool getGoTherePath(std::vector<DistributedGlobalPathPlanner::Common::Pose>& goTherePath);
        bool clearAgentsPoseBuffer_cgpp();
        bool getPhasesAndTimeRecordOfAgents(std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PhaseAndTime>& phaseAndTimeMap_gpp);
        bool pubPhaseAndTime_mrf_cgpp(const int32_t, const DistributedGlobalPathPlanner::Common::PhaseAndTime& ownAgentPhaseAndTime);

        void clearPhaseAndTimeMap();
        void clearPoseMap();
        void clearDirectionUtilityMap();
        void clearConvexRegion2DMap();
        void clearConvexRegion3DMap();
        void clearAssignedVirtualPoseMap();

        // Functions 
        void teamingPlannerMain();
        bool checkAndAddHumanSystemPose(std::vector<DistributedFormation::Common::Pose>& historyOfHumanPoses, const DistributedFormation::Common::Pose aPose);
        double euclideanDistance(const double x1, const double y1, const double x2, const double y2);

        geometry_msgs::PoseStamped subtractPoseStamped(geometry_msgs::PoseStamped previous, geometry_msgs::PoseStamped current); // current - previous
        geometry_msgs::PoseStamped addPoseStamped(geometry_msgs::PoseStamped vector_pose, geometry_msgs::PoseStamped current);

    public:
        TeamingPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nhPrivate);
        virtual ~TeamingPlanner();



}; // TeamingPlanner

#endif // TEAMING_PLANNER_H
