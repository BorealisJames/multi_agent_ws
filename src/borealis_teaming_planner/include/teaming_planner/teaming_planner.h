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

#include "../../../Common/ConstantsEnum.h"
#include "../../../Common/Config/ConfigFileReader.h"
#include "teaming_planner_constants.h"

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <std_msgs/Int8.h>
#include <tf/transform_listener.h>
#include <mt_msgs/pose.h>
#include <mt_msgs/mtTask.h>
#include <mt_msgs/angleIndexAndUtility.h>
#include <mt_msgs/convexRegion2D.h>
#include <mt_msgs/convexRegion3D.h>
#include <mt_msgs/phaseAndTime.h>
#include <mt_msgs/position.h>
#include <mt_msgs/posevector.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


// main changes -> depth camera ignored/deleted , system pose cb changed to local pose cb then localpose cb will transform to systempose and pub it, pointcloud 2

class TeamingPlanner
{
    private:
        ros::NodeHandle mNh;
        ros::NodeHandle mNhPrivate;

        Common::Utils::ConfigFileReader mConfigFileReader;

        DistributedFormation::DistributedMultiRobotFormationHandler::Ptr mHandlerPtr;
        DistributedFormation::DistributedMultiRobotFormation mDistributedFormation;

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

        sensor_msgs::PointCloud2 mSystemPointCloud2;

        sensor_msgs::PointCloud mSystemDepthCamera;
        std::unordered_map<int32_t, DistributedFormation::Common::PhaseAndTime> mAgentsPhaseAndTimeMap;
        std::unordered_map<int32_t, DistributedFormation::Common::Pose> mAgentsPoseMap;
        std::unordered_map<int32_t, DistributedFormation::Common::DirectionUtility> mAgentsDirectionUtilityMap;
        std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion2D> mAgentsConvexRegion2DMap;
        std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion3D> mAgentsConvexRegion3DMap;
        std::unordered_map<int32_t, std::unordered_map<int32_t, DistributedFormation::Common::Pose>> mAgentsAssignedVirtualPoseMap;
        Common::Entity::ControlState mControlState;
        tf::TransformListener mPoseTransformListener;
        tf::TransformListener mPointCloudTransformListener;

        tf::TransformListener mPointCloud2TransformListener;

        bool mHistoryOfHumanPosesReceived;
        
        // There is a probably better way to implement this but htis should do
        // Whatever lmao
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
        ros::Publisher mAssignedVirtualPoseMapPublisher;
        ros::Publisher mControlStatePublisher;

        ros::Publisher mVoxel_filter_cloudPublisher;

        // Subscribers 
        ros::Subscriber mGoalSubscriber;
        ros::Subscriber mTaskSubscriber;
        ros::Subscriber mHumanSystemPoseSubscriber;
        ros::Subscriber mSelfSystemPoseSubscriber;
        ros::Subscriber mSystemPointCloudSubscriber;
        ros::Subscriber mSystemPointCloud2Subscriber;
        ros::Subscriber mSystemDepthCameraSubscriber;

        ros::Subscriber mGunTargetPoseSubscriber;

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
        bool pubPhaseAndTime(const int32_t aAgentId, const DistributedFormation::Common::PhaseAndTime aPhaseAndTime);
        bool pubPose(const int32_t aAgentId, const DistributedFormation::Common::Pose aPose);
        bool pubDirectionUtility(const int32_t aAgentId, const DistributedFormation::Common::DirectionUtility aDirectionUtility);
        bool pubConvexRegion2D(const int32_t aAgentId, const DistributedFormation::Common::ConvexRegion2D aConvexRegion2D);
        bool pubConvexRegion3D(const int32_t aAgentId, const DistributedFormation::Common::ConvexRegion3D aConvexRegion3D);
        bool pubAssignedPose(const int32_t aAgentId, const DistributedFormation::Common::Pose aAssignedVirtualPose);
        bool pubAssignedPoseMap(const int32_t aAgentId, const std::unordered_map<int32_t, DistributedFormation::Common::Pose> aAssignedVirtualPoseMap);
        bool pubControlState(const Common::Entity::ControlState aControlState);
        bool switchToGunTargetPose(const int32_t aAgentId);

        // Subscriber Functions
        void goalCallback(const mt_msgs::pose::ConstPtr& aGoal);
        void taskCallback(const mt_msgs::mtTask::ConstPtr& aTask);
        void humanSystemPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& aHumanSystemPose);
        void selfSystemPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& aSelfSystemPose);
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

        // Timer Functions 
        void moduleLoopCallback(const ros::TimerEvent& event);

        // Get Functions
        bool getOwnAgentId(int32_t& ownAgentID);
        bool getHistoryOfHumanPoses(std::vector<DistributedFormation::Common::Pose>& historyOfHumanPoses);
        bool getPhaseAndTimeMap(std::unordered_map<int32_t, DistributedFormation::Common::PhaseAndTime>& phaseAndTimeMap);
        bool getPoseMap(std::unordered_map<int32_t, DistributedFormation::Common::Pose>& poseMap);
        bool getDirectionUtilityMap(std::unordered_map<int32_t, DistributedFormation::Common::DirectionUtility>& directionUtilityMap);
        bool getOwnAgentLidarPointCloud(sensor_msgs::PointCloud& cloud);
        bool getOwnAgentDepthCamera(sensor_msgs::PointCloud& depthCamera);
        bool getConvexRegion2DMap(std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion2D>& convexRegion2DMap);
        bool getConvexRegion3DMap(std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion3D>& convexRegion3DMap);
        bool getAssignedVirtualPoseMap(std::unordered_map<int32_t, std::unordered_map<int32_t, DistributedFormation::Common::Pose>>& assignedVirtualPoseMap);
        bool getHumanSystemPose(DistributedFormation::Common::Pose& aHumanSystemPose);
        bool getOwnUAVSystemPose(DistributedFormation::Common::Pose& aUAVSystemPose);

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

    public:
        TeamingPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nhPrivate);
        virtual ~TeamingPlanner();



}; // TeamingPlanner

#endif // TEAMING_PLANNER_H
