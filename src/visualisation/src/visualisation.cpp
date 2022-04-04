#include "../include/visualisation/visualisation.h"

void GetRGBColorFromIndex(const int index, double& r, double&g, double& b)
{
    int i = index%10;

    switch(i)
    {
        case(0):
            r = 0.0; g = 1.0; b = 0.0;
            break;
        case(1):
            r = 1.0; g = 0.0; b = 1.0;
            break;
        case(2):
            r = 0.0; g = 0.5; b = 1.0;
            break;
        case(3):
            r = 1.0; g = 0.5; b = 0.0;
            break;
        case(4):
            r = 0.5; g = 0.75; b = 0.5;
            break;
        case(5):
            r = 0.533892398764885; g = 0.16088038146846761; b = 0.4546505629015031;
            break;
        case(6):
            r = 0.8042233302743353; g = 0.4917729931430055; b = 0.9969995072416731;
            break;
        case(7):
            r = 0.07574136791722907; g = 0.9995171645066149; b = 0.8960459718316001;
            break;
        case(8):
            r = 0.22114602566292718; g = 0.018919839112809256; b = 0.997146736229133;
            break;
        case(9):
            r = 0.8412342877005651; g = 0.9895682371606811; b = 0.005968595765017959;
            break;
        default:
            r = 1.0; g = 0.0; b = 0.0;
            break;
    }
}

Visualisation::Visualisation(const ros::NodeHandle& nh, const ros::NodeHandle& nhPrivate):
    mNh(nh),
    mNhPrivate(nhPrivate),
    mConfigFileReader(),
    mTransformListener()
{
    // Configured Variables
    mConfigFileReader.getParam(nhPrivate, "NumberOfUAV", mNumberOfUAV, 3);

    mConfigFileReader.getParam(nhPrivate, "UAVScaleX", mUAVScaleX, 0.7);
    mConfigFileReader.getParam(nhPrivate, "UAVScaleY", mUAVScaleY, 0.1);
    mConfigFileReader.getParam(nhPrivate, "UAVScaleZ", mUAVScaleZ, 0.1);

    mConfigFileReader.getParam(nhPrivate, "GoalScaleX", mGoalScaleX, 0.7);
    mConfigFileReader.getParam(nhPrivate, "GoalScaleY", mGoalScaleY, 0.1);
    mConfigFileReader.getParam(nhPrivate, "GoalScaleZ", mGoalScaleZ, 0.1);

    mConfigFileReader.getParam(nhPrivate, "PoseScaleX", mPoseScaleX, 1.0);
    mConfigFileReader.getParam(nhPrivate, "PoseScaleY", mPoseScaleY, 1.0);
    mConfigFileReader.getParam(nhPrivate, "PoseScaleZ", mPoseScaleZ, 1.0);

    // Publishers
    mSetPointPositionPublisher = mNh.advertise<visualization_msgs::Marker>("/rviz_setpoint_marker", 10);
    mGoalPositionPublisher = mNh.advertise<visualization_msgs::Marker>("/rviz_goal_marker", 10);
    mSystemPosePublisher = mNh.advertise<visualization_msgs::Marker>("/rviz_systempose_marker", 10);

    // Subscribers
    for(int i = 0; i < mNumberOfUAV; i++)
    {
        std::string setPointPositionTopic = "iris" + std::to_string(i) + "/setpoint_position";
        std::string systemPoseTopic = "iris" + std::to_string(i) + "/system_pose";

        ros::Subscriber setPointPositionSubscriber = mNh.subscribe<geometry_msgs::PoseStamped>(setPointPositionTopic, 10, std::bind(&Visualisation::UAVSetPointPositionCallback, this, std::placeholders::_1, i));
        ros::Subscriber systemPoseSubscriber = mNh.subscribe<geometry_msgs::PoseStamped>(systemPoseTopic, 10, std::bind(&Visualisation::UAVSystemPoseCallback, this, std::placeholders::_1, i));

        mUAVSetPointPositionSubscriberVector.push_back(setPointPositionSubscriber);
        mUAVSystemPoseSubscriberVector.push_back(systemPoseSubscriber);
    }

    mGoalPositionSubscriber = mNh.subscribe<mt_msgs::pose>("/goal", 10, &Visualisation::GoalPositionCallback, this);
    mHumanSystemPoseSubscriber = mNh.subscribe<geometry_msgs::PoseStamped>("human/system_pose", 10, &Visualisation::HumanSystemPoseCallback, this);

    // Visualisation Markers
    for(int i = 0; i < mNumberOfUAV; i++)
    {
        double r, g, b;
        GetRGBColorFromIndex(i, r, g, b);

        visualization_msgs::Marker setPointPositionMarker;
        setPointPositionMarker.header.frame_id = "map";
        setPointPositionMarker.header.stamp = ros::Time();
        setPointPositionMarker.ns = "iris" + std::to_string(i) + "_setpoint";
        setPointPositionMarker.id = i;
        setPointPositionMarker.type = visualization_msgs::Marker::ARROW;
        setPointPositionMarker.action = visualization_msgs::Marker::ADD;
        setPointPositionMarker.scale.x = mUAVScaleX;
        setPointPositionMarker.scale.y = mUAVScaleY;
        setPointPositionMarker.scale.z = mUAVScaleZ;
        setPointPositionMarker.color.a = 1.0;
        setPointPositionMarker.color.r = r;
        setPointPositionMarker.color.g = g;
        setPointPositionMarker.color.b = b;
        setPointPositionMarker.lifetime = ros::Duration(0);
        mUAVSetPointPositionMarkerVector.push_back(setPointPositionMarker);

        visualization_msgs::Marker systemPoseMarker;
        systemPoseMarker.header.frame_id = "map";
        systemPoseMarker.header.stamp = ros::Time();
        systemPoseMarker.ns = "iris" + std::to_string(i) + "_pose";
        systemPoseMarker.id = i;
        systemPoseMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
        systemPoseMarker.action = visualization_msgs::Marker::ADD;
        systemPoseMarker.scale.x = mPoseScaleX;
        systemPoseMarker.scale.y = mPoseScaleY;
        systemPoseMarker.scale.z = mPoseScaleZ;
        systemPoseMarker.color.a = 1.0;
        systemPoseMarker.color.r = r;
        systemPoseMarker.color.g = g;
        systemPoseMarker.color.b = b;
        systemPoseMarker.mesh_resource = "package://rotors_description/meshes/iris.dae";
        systemPoseMarker.lifetime = ros::Duration(0);
        mUAVSystemPoseMarkerVector.push_back(systemPoseMarker);
    }

    mGoalPositionMarker.header.frame_id = "map";
    mGoalPositionMarker.header.stamp = ros::Time();
    mGoalPositionMarker.ns = "goal";
    mGoalPositionMarker.id = 0;
    mGoalPositionMarker.type = visualization_msgs::Marker::ARROW;
    mGoalPositionMarker.action = visualization_msgs::Marker::ADD;
    mGoalPositionMarker.scale.x = mGoalScaleX;
    mGoalPositionMarker.scale.y = mGoalScaleY;
    mGoalPositionMarker.scale.z = mGoalScaleZ;
    mGoalPositionMarker.color.a = 1.0;
    mGoalPositionMarker.color.r = 1.0;
    mGoalPositionMarker.color.g = 1.0;
    mGoalPositionMarker.color.b = 1.0;
    mGoalPositionMarker.lifetime = ros::Duration(0);

    mHumanSystemPoseMarker.header.frame_id = "map";
    mHumanSystemPoseMarker.header.stamp = ros::Time();
    mHumanSystemPoseMarker.ns = "human_pose";
    mHumanSystemPoseMarker.id = 0;
    mHumanSystemPoseMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
    mHumanSystemPoseMarker.action = visualization_msgs::Marker::ADD;
    mHumanSystemPoseMarker.scale.x = mPoseScaleX;
    mHumanSystemPoseMarker.scale.y = mPoseScaleY;
    mHumanSystemPoseMarker.scale.z = mPoseScaleZ;
    mHumanSystemPoseMarker.color.a = 1.0;
    mHumanSystemPoseMarker.color.r = 1.0;
    mHumanSystemPoseMarker.color.g = 1.0;
    mHumanSystemPoseMarker.color.b = 1.0;
    mHumanSystemPoseMarker.mesh_resource = "package://rotors_description/meshes/human.dae";
    mHumanSystemPoseMarker.lifetime = ros::Duration(0);
}

Visualisation::~Visualisation()
{
    // Destructor
}
