#include "../include/borealis_sim_human_uav.h"


BorealisFollowMe::BorealisFollowMe(const ros::NodeHandle& nh, const ros::NodeHandle& nhPrivate):
    mNh(nh),
    mNhPrivate(nhPrivate)
{
    mModulePeriod = 1;

    //Subscriber
    mMavrosStateSubscriber = mNh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &BorealisFollowMe::mavrosStateCallback, this);

    // Mavros services
    mArmingClient = mNh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mSetModeClient = mNh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    
    // Publisher
    mTaskPublisher = mNh.advertise<mt_msgs::mtTask>("/task", 10);
    // Might not be needed see pubGoal comments bellow 
    // mGoalPublisher = mNh.advertise<mt_msgs::pose>("/goal", 10); 

    ros::Publisher local_pos_pub = mNh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::Rate rate(20.0);
    // wait for FCU connection
    while(ros::ok() && !mCurrentState.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 20; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    // mModuleLoopTimer = mNh.createTimer(ros::Duration(1/mModulePeriod), &BorealisFollowMe::moduleLoopCallback, this);

    ROS_INFO("Begin OFFBOARD and ARMING");
    while(ros::ok()){
        if( mCurrentState.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( mSetModeClient.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Human Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !mCurrentState.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( mArmingClient.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Human Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void BorealisFollowMe::mavrosStateCallback(const mavros_msgs::State::ConstPtr& aState)
{
    mCurrentState = *aState;
}


void BorealisFollowMe::moduleLoopCallback(const ros::TimerEvent& aEvent)
{
    // pubGoal(mGoal);
    mTask.type = Common::Entity::MTTaskEnum::FOLLOW_ME;
    pubTask(mTask);
}

bool BorealisFollowMe::pubTask(const Common::Entity::MTTaskBundle aTask)
{
    bool status = true;

    mt_msgs::mtTask tmp;
    tmp.header.stamp = ros::Time::now();
    tmp.type = static_cast<int8_t>(aTask.type);

    mTaskPublisher.publish(tmp);
    return status;
}

BorealisFollowMe::~BorealisFollowMe()
{
    // Destructor
}