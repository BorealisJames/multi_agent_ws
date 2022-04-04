#include "../include/borealis_HRI_interface.h"

BorealisHRIInterface::BorealisHRIInterface(const ros::NodeHandle& nh, const ros::NodeHandle& nhPrivate):
    mNh(nh),
    mNhPrivate(nhPrivate)
{
    // Configurable Variables
    mModulePeriod = 1;
    mTask.type = Common::Entity::MTTaskEnum::UNDEFINED;
    //Subscriber
    mHumanTaskSubscriber = mNh.subscribe("/hri_mode",10, &BorealisHRIInterface::HRITaskCallback, this);

    // Publisher
    mTaskPublisher = mNh.advertise<mt_msgs::mtTask>("/task", 10);
    // Might not be needed see pubGoal comments bellow 
    // mGoalPublisher = mNh.advertise<mt_msgs::pose>("/goal", 10);
    mModuleLoopTimer = mNh.createTimer(ros::Duration(1/mModulePeriod), &BorealisHRIInterface::moduleLoopCallback, this);
}

void BorealisHRIInterface::HRITaskCallback(const std_msgs::String::ConstPtr& msg)
{
    // ROS_INFO("HRI Node I heard: %s", msg->data.c_str());
    std::string str1 ("Follow_Me");
    std::string str2 ("Go_There");
    
    // ROS_INFO("compare results is this follow me? %i", str1.compare(msg->data.c_str()));
    if (str1.compare(msg->data.c_str()) == 0) // They compare equal
    {
        mTask.type = Common::Entity::MTTaskEnum::FOLLOW_ME;
    }
    if (str2.compare(msg->data.c_str()) == 0)
    {
        mTask.type = Common::Entity::MTTaskEnum::GO_THERE;

    }
}


void BorealisHRIInterface::moduleLoopCallback(const ros::TimerEvent& aEvent)
{
    // pubGoal(mGoal);
    // mTask.type = Common::Entity::MTTaskEnum::FOLLOW_ME;
    // UNDEFINED = 0,
    // GO_THERE = 1,
    // FOLLOW_ME = 2,
    // DISTRACT_TARGET = 3

    // Whatever lmao
    std::string task_string;
    switch (mTask.type) {
        case (Common::Entity::MTTaskEnum::UNDEFINED):
            task_string = "UNDEFINED";
            break;
        case (Common::Entity::MTTaskEnum::GO_THERE):
            task_string = "GO_THERE";
            break;
        case (Common::Entity::MTTaskEnum::FOLLOW_ME):
            task_string = "Follow_Me";
            break;
        case (Common::Entity::MTTaskEnum::DISTRACT_TARGET):
            task_string = "DISTRACT_TARGET";
            break;
    }
    ROS_INFO("Agent %i: Publishing task %s", 1,task_string.c_str());
    
    pubTask(mTask);
}

bool BorealisHRIInterface::pubTask(const Common::Entity::MTTaskBundle aTask)
{
    bool status = true;

    mt_msgs::mtTask tmp;
    tmp.header.stamp = ros::Time::now();
    tmp.type = static_cast<int8_t>(aTask.type);

    mTaskPublisher.publish(tmp);
    return status;
}

// Might not be neeeded since the planner itself already subscribe to human system pose as the goal
// bool BorealisHRIInterface::pubGoal(DistributedFormation::Common::Pose aGoal)
// {
//     bool status = true;

//     mt_msgs::pose tmp;
//     tmp.header.stamp = ros::Time::now();
//     tmp.position.x = aGoal.position.x;
//     tmp.position.y = aGoal.position.y;
//     tmp.position.z = aGoal.position.z;
//     tmp.headingRad = aGoal.headingRad;

//     mGoalPublisher.publish(tmp);
//     return status;
// }

BorealisHRIInterface::~BorealisHRIInterface()
{
    // Destructor
}