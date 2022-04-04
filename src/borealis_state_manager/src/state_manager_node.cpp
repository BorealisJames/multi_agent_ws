#include "state_manager.cpp"
#include "state_manager_middleware.cpp"
#include "state_manager_controller.cpp"

int main (int argc, char** argv)
{  
    ros::init(argc, argv, "state_manager");
    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle);
    ros::NodeHandlePtr nhPrivate = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    std::unique_ptr<SM::StateManager> stateManager(new SM::StateManager(nh, nhPrivate));

    ros::spin();
    return 0;
}