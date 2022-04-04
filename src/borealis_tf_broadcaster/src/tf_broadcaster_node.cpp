#include "tf_broadcaster.cpp"
#include "tf_broadcaster_middleware.cpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle);
    ros::NodeHandlePtr nhPrivate = ros::NodeHandlePtr(new ros::NodeHandle("~"));
    TF::TfBroadcaster tfBroadcaster(nh, nhPrivate);

    ros::spin();
    return 0;
}
