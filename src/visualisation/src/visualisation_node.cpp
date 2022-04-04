#include "visualisation.cpp"
#include "visualisation_middleware.cpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualisation");
    ros::NodeHandle nh("");
    ros::NodeHandle nhPrivate("~");

    std::unique_ptr<Visualisation> visualisationModule(new Visualisation(nh, nhPrivate));
    ros::spin();
    return 0;
}