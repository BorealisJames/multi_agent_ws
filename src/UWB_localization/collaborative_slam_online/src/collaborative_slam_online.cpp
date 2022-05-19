#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf_conversions/tf_eigen.h"
#include <tf2_ros/transform_broadcaster.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <mutex>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/circular_buffer.hpp>

#include "gnuplot.h"
#include "text/to_string.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>



#include <g2o/types/slam2d/parameter_se2_offset.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>
#include <g2o/types/slam3d/edge_se3_pointxyz_distance.h>
#include <g2o/types/slam3d/edge_se3_distance.h>
#include <g2o/types/slam3d/types_edge_se3range.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/robust_kernel_factory.h>
#include "g2o/core/estimate_propagator.h"

#include "carmen.h"

#include "nlink_parser/LinktrackNodeframe2.h"

using namespace g2o;

//UWb message
class RangingMeasurement
{
public: RangingMeasurement(){}
double timestamp;//timestamp in seconds
long int local_time;//local time of a UWB device
long int system_time;//system of a UWB network
std::string tag_id;//tag id
std::map<std::string, double> v_ranging;//rangings to the anchors
std::map<std::string, double> v_rss_difference;//Absolute RSS difference be rxRSS and fpRSS
};

// the robot pose with the uwb readings 
class RobotPose
{
public:
	RobotPose()
{

}
	double timestamp;//timestamp is in double and in seconds
	double x;
	double y;
	double z;
	double roll;
	double pitch;
	double yaw;
	double qx;
	double qy;
	double qz;
	double qw;
	double acc_d;//accumulated distance the robot moves
	double acc_orientation;//accumulated angles the robot rotates
        //We have the ranging measurement here
	std::map<std::string, RangingMeasurement> v_uwb_measure;//UWb measurement of a given tag installed on the robot
};


//plot for the trajectories
GnuplotInterface * m_plot_optimized_track= new GnuplotInterface();
GnuplotInterface * m_plot_published_pose= new GnuplotInterface();


//defines the areas will be plotted
double m_plot_min_x=-60;
double m_plot_max_x=20;
double m_plot_min_y=-10;
double m_plot_max_y=60;

/** Constraints/connections of robot poses */
class Connection
{
public:
	Connection()
{

}
	int from;
	int to;
};


/** UWB connection from robot external UWBs*/
class UWBConnection
{
public:
	UWBConnection()
{

}
	int robot;
	std::string uwb;
};


int m_first_connection_between_robots=0;//indicator if we already have a connection between two robots, used to fix the initial poses of two robot
int m_first_node_fixed=0;//first node in the pose graph needs to be fixed
int m_vertex_index=0;//count of the vertex in pose graph


int m_optimizer_first_time=1;//indicator the optimizer has been initialized. If initialized, online incremental optimization will be used

// Set up the optimiser
SparseOptimizer m_optimizer;//global pointer to the optimizer

int m_max_iterations=100;//maximum interations for the initial optimization 
int m_incremental_iterations=40;//interations for the incremental optimization 
double m_uwb_range_time_window=0.1;//time window to average the uwb ranging
std::map<std::string, int > m_uwb_nodes_external;//list of external UWBs
std::map<std::string, int > m_uwb_nodes_dog1;//list of uwbs on the dog1
std::map<std::string, int > m_uwb_nodes_dog2;//list of uwbs on the dog2

std::map<std::string, int > m_uwb_nodes_human;//list of uwbs on the human

ros::Publisher  m_pub_human_pose;//human odom publisher, which is calculated based on the dog pose and our collabrative SLAM
ros::Publisher  m_pub_dog1_pose;//human odom publisher, which is calculated based on the dog pose and our collabrative SLAM
ros::Publisher  m_pub_dog2_pose;//human odom publisher, which is calculated based on the dog pose and our collabrative SLAM

ros::Publisher  m_pub_human_human_pose;//publish the position of human in human local frame
ros::Publisher  m_pub_human_dog1_pose;//publish the position of dog1 in human local frame
ros::Publisher  m_pub_human_dog2_pose;//publish the position of dog2 in human local frame

ros::Publisher  m_pub_dog1_human_pose;//publish the position of human in dog1 local frame
ros::Publisher  m_pub_dog1_dog1_pose;//publish the position of dog1 in dog1 local frame
ros::Publisher  m_pub_dog1_dog2_pose;//publish the position of dog2 in dog1 local frame

ros::Publisher  m_pub_dog2_human_pose;//publish the position of human in dog2 local frame
ros::Publisher  m_pub_dog2_dog1_pose;//publish the position of dog1 in dog2 local frame
ros::Publisher  m_pub_dog2_dog2_pose;//publish the position of dog2 in dog2 local frame

std::string m_str_human_in_human_localframe_published_topic="/HumanInHumanLocalFrame";
std::string m_str_dog1_in_human_localframe_published_topic="/UAV1InHumanLocalFrame";
std::string m_str_dog2_in_human_localframe_published_topic="/UAV2InHumanLocalFrame";

std::string	m_str_human_in_dog1_localframe_published_topic="/HumanInUAV1LocalFrame";
std::string	m_str_dog1_in_dog1_localframe_published_topic="/UAV1InUAV1LocalFrame";
std::string	m_str_dog2_in_dog1_localframe_published_topic="/UAV2InUAV1LocalFrame";

std::string	m_str_human_in_dog2_localframe_published_topic="/HumanInUAV2LocalFrame";
std::string	m_str_dog1_in_dog2_localframe_published_topic="/UAV1InUAV2LocalFrame";
std::string	m_str_dog2_in_dog2_localframe_published_topic="/UAV2InUAV2LocalFrame";


double m_acc_distance_human_odom=0;//accumulated distance of human
double m_acc_orientation_human_odom=0;//accumulated orientation of human 
double m_last_timestamp_human_odom=0;//timestamp of human odom 


double m_last_acc_distance_human_odom=0;//previous accumulated distance of human
double m_last_acc_orientation_human_odom=0;//previous accumulated orientation of human

double m_acc_distance_dog1_odom=0;//accumulated distance of dog1
double m_acc_orientation_dog1_odom=0;//accumulated orientation of dog1
double m_last_timestamp_dog1_odom=0;//timestamp of dog1 odom 


double m_last_acc_distance_dog1_odom=0;//previous accumulated distance of dog1
double m_last_acc_orientation_dog1_odom=0;//previous accumulated orientation of dog1
double m_last_timestamp_dog2_odom=0;//timestamp of dog2 odom 




double m_acc_distance_dog2_odom=0;//accumulated distance of dog2
double m_acc_orientation_dog2_odom=0;//accumulated orientation of dog2

double m_last_acc_distance_dog2_odom=0;//previous accumulated distance of dog2
double m_last_acc_orientation_dog2_odom=0;//previous accumulated orientation of dog2



RobotPose m_latest_human_odom;//latest human odometry
RobotPose m_latest_dog1_odom;//latest dog1 odometry
RobotPose m_latest_dog2_odom;//latest dog2 odometry

int m_index_human_odom=0;//human odometry index
int m_index_dog1_odom=0;//dog1 odometry index
int m_index_dog2_odom=0;//dog2 odometry index

double m_update_min_d=0.5;//minumun distance required for a pose update
double m_update_min_a=0.5;//minumun angle required for a pose update
double m_update_min_time=5.0;//minumun time required for a pose update


double m_human_init_x=0;//human init x
double m_human_init_y=0;//human init y
double m_human_init_z=0;//human init z
double m_human_init_theta=0;//human init theta

double m_dog1_init_x=0;//dog1 init x
double m_dog1_init_y=0;//dog1 init y
double m_dog1_init_z=0;//dog1 init z
double m_dog1_init_theta=0;//dog1 init theta

double m_dog2_init_x=0;//dog2 init x
double m_dog2_init_y=0;//dog2 init y
double m_dog2_init_z=0;//dog2 init z
double m_dog2_init_theta=0;//dog2 init theta


g2o::SE3Quat m_human_init_se3;//the initial human pose
g2o::SE3Quat m_dog1_init_se3;//the initial dog1 pose
g2o::SE3Quat m_dog2_init_se3;//the initial dog2 pose



std::map<int, RobotPose > m_vDog1Odometry;//dog1 odometry
std::map<int, RobotPose > m_vDog2Odometry;//dog2 odometry
std::map<int, RobotPose > m_vHumanOdometry;//human odometry

boost::circular_buffer<RangingMeasurement> m_vDog1UWBMeasurements(300);//the buffered dog1 uwb measurements
boost::circular_buffer<RangingMeasurement> m_vDog2UWBMeasurements(300);//the buffered dog2 uwb measurements 
boost::circular_buffer<RangingMeasurement> m_vHumanUWBMeasurements(300);//the buffered human uwb measurements


//for dog1
std::map<int, int > m_vertex_odom_map_dog1;//dog1 vertex in the pose graph --> id in the odom
std::map<int, int > m_odom_vertex_map_dog1;//dog1 id in the odom --> vertex in the pose graph 


//for dog2
std::map<int, int > m_vertex_odom_map_dog2;//dog2 vertex in the pose graph --> id in the odom
std::map<int, int > m_odom_vertex_map_dog2;//dog2 id in the odom --> vertex in the pose graph 


//for human
std::map<int, int > m_vertex_odom_map_human;//human vertex in the pose graph --> id in the odom
std::map<int, int > m_odom_vertex_map_human;//human id in the odom --> vertex in the pose graph


//for uwb pose
std::map<int, std::string > m_vertex_uwb_map;//UWB vertex in the pose graph --> id of the uwb
std::map<std::string, int > m_uwb_vertex_map;//id of the uwb --> UWB vertex in the pose graph

int m_first_odom_vertex_dog1=-1;//first vertex of dog1
int m_first_odom_vertex_dog2=-1;//first vertex of dog2
int m_first_odom_vertex_human=-1;//first vertex of human

std::vector< Connection > m_vOdomConstraints_human;//odometry constaints of human
std::vector< Connection > m_vOdomConstraints_dog1;//odometry constraints of the dog1
std::vector< Connection > m_vOdomConstraints_dog2;//odometry constraints of the dog2

std::vector< UWBConnection > m_v_uwb_connections_dog1;//dog1 uwb constraints to the fixed uwb anchors
std::vector< UWBConnection > m_v_uwb_connections_dog2;//dog2 uwb constraints to the fixed uwb anchors
std::vector< UWBConnection > m_v_uwb_connections_human;//human uwb constraints to the fixed uwb anchors

int m_num_connection_fixed_dog1=0;//number of external UWB connections of dog1
int m_num_connection_fixed_dog2=0;//number of external UWB connections of dog2
int m_num_connection_fixed_human=0;//number of external UWB connections of human

int m_num_connection_human_dog1=0;//number of connections from human to dog1
int m_num_connection_dog1_human=0;//number of connections from dog to human


int m_num_connection_human_dog2=0;//number of connections from human to dog2
int m_num_connection_dog2_human=0;//number of connections from dog2 to human


int m_num_connection_dog1_dog2=0;//number of connections from dog1 to dog2
int m_num_connection_dog2_dog1=0;//number of connections from dog2 to dog1


std::vector< Connection > m_v_connections_dog1_human;//UWB connections between dog1 and human
std::vector< Connection > m_v_connections_dog2_human;//UWB connections between dog2 and human
std::vector< Connection > m_v_connections_dog1_dog2;//UWB connections between dog and human

int m_index_updated_pose_dog1=-1;//index of the dog1 pose optimized by SLAM 
int m_index_updated_pose_dog2=-1;//index of the dog2 pose optimized by SLAM 
int m_index_updated_pose_human=-1;//index of the robot pose optimized by SLAM

RobotPose m_updated_pose_dog1;//the last updated pose of the dog1 
RobotPose m_updated_pose_dog2;//the last updated pose of the dog2 
RobotPose m_updated_pose_human;//the last updated pose of the human

RobotPose m_initial_dog1_odom;//initial odometry of the dog1 
RobotPose m_initial_dog2_odom;//initial odometry of the dog2 
RobotPose m_initial_human_odom;//initial odometry of the human




g2o::SE3Quat m_first_vertex_human_se3;//the human pose of first vertex in the graph 
g2o::SE3Quat m_first_vertex_dog1_se3;//the dog1 pose of first vertex in the graph
g2o::SE3Quat m_first_vertex_dog2_se3;//the dog2 pose of first vertex in the graph

std::string m_str_human_odom_topic;//human odometry topic
std::string m_str_dog1_odom_topic;//dog1 odometry topic
std::string m_str_dog2_odom_topic;//dog1 odometry topic
std::string m_str_human_frame_id;//human frame id
std::string m_str_dog1_frame_id;//dog1 frame id
std::string m_str_dog2_frame_id;//dog2 frame id

std::string m_str_human_child_frame_id;//human child frame id
std::string m_str_dog1_child_frame_id;//dog1 child frame id
std::string m_str_dog2_child_frame_id;//dog2 child frame id


std::string m_str_dog1_uwb_topic;//dog1 uwb topic
std::string m_str_dog2_uwb_topic;//dog2 uwb topic
std::string m_str_human_uwb_topic;//human uwb topic		

std::string m_str_human_pose_published_topic="/HumanPose";//published human odom
std::string m_str_dog1_pose_published_topic="/UAV1Pose";//published human odom
std::string m_str_dog2_pose_published_topic="/UAV2Pose";//published human odom

double m_human_odom_tran_noise=0.5;
double m_human_odom_rot_noise=0.01;
double m_dog_odom_tran_noise=0.01;
double m_dog_odom_rot_noise=0.01;

double m_max_peer_range=100;//maximum UWB ranging between the robot
double m_max_range_external=100.0;//maximum UWB ranging to external UWBs

double m_close_range=4.0;//define close range for UWB

double m_far_uwb_noise=1.0;//noise for far range
double m_close_noise=0.1;//noise for close range
double m_far_uwb_information=1.0/(m_far_uwb_noise*m_far_uwb_noise);//UWB information for far range
double m_close_uwb_information=1.0/(m_close_noise*m_close_noise);//define the close range



double m_initial_guess_information=10000;

double m_information_gain=1e-10;//UWB information in the SLAM

bool m_enable_gnuplot=false;//enable the gnuplot of the trajectory
int m_optimization_interval=10;//optimize interval in seconds
bool m_use_mean_ranging=false;//use mean ranging

bool m_use_closest_ranging=true;//use closest uwb ranging with in time (no smooth at all)

std::vector < RobotPose > v_published_pose_dog1;//published dog1 pose after SLAM
std::vector < RobotPose > v_published_pose_dog2;//published dog2 pose after SLAM
std::vector < RobotPose > v_published_pose_human;//published dog pose after SLAM

int m_peer_uwb_optimization=1;
int m_external_uwb_optimization=1;

double m_external_uwb_init_range=10.0;

std::mutex m_pose_update_mutex;

bool m_force_human_2d=false;
bool m_force_robot_2d=false;


// --------------------------------------------------------------------------
g2o::SE3Quat getUpdatedPoseSE3(RobotPose recent_updated_odom, 
		g2o::SE3Quat updated_pose, 
		RobotPose latest_odom)
//recent_updated_odom: odom with respect to updated_pose
//relative pose: updated_pose*recent_updated_odom.inverse*latest_odom
// --------------------------------------------------------------------------
{
	//we get the odom of the last update
	double recent_odom_x=recent_updated_odom.x;
	double recent_odom_y=recent_updated_odom.y;
	double recent_odom_z=recent_updated_odom.z;

	double recent_odom_qx=recent_updated_odom.qx;
	double recent_odom_qy=recent_updated_odom.qy;
	double recent_odom_qz=recent_updated_odom.qz;
	double recent_odom_qw=recent_updated_odom.qw;

	//we get the current odom
	double x=latest_odom.x;
	double y=latest_odom.y;
	double z=latest_odom.z;

	double qx=latest_odom.qx;
	double qy=latest_odom.qy;
	double qz=latest_odom.qz;
	double qw=latest_odom.qw;

	Eigen::Vector3d trans_recent_odom(recent_odom_x,recent_odom_y,recent_odom_z);
	Eigen::Quaterniond q_recent_odom(recent_odom_qw,recent_odom_qx,recent_odom_qy,recent_odom_qz);//w,x,y,z
	g2o::SE3Quat recent_odom(q_recent_odom,trans_recent_odom);

	Eigen::Vector3d trans_odom(x,y,z);
	Eigen::Quaterniond q_odom(qw,qx,qy,qz);//w,x,y,z
	g2o::SE3Quat current_odom(q_odom,trans_odom);

	return updated_pose*recent_odom.inverse()*current_odom;
}


// --------------------------------------------------------------------------
g2o::SE3Quat getUpdatedPose(RobotPose recent_updated_odom, 
		RobotPose updated_pose, 
		RobotPose latest_odom)
//recent_updated_odom: odom with respect to updated_pose
//relative pose: updated_pose*recent_updated_odom.inverse*latest_odom
// --------------------------------------------------------------------------
{
	//we get the odom of the last update
	double recent_odom_x=recent_updated_odom.x;
	double recent_odom_y=recent_updated_odom.y;
	double recent_odom_z=recent_updated_odom.z;

	double recent_odom_qx=recent_updated_odom.qx;
	double recent_odom_qy=recent_updated_odom.qy;
	double recent_odom_qz=recent_updated_odom.qz;
	double recent_odom_qw=recent_updated_odom.qw;

	//we get the last pose from the optimization
	double recent_pose_x=updated_pose.x;
	double recent_pose_y=updated_pose.y;
	double recent_pose_z=updated_pose.z;

	double recent_pose_qx=updated_pose.qx;
	double recent_pose_qy=updated_pose.qy;
	double recent_pose_qz=updated_pose.qz;
	double recent_pose_qw=updated_pose.qw;

	//we get the current odom
	double x=latest_odom.x;
	double y=latest_odom.y;
	double z=latest_odom.z;

	double qx=latest_odom.qx;
	double qy=latest_odom.qy;
	double qz=latest_odom.qz;
	double qw=latest_odom.qw;

	Eigen::Vector3d trans_recent_odom(recent_odom_x,recent_odom_y,recent_odom_z);
	Eigen::Quaterniond q_recent_odom(recent_odom_qw,recent_odom_qx,recent_odom_qy,recent_odom_qz);//w,x,y,z
	g2o::SE3Quat recent_odom(q_recent_odom,trans_recent_odom);

	Eigen::Vector3d trans_odom(x,y,z);
	Eigen::Quaterniond q_odom(qw,qx,qy,qz);//w,x,y,z
	g2o::SE3Quat current_odom(q_odom,trans_odom);

	Eigen::Vector3d trans_recent_pose(recent_pose_x,recent_pose_y,recent_pose_z);
	Eigen::Quaterniond q_recent_pose(recent_pose_qw,recent_pose_qx,recent_pose_qy,recent_pose_qz);//w,x,y,z
	g2o::SE3Quat recent_pose(q_recent_pose,trans_recent_pose);
	return recent_pose*recent_odom.inverse()*current_odom;

}

// --------------------------------------------------------------------------
g2o::SE3Quat getHumanPoseBasedOnDogPose(RobotPose dog_pose, g2o::SE3Quat relative_pose)
// --------------------------------------------------------------------------
{
	Eigen::Vector3d trans_pose(dog_pose.x,dog_pose.y,dog_pose.z);
	Eigen::Quaterniond q_pose(dog_pose.qw,dog_pose.qx,dog_pose.qy,dog_pose.qz);//w,x,y,z
	g2o::SE3Quat pose_se3(q_pose,trans_pose);
	return pose_se3*relative_pose;

}


// --------------------------------------------------------------------------
//plot the optimized trajectories and the constraints
void plotOptimizedTrack(std::map < int, RobotPose > v_optimized_pose_dog1,
		std::map < int, RobotPose > v_optimized_pose_dog2,
		std::map < int, RobotPose > v_optimized_pose_human, 
		std::map < std::string, carmen_point_t > v_optimized_landmark_pose,
		std::vector< Connection > connections_dog1_human,
		std::vector< Connection > connections_dog2_human,
		std::vector< Connection > connections_dog1_dog2,		
		std::vector< UWBConnection > uwb_connecntions_dog1,
		std::vector< UWBConnection > uwb_connecntions_dog2,
		std::vector< UWBConnection > uwb_connecntions_human)
// --------------------------------------------------------------------------
{
	std::string cmd;//( "set size ratio 1\n");
	cmd+="set grid\n";
	cmd+="set xlabel 'x(m)'\n";
	cmd+="set ylabel 'y(m)'\n";
	cmd+="set title 'Collaborative SLAM (large weights for UWB in close proximity)'\n";
	cmd+="set xrange ["+toString(m_plot_min_x)+":"+toString(m_plot_max_x)+"]\n";
	cmd+="set yrange ["+toString(m_plot_min_y)+":"+toString(m_plot_max_y)+"]\n";
	cmd+="set size ratio -1\n";
	//cmd+="set key box bottom right\n";	
	cmd+="unset arrow\n";

	//connections to the external UWBS
	for(int i=0;i<uwb_connecntions_dog1.size();i++)
	{ 
		UWBConnection co=uwb_connecntions_dog1[i];
		cmd += "set arrow from "+toString(v_optimized_pose_dog1[co.robot].x)+ ','+toString(v_optimized_pose_dog1[co.robot].y)+
				','+toString(0)+" to " + toString(v_optimized_landmark_pose[co.uwb].x)+ ','+toString(v_optimized_landmark_pose[co.uwb].y)+','+toString(0) + " lc rgb '#00FF00' lw 1\n";
	}
	
	//connections to the external UWBS
	for(int i=0;i<uwb_connecntions_dog2.size();i++)
	{ 
		UWBConnection co=uwb_connecntions_dog2[i];
		cmd += "set arrow from "+toString(v_optimized_pose_dog2[co.robot].x)+ ','+toString(v_optimized_pose_dog2[co.robot].y)+
				','+toString(0)+" to " + toString(v_optimized_landmark_pose[co.uwb].x)+ ','+toString(v_optimized_landmark_pose[co.uwb].y)+','+toString(0) + " lc rgb '#00FF00' lw 1\n";
	}

	for(int i=0;i<uwb_connecntions_human.size();i++)
	{ 
		UWBConnection co=uwb_connecntions_human[i];
		cmd += "set arrow from "+toString(v_optimized_pose_human[co.robot].x)+ ','+toString(v_optimized_pose_human[co.robot].y)+
				','+toString(0)+" to " + toString(v_optimized_landmark_pose[co.uwb].x)+ ','+toString(v_optimized_landmark_pose[co.uwb].y)+','+toString(0) + " lc rgb '#00FF00' lw 1\n";
	}

	//connections to between the dog1 and human
	for(int i=0;i<connections_dog1_human.size();i++)
	{ 
		Connection co=connections_dog1_human[i];
		cmd += "set arrow from "+toString(v_optimized_pose_dog1[co.from].x)+ ','+toString(v_optimized_pose_dog1[co.from].y)+
				','+toString(0)+" to " + toString(v_optimized_pose_human[co.to].x)+ ','+toString(v_optimized_pose_human[co.to].y)+','+toString(0) + " lc rgb '#00FF00' lw 1\n";
	}


	//connections to between the dog1 and dog2
	for(int i=0;i<connections_dog1_dog2.size();i++)
	{ 
		Connection co=connections_dog1_dog2[i];
		cmd += "set arrow from "+toString(v_optimized_pose_dog1[co.from].x)+ ','+toString(v_optimized_pose_dog1[co.from].y)+
				','+toString(0)+" to " + toString(v_optimized_pose_dog2[co.to].x)+ ','+toString(v_optimized_pose_dog2[co.to].y)+','+toString(0) + " lc rgb '#00FF00' lw 1\n";
	}

	//connections to between the dog2 and human
	for(int i=0;i<connections_dog2_human.size();i++)
	{ 
		Connection co=connections_dog2_human[i];
		cmd += "set arrow from "+toString(v_optimized_pose_dog2[co.from].x)+ ','+toString(v_optimized_pose_dog2[co.from].y)+
				','+toString(0)+" to " + toString(v_optimized_pose_human[co.to].x)+ ','+toString(v_optimized_pose_human[co.to].y)+','+toString(0) + " lc rgb '#00FF00' lw 1\n";
	}


	cmd+="plot '-' u 1:2 w l pt 1 lw 3 lt 1 lc rgb '#FF0000' ti 'UAV1','-' u 1:2 w l pt 1 lw 3 lt 1 lc rgb '#FF00FF' ti 'UAV2', '-' u 1:2 w l pt 2 lw 3 lt 2 lc rgb '#0000FF' ti 'Human','-' u 1:2 w p pt 3 lw 2 lt 3 lc rgb '#FF00FF' ti 'UWB anchors',\n";

	std::map < int, RobotPose >::iterator it_;

	for(it_ = v_optimized_pose_dog1.begin(); it_ != v_optimized_pose_dog1.end(); ++it_)
	{
		cmd += toString( it_->second.x ) + ' ' + toString( it_->second.y) + ' ' + toString(0) + '\n';
	}
	cmd += "e\n";
	
	for(it_ = v_optimized_pose_dog2.begin(); it_ != v_optimized_pose_dog2.end(); ++it_)
	{
		cmd += toString( it_->second.x ) + ' ' + toString( it_->second.y) + ' ' + toString(0) + '\n';
	}
	cmd += "e\n";


	for(it_ = v_optimized_pose_human.begin(); it_ != v_optimized_pose_human.end(); ++it_)
	{
		cmd += toString( it_->second.x ) + ' ' + toString( it_->second.y) + ' ' + toString(0) + '\n';
	}
	cmd += "e\n";

	std::map < std::string, carmen_point_t >::iterator it_landmarker;
	for(it_landmarker = v_optimized_landmark_pose.begin(); it_landmarker != v_optimized_landmark_pose.end(); ++it_landmarker)
	{
		cmd += toString( it_landmarker->second.x ) + ' ' + toString( it_landmarker->second.y) + ' ' + toString(0) + '\n';
	}
	cmd += "e\n";

	m_plot_optimized_track->commandStr( cmd);
}


// --------------------------------------------------------------------------
//check if a new constraint should be added, return 1 if we need to add this constraint
int addConstraints(std::vector< Connection >& v_connections,		
		int index_from,		
		int index_to)
// --------------------------------------------------------------------------
{

	int newConstraint=1;

	for(int i=0;i<v_connections.size();i++)
	{
		if(v_connections[i].from==index_from && v_connections[i].to==index_to)
		{
			newConstraint=-1;
			break;
		}

	}

	if(newConstraint>0)
	{
		//we add a new constraint
		Connection c;
		c.from=index_from;
		c.to=index_to;
		v_connections.push_back(c);

	}

	return newConstraint;

}


// --------------------------------------------------------------------------
//check if a new constraint should be added, return 1 if we need to add this constraint
int addConstraints(SparseOptimizer& optimizer, 
		std::vector< UWBConnection >& v_connections, 
		HyperGraph::EdgeSet& edges,
		int robot_id, 
		int robot_vertex_id,
		std::string uwb_id,
		int uwb_vertex_id,
		double mean_ranging,
		double uwb_information,
		int robustKernel, 
		double bandwidth)
// --------------------------------------------------------------------------
{

	int newConstraint=1;

	for(int i=0;i<v_connections.size();i++)
	{
		if(v_connections[i].robot==robot_id && v_connections[i].uwb==uwb_id)
		{
			newConstraint=-1;
			break;
		}

	}

	if(newConstraint>0)
	{
		//we add a new constraint
		UWBConnection c;
		c.robot=robot_id;
		c.uwb=uwb_id;
		v_connections.push_back(c);

		//We add this constraint to the graph		
		g2o::EdgeSE3Range* landmarkObservation = new EdgeSE3Range;							
		landmarkObservation->vertices()[0] = optimizer.vertex(robot_vertex_id);
		landmarkObservation->vertices()[1] = optimizer.vertex(uwb_vertex_id);

		landmarkObservation->setMeasurement( mean_ranging );

		//we set the information matrix to be one, large value means we trust the measurement more
		Eigen::Matrix<double,1,1> information_matrix=uwb_information*Eigen::Matrix<double,1,1>::Identity();

		landmarkObservation->setInformation(information_matrix);
		landmarkObservation->setParameterId(0, 0);

		if(robustKernel>0)
		{
			landmarkObservation->setRobustKernel(new g2o::RobustKernelHuber());
			if(bandwidth>0)
			{
				landmarkObservation->robustKernel()->setDelta(bandwidth);	
			}
		}

		optimizer.addEdge(landmarkObservation);
		edges.insert(landmarkObservation);	

	}

	return newConstraint;

}



// --------------------------------------------------------------------------
void performPoseGraphOptimizationOnline(SparseOptimizer& optimizer,
		std::map<int, RobotPose > v_odom_dog1,
		std::map<int, RobotPose > v_odom_dog2,
		std::map<int, RobotPose > v_odom_human,
		int optimized_peer_uwb,
		int optimized_external_uwb,		
		int robustKernel,
		double bandwidth)
// --------------------------------------------------------------------------
{	

	ROS_INFO("Last optimized pose, dog1: %d, dog2: %d, human: %d", m_index_updated_pose_dog1, m_index_updated_pose_dog2, m_index_updated_pose_human);

	HyperGraph::VertexSet verticesAdded;//the vertices added to the pose graph for the current iteration
	HyperGraph::EdgeSet edgesAdded;//the constraints added to the pose graph for the current iteration

	std::map<int, RobotPose >::iterator it_odom;

	std::cout<<"Vertex_index: "<<m_vertex_index<<", dog1 vertex size: "<<m_odom_vertex_map_dog1.size()<<", dog2 vertex size:"<<m_odom_vertex_map_dog2.size()<<", human vertex size: "<<m_odom_vertex_map_human.size()<<std::endl;

	//adding dog1 pose to the graph
	for(it_odom = v_odom_dog1.begin(); it_odom != v_odom_dog1.end(); it_odom++)
	{
		std::map<int, int >::iterator it_odom_vertex;

		it_odom_vertex=m_odom_vertex_map_dog1.find(it_odom->first);

		if(it_odom_vertex==m_odom_vertex_map_dog1.end())
		{				
			if(m_index_updated_pose_dog1<0)				
			{
				//global offset
				tf::Quaternion tf_q_offset;
				tf_q_offset.setRPY( 0, 0, m_dog1_init_theta);
				Eigen::Vector3d relative_trans_offset(m_dog1_init_x,m_dog1_init_y,m_dog1_init_z);
				Eigen::Quaterniond q_offset(tf_q_offset.w(),tf_q_offset.x(),tf_q_offset.y(),tf_q_offset.z());//w,x,y,z
				g2o::SE3Quat se3_offset(q_offset,relative_trans_offset);
				ROS_ERROR("init dog1: %f %f %f ", m_dog1_init_x, m_dog1_init_y, m_dog1_init_z);

				//initial odom of dog
				Eigen::Vector3d initial_trans_dog(m_initial_dog1_odom.x,m_initial_dog1_odom.y,m_initial_dog1_odom.z);
				Eigen::Quaterniond initial_q_dog(m_initial_dog1_odom.qw,m_initial_dog1_odom.qx,m_initial_dog1_odom.qy,m_initial_dog1_odom.qz);//w,x,y,z
				g2o::SE3Quat se3_initial_dog(initial_q_dog,initial_trans_dog);

				double x=it_odom->second.x;
				double y=it_odom->second.y;
				double z=it_odom->second.z;

				double qx=it_odom->second.qx;
				double qy=it_odom->second.qy;
				double qz=it_odom->second.qz;
				double qw=it_odom->second.qw;

				Eigen::Vector3d trans(x,y,z);
				Eigen::Quaterniond q(qw,qx,qy,qz);//w,x,y,z
				g2o::SE3Quat current(q,trans);

				VertexSE3* robot =  new VertexSE3;
				robot->setId(m_vertex_index);
				m_vertex_odom_map_dog1[m_vertex_index]=it_odom->first;
				m_odom_vertex_map_dog1[it_odom->first]=m_vertex_index;

				g2o::SE3Quat pose_wrt_first_odom=se3_offset*se3_initial_dog.inverse()*current;

				robot->setEstimate(pose_wrt_first_odom);
				optimizer.addVertex(robot);

				if(m_first_odom_vertex_dog1<0)
				{
					m_first_odom_vertex_dog1=m_vertex_index;
					m_first_vertex_dog1_se3=pose_wrt_first_odom;
				}

				std::cout<<"Dog1 vertex initialization robot odom: "<<it_odom->first<<" "<<m_vertex_index<<std::endl;
				verticesAdded.insert(robot);
				m_vertex_index=m_vertex_index+1;

			}
			else
			{
				//If a SLAM pose is available.... 
				//we get the odom of the last update
				double recent_odom_x=v_odom_dog1[m_index_updated_pose_dog1].x;
				double recent_odom_y=v_odom_dog1[m_index_updated_pose_dog1].y;
				double recent_odom_z=v_odom_dog1[m_index_updated_pose_dog1].z;

				double recent_odom_qx=v_odom_dog1[m_index_updated_pose_dog1].qx;
				double recent_odom_qy=v_odom_dog1[m_index_updated_pose_dog1].qy;
				double recent_odom_qz=v_odom_dog1[m_index_updated_pose_dog1].qz;
				double recent_odom_qw=v_odom_dog1[m_index_updated_pose_dog1].qw;

				//we get the last pose from the optimization
				double recent_pose_x=m_updated_pose_dog1.x;
				double recent_pose_y=m_updated_pose_dog1.y;
				double recent_pose_z=m_updated_pose_dog1.z;

				double recent_pose_qx=m_updated_pose_dog1.qx;
				double recent_pose_qy=m_updated_pose_dog1.qy;
				double recent_pose_qz=m_updated_pose_dog1.qz;
				double recent_pose_qw=m_updated_pose_dog1.qw;

				//we get the current odom
				double x=it_odom->second.x;
				double y=it_odom->second.y;
				double z=it_odom->second.z;

				double qx=it_odom->second.qx;
				double qy=it_odom->second.qy;
				double qz=it_odom->second.qz;
				double qw=it_odom->second.qw;

				Eigen::Vector3d trans_recent_odom(recent_odom_x,recent_odom_y,recent_odom_z);
				Eigen::Quaterniond q_recent_odom(recent_odom_qw,recent_odom_qx,recent_odom_qy,recent_odom_qz);//w,x,y,z
				g2o::SE3Quat recent_odom(q_recent_odom,trans_recent_odom);

				Eigen::Vector3d trans_odom(x,y,z);
				Eigen::Quaterniond q_odom(qw,qx,qy,qz);//w,x,y,z
				g2o::SE3Quat current_odom(q_odom,trans_odom);

				Eigen::Vector3d trans_recent_pose(recent_pose_x,recent_pose_y,recent_pose_z);
				Eigen::Quaterniond q_recent_pose(recent_pose_qw,recent_pose_qx,recent_pose_qy,recent_pose_qz);//w,x,y,z
				g2o::SE3Quat recent_pose(q_recent_pose,trans_recent_pose);

				g2o::SE3Quat current_pose=recent_pose*recent_odom.inverse()*current_odom;					
				VertexSE3* robot =  new VertexSE3;
				robot->setId(m_vertex_index);
				m_vertex_odom_map_dog1[m_vertex_index]=it_odom->first;
				m_odom_vertex_map_dog1[it_odom->first]=m_vertex_index;

				robot->setEstimate(current_pose);
				optimizer.addVertex(robot);

				if(m_first_odom_vertex_dog1<0)
				{
					m_first_odom_vertex_dog1=m_vertex_index;
					m_first_vertex_dog1_se3=current_pose;

				}

				std::cout<<"Adding dog1 vertex: "<<it_odom->first<<" "<<m_index_updated_pose_dog1<<" "<<m_vertex_index<<" "<<
						recent_pose_x<<" "<<recent_pose_y<<" "<<recent_pose_z<<" "<<
						current_pose.translation()[0]<<" "<<current_pose.translation()[1]<<" "<<current_pose.translation()[2]<<std::endl;
				verticesAdded.insert(robot);
				m_vertex_index=m_vertex_index+1;
			}

		}

	}


	
	//adding dog2 pose to the graph
	for(it_odom = v_odom_dog2.begin(); it_odom != v_odom_dog2.end(); it_odom++)
	{
		std::map<int, int >::iterator it_odom_vertex;

		it_odom_vertex=m_odom_vertex_map_dog2.find(it_odom->first);

		if(it_odom_vertex==m_odom_vertex_map_dog2.end())
		{				
			if(m_index_updated_pose_dog2<0)				
			{
				//global offset
				tf::Quaternion tf_q_offset;
				tf_q_offset.setRPY( 0, 0, m_dog2_init_theta);
				Eigen::Vector3d relative_trans_offset(m_dog2_init_x,m_dog2_init_y,m_dog2_init_z);
				Eigen::Quaterniond q_offset(tf_q_offset.w(),tf_q_offset.x(),tf_q_offset.y(),tf_q_offset.z());//w,x,y,z
				g2o::SE3Quat se3_offset(q_offset,relative_trans_offset);
				ROS_ERROR("init dog2: %f %f %f ", m_dog2_init_x, m_dog2_init_y, m_dog2_init_z);


				//initial odom of dog
				Eigen::Vector3d initial_trans_dog(m_initial_dog2_odom.x,m_initial_dog2_odom.y,m_initial_dog2_odom.z);
				Eigen::Quaterniond initial_q_dog(m_initial_dog2_odom.qw,m_initial_dog2_odom.qx,m_initial_dog2_odom.qy,m_initial_dog2_odom.qz);//w,x,y,z
				g2o::SE3Quat se3_initial_dog(initial_q_dog,initial_trans_dog);

				double x=it_odom->second.x;
				double y=it_odom->second.y;
				double z=it_odom->second.z;

				double qx=it_odom->second.qx;
				double qy=it_odom->second.qy;
				double qz=it_odom->second.qz;
				double qw=it_odom->second.qw;

				Eigen::Vector3d trans(x,y,z);
				Eigen::Quaterniond q(qw,qx,qy,qz);//w,x,y,z
				g2o::SE3Quat current(q,trans);

				VertexSE3* robot =  new VertexSE3;
				robot->setId(m_vertex_index);
				m_vertex_odom_map_dog2[m_vertex_index]=it_odom->first;
				m_odom_vertex_map_dog2[it_odom->first]=m_vertex_index;

				g2o::SE3Quat pose_wrt_first_odom=se3_offset*se3_initial_dog.inverse()*current;

				robot->setEstimate(pose_wrt_first_odom);
				optimizer.addVertex(robot);

				if(m_first_odom_vertex_dog2<0)
				{
					m_first_odom_vertex_dog2=m_vertex_index;
					m_first_vertex_dog2_se3=pose_wrt_first_odom;
				}

				std::cout<<"Dog2 vertex initialization robot odom: "<<it_odom->first<<" "<<m_vertex_index<<std::endl;
				verticesAdded.insert(robot);
				m_vertex_index=m_vertex_index+1;

			}
			else
			{
				//If a SLAM pose is available.... 
				//We get the odom of the last update
				double recent_odom_x=v_odom_dog2[m_index_updated_pose_dog2].x;
				double recent_odom_y=v_odom_dog2[m_index_updated_pose_dog2].y;
				double recent_odom_z=v_odom_dog2[m_index_updated_pose_dog2].z;

				double recent_odom_qx=v_odom_dog2[m_index_updated_pose_dog2].qx;
				double recent_odom_qy=v_odom_dog2[m_index_updated_pose_dog2].qy;
				double recent_odom_qz=v_odom_dog2[m_index_updated_pose_dog2].qz;
				double recent_odom_qw=v_odom_dog2[m_index_updated_pose_dog2].qw;

				//we get the last pose from the optimization
				double recent_pose_x=m_updated_pose_dog2.x;
				double recent_pose_y=m_updated_pose_dog2.y;
				double recent_pose_z=m_updated_pose_dog2.z;

				double recent_pose_qx=m_updated_pose_dog2.qx;
				double recent_pose_qy=m_updated_pose_dog2.qy;
				double recent_pose_qz=m_updated_pose_dog2.qz;
				double recent_pose_qw=m_updated_pose_dog2.qw;

				//we get the current odom
				double x=it_odom->second.x;
				double y=it_odom->second.y;
				double z=it_odom->second.z;

				double qx=it_odom->second.qx;
				double qy=it_odom->second.qy;
				double qz=it_odom->second.qz;
				double qw=it_odom->second.qw;

				Eigen::Vector3d trans_recent_odom(recent_odom_x,recent_odom_y,recent_odom_z);
				Eigen::Quaterniond q_recent_odom(recent_odom_qw,recent_odom_qx,recent_odom_qy,recent_odom_qz);//w,x,y,z
				g2o::SE3Quat recent_odom(q_recent_odom,trans_recent_odom);

				Eigen::Vector3d trans_odom(x,y,z);
				Eigen::Quaterniond q_odom(qw,qx,qy,qz);//w,x,y,z
				g2o::SE3Quat current_odom(q_odom,trans_odom);

				Eigen::Vector3d trans_recent_pose(recent_pose_x,recent_pose_y,recent_pose_z);
				Eigen::Quaterniond q_recent_pose(recent_pose_qw,recent_pose_qx,recent_pose_qy,recent_pose_qz);//w,x,y,z
				g2o::SE3Quat recent_pose(q_recent_pose,trans_recent_pose);

				g2o::SE3Quat current_pose=recent_pose*recent_odom.inverse()*current_odom;					
				VertexSE3* robot =  new VertexSE3;
				robot->setId(m_vertex_index);
				m_vertex_odom_map_dog2[m_vertex_index]=it_odom->first;
				m_odom_vertex_map_dog2[it_odom->first]=m_vertex_index;

				robot->setEstimate(current_pose);
				optimizer.addVertex(robot);

				if(m_first_odom_vertex_dog2<0)
				{
					m_first_odom_vertex_dog2=m_vertex_index;
					m_first_vertex_dog2_se3=current_pose;

				}

				std::cout<<"Adding dog2 vertex: "<<it_odom->first<<" "<<m_index_updated_pose_dog2<<" "<<m_vertex_index<<" "<<
						recent_pose_x<<" "<<recent_pose_y<<" "<<recent_pose_z<<" "<<
						current_pose.translation()[0]<<" "<<current_pose.translation()[1]<<" "<<current_pose.translation()[2]<<std::endl;
				verticesAdded.insert(robot);
				m_vertex_index=m_vertex_index+1;
			}

		}

	}


	//adding human pose to the graph
	for(it_odom = v_odom_human.begin(); it_odom != v_odom_human.end(); it_odom++)
	{
		std::map<int, int >::iterator it_odom_vertex;

		it_odom_vertex=m_odom_vertex_map_human.find(it_odom->first);

		if(it_odom_vertex==m_odom_vertex_map_human.end())
		{

			if(m_index_updated_pose_human<0)				
			{
				//If a SLAM pose is not available...

				//global offset
				tf::Quaternion tf_q_offset;
				tf_q_offset.setRPY( 0, 0, m_human_init_theta);
				Eigen::Vector3d relative_trans_offset(m_human_init_x,m_human_init_y,m_human_init_z);
				Eigen::Quaterniond q_offset(tf_q_offset.w(),tf_q_offset.x(),tf_q_offset.y(),tf_q_offset.z());//w,x,y,z
				g2o::SE3Quat se3_offset(q_offset,relative_trans_offset);

				
				//initial odom of human
				Eigen::Vector3d initial_trans_human(m_initial_human_odom.x,m_initial_human_odom.y,m_initial_human_odom.z);
				Eigen::Quaterniond initial_q_human(m_initial_human_odom.qw,m_initial_human_odom.qx,m_initial_human_odom.qy,m_initial_human_odom.qz);//w,x,y,z
				g2o::SE3Quat se3_initial_human(initial_q_human,initial_trans_human);

				double x=it_odom->second.x;
				double y=it_odom->second.y;
				double z=it_odom->second.z;

				double qx=it_odom->second.qx;
				double qy=it_odom->second.qy;
				double qz=it_odom->second.qz;
				double qw=it_odom->second.qw;


				Eigen::Vector3d trans(x,y,z);
				Eigen::Quaterniond q(qw,qx,qy,qz);//w,x,y,z
				g2o::SE3Quat current(q,trans);

				VertexSE3* robot =  new VertexSE3;
				robot->setId(m_vertex_index);
				m_vertex_odom_map_human[m_vertex_index]=it_odom->first;
				m_odom_vertex_map_human[it_odom->first]=m_vertex_index;

				g2o::SE3Quat pose_wrt_first_odom=se3_offset*se3_initial_human.inverse()*current;

				robot->setEstimate(pose_wrt_first_odom);
				optimizer.addVertex(robot);
				if(m_first_odom_vertex_human<0)
				{
					m_first_odom_vertex_human=m_vertex_index;
					m_first_vertex_human_se3=pose_wrt_first_odom;
				}


				std::cout<<"Human vertex initialization robot odom: "<<it_odom->first<<" "<<m_vertex_index<<std::endl;
				verticesAdded.insert(robot);
				m_vertex_index=m_vertex_index+1;	
			}
			else
			{
				//If a SLAM pose is available...
				//we get the odom of the last update
				double recent_odom_x=v_odom_human[m_index_updated_pose_human].x;
				double recent_odom_y=v_odom_human[m_index_updated_pose_human].y;
				double recent_odom_z=v_odom_human[m_index_updated_pose_human].z;

				double recent_odom_qx=v_odom_human[m_index_updated_pose_human].qx;
				double recent_odom_qy=v_odom_human[m_index_updated_pose_human].qy;
				double recent_odom_qz=v_odom_human[m_index_updated_pose_human].qz;
				double recent_odom_qw=v_odom_human[m_index_updated_pose_human].qw;

				//we get the last pose from the optimization
				double recent_pose_x=m_updated_pose_human.x;
				double recent_pose_y=m_updated_pose_human.y;
				double recent_pose_z=m_updated_pose_human.z;

				double recent_pose_qx=m_updated_pose_human.qx;
				double recent_pose_qy=m_updated_pose_human.qy;
				double recent_pose_qz=m_updated_pose_human.qz;
				double recent_pose_qw=m_updated_pose_human.qw;

				//we get the current odom
				double x=it_odom->second.x;
				double y=it_odom->second.y;
				double z=it_odom->second.z;

				double qx=it_odom->second.qx;
				double qy=it_odom->second.qy;
				double qz=it_odom->second.qz;
				double qw=it_odom->second.qw;

				Eigen::Vector3d trans_recent_odom(recent_odom_x,recent_odom_y,recent_odom_z);
				Eigen::Quaterniond q_recent_odom(recent_odom_qw,recent_odom_qx,recent_odom_qy,recent_odom_qz);//w,x,y,z
				g2o::SE3Quat recent_odom(q_recent_odom,trans_recent_odom);

				Eigen::Vector3d trans_odom(x,y,z);
				Eigen::Quaterniond q_odom(qw,qx,qy,qz);//w,x,y,z
				g2o::SE3Quat current_odom(q_odom,trans_odom);

				Eigen::Vector3d trans_recent_pose(recent_pose_x,recent_pose_y,recent_pose_z);
				Eigen::Quaterniond q_recent_pose(recent_pose_qw,recent_pose_qx,recent_pose_qy,recent_pose_qz);//w,x,y,z
				g2o::SE3Quat recent_pose(q_recent_pose,trans_recent_pose);


				g2o::SE3Quat current_pose=recent_pose*recent_odom.inverse()*current_odom;					
				VertexSE3* robot =  new VertexSE3;
				robot->setId(m_vertex_index);
				m_vertex_odom_map_human[m_vertex_index]=it_odom->first;
				m_odom_vertex_map_human[it_odom->first]=m_vertex_index;

				robot->setEstimate(current_pose);
				optimizer.addVertex(robot);

				if(m_first_odom_vertex_human<0)
				{
					m_first_odom_vertex_human=m_vertex_index;
					m_first_vertex_human_se3=current_pose;
				}

				std::cout<<"Add human vertex: "<<it_odom->first<<" "<<m_index_updated_pose_human<<" "<<m_vertex_index<<" "<<
						recent_pose_x<<" "<<recent_pose_y<<" "<<recent_pose_z<<" "<<
						current_pose.translation()[0]<<" "<<current_pose.translation()[1]<<" "<<current_pose.translation()[2]<<std::endl;
				verticesAdded.insert(robot);
				m_vertex_index=m_vertex_index+1;
			}

		}


	}


	//noise of robot odometry
	double tran_noise=m_dog_odom_tran_noise;
	double rotation_noise=m_dog_odom_rot_noise*M_PI/180.0;
	double rotation_noise_tran=rotation_noise*0.0;
	double additional_noise_odom=0.001;
	
	//noise of human odometry
	double tran_noise_human=m_human_odom_tran_noise;
	double rotation_noise_human=m_human_odom_rot_noise*M_PI/180.0;
	double rotation_noise_tran_human=rotation_noise_human*0.0;
	double additional_noise_odom_human=0.001;



	//add odometry-based edge for the dog1
	for(it_odom = v_odom_dog1.begin(); it_odom != v_odom_dog1.end(); it_odom++)
	{		
		int previous_id=it_odom->first-1;

		std::map<int, int >::iterator it_current_odom;
		std::map<int, int >::iterator it_previous_odom;

		it_current_odom=m_odom_vertex_map_dog1.find(it_odom->first);
		it_previous_odom=m_odom_vertex_map_dog1.find(previous_id);

		if(it_current_odom!=m_odom_vertex_map_dog1.end()&&it_previous_odom!=m_odom_vertex_map_dog1.end())
		{
			if(addConstraints(m_vOdomConstraints_dog1, previous_id, it_odom->first)>0)
			{
				EdgeSE3* odometry = new EdgeSE3;
				odometry->vertices()[0] = optimizer.vertex(it_previous_odom->second);
				odometry->vertices()[1] = optimizer.vertex(it_current_odom->second);

				double previous_x=v_odom_dog1[previous_id].x;
				double previous_y=v_odom_dog1[previous_id].y;
				double previous_z=v_odom_dog1[previous_id].z;

				double previous_qx=v_odom_dog1[previous_id].qx;
				double previous_qy=v_odom_dog1[previous_id].qy;
				double previous_qz=v_odom_dog1[previous_id].qz;
				double previous_qw=v_odom_dog1[previous_id].qw;

				double x=it_odom->second.x;
				double y=it_odom->second.y;
				double z=it_odom->second.z;

				double qx=it_odom->second.qx;
				double qy=it_odom->second.qy;
				double qz=it_odom->second.qz;
				double qw=it_odom->second.qw;

				Eigen::Vector3d previous_trans(previous_x,previous_y,previous_z);
				Eigen::Quaterniond previous_q(previous_qw,previous_qx,previous_qy,previous_qz);//w,x,y,z

				Eigen::Vector3d trans(x,y,z);
				Eigen::Quaterniond q(qw,qx,qy,qz);//w,x,y,z

				g2o::SE3Quat previous(previous_q,previous_trans);
				g2o::SE3Quat current(q,trans);

				g2o::SE3Quat relative_odom=previous.inverse()*current;//relative transformtion T=inverse(A)*B,
				Eigen::Quaterniond qd_relative_odom= relative_odom.rotation();
				Eigen::Quaternion<float> q_relative_odom=relative_odom.rotation().cast<float>();
				Eigen::Vector3d t_relative_odom=relative_odom.translation();

				//we need to get the rpy from the q
				tf::Quaternion q_ros(qd_relative_odom.x(), qd_relative_odom.y(), qd_relative_odom.z(), qd_relative_odom.w());
				double roll, pitch, yaw;
				tf::Matrix3x3(q_ros).getRPY(roll, pitch, yaw);			


				double delta_x=t_relative_odom.x();
				double delta_y=t_relative_odom.y();
				double delta_z=t_relative_odom.z();

				double delta_roll=fabs(roll);
				double delta_pitch=fabs(pitch);
				double delta_yaw=fabs(yaw);


				double delta_d=sqrt(delta_x*delta_x+delta_y*delta_y+delta_z*delta_z);

				Eigen::Matrix<double,6,6> covariance;
				covariance.fill(0);

				//for x y z
				covariance(0,0)     = carmen_square( fabs(delta_x * tran_noise) +  additional_noise_odom );
				covariance(1,1)     = carmen_square( fabs(delta_y * tran_noise) +  additional_noise_odom );
				covariance(2,2)     = carmen_square( fabs(delta_z * tran_noise) +  additional_noise_odom );


				//for roll pitch and yaw
				covariance(3,3) = carmen_square( fabs(delta_roll * rotation_noise) + additional_noise_odom*M_PI/180.0 );
				covariance(4,4) = carmen_square( fabs(delta_pitch * rotation_noise) + additional_noise_odom*M_PI/180.0 );				
				covariance(5,5) = carmen_square( fabs(delta_yaw * rotation_noise) + fabs(delta_d*rotation_noise_tran)+ additional_noise_odom*M_PI/180.0 );


				Eigen::Matrix<double,6,6> information=Eigen::Matrix<double,6,6>::Identity();

				information(0,0)=1.0/covariance(0,0);
				information(1,1)=1.0/covariance(1,1);
				information(2,2)=1.0/covariance(2,2);				

				information(3,3)=1.0/covariance(3,3);
				information(4,4)=1.0/covariance(4,4);
				information(5,5)=1.0/covariance(5,5);

				odometry->setMeasurement(relative_odom);
				odometry->setInformation(information);

				if(robustKernel>0)
				{
					odometry->setRobustKernel(new g2o::RobustKernelHuber());

					if(bandwidth>0)
					{
						odometry->robustKernel()->setDelta(bandwidth);	
					}			

				}

				optimizer.addEdge(odometry);
				edgesAdded.insert(odometry);				
				std::cout<<"Dog1 odom: "<<previous_id<<" "<<it_odom->first<<std::endl;
			}

		}

	}

	
	//add odometry-based edge for the dog2
	for(it_odom = v_odom_dog2.begin(); it_odom != v_odom_dog2.end(); it_odom++)
	{		
		int previous_id=it_odom->first-1;

		std::map<int, int >::iterator it_current_odom;
		std::map<int, int >::iterator it_previous_odom;

		it_current_odom=m_odom_vertex_map_dog2.find(it_odom->first);
		it_previous_odom=m_odom_vertex_map_dog2.find(previous_id);

		if(it_current_odom!=m_odom_vertex_map_dog2.end()&&it_previous_odom!=m_odom_vertex_map_dog2.end())
		{
			if(addConstraints(m_vOdomConstraints_dog2, previous_id, it_odom->first)>0)
			{
				EdgeSE3* odometry = new EdgeSE3;
				odometry->vertices()[0] = optimizer.vertex(it_previous_odom->second);
				odometry->vertices()[1] = optimizer.vertex(it_current_odom->second);

				double previous_x=v_odom_dog2[previous_id].x;
				double previous_y=v_odom_dog2[previous_id].y;
				double previous_z=v_odom_dog2[previous_id].z;

				double previous_qx=v_odom_dog2[previous_id].qx;
				double previous_qy=v_odom_dog2[previous_id].qy;
				double previous_qz=v_odom_dog2[previous_id].qz;
				double previous_qw=v_odom_dog2[previous_id].qw;

				double x=it_odom->second.x;
				double y=it_odom->second.y;
				double z=it_odom->second.z;

				double qx=it_odom->second.qx;
				double qy=it_odom->second.qy;
				double qz=it_odom->second.qz;
				double qw=it_odom->second.qw;

				Eigen::Vector3d previous_trans(previous_x,previous_y,previous_z);
				Eigen::Quaterniond previous_q(previous_qw,previous_qx,previous_qy,previous_qz);//w,x,y,z

				Eigen::Vector3d trans(x,y,z);
				Eigen::Quaterniond q(qw,qx,qy,qz);//w,x,y,z

				g2o::SE3Quat previous(previous_q,previous_trans);
				g2o::SE3Quat current(q,trans);

				g2o::SE3Quat relative_odom=previous.inverse()*current;//relative transformtion T=inverse(A)*B,
				Eigen::Quaterniond qd_relative_odom= relative_odom.rotation();
				Eigen::Quaternion<float> q_relative_odom=relative_odom.rotation().cast<float>();
				Eigen::Vector3d t_relative_odom=relative_odom.translation();

				//we need to get the rpy from the q
				tf::Quaternion q_ros(qd_relative_odom.x(), qd_relative_odom.y(), qd_relative_odom.z(), qd_relative_odom.w());
				double roll, pitch, yaw;
				tf::Matrix3x3(q_ros).getRPY(roll, pitch, yaw);			


				double delta_x=t_relative_odom.x();
				double delta_y=t_relative_odom.y();
				double delta_z=t_relative_odom.z();

				double delta_roll=fabs(roll);
				double delta_pitch=fabs(pitch);
				double delta_yaw=fabs(yaw);


				double delta_d=sqrt(delta_x*delta_x+delta_y*delta_y+delta_z*delta_z);

				Eigen::Matrix<double,6,6> covariance;
				covariance.fill(0);

				//for x y z
				covariance(0,0)     = carmen_square( fabs(delta_x * tran_noise) +  additional_noise_odom );
				covariance(1,1)     = carmen_square( fabs(delta_y * tran_noise) +  additional_noise_odom );
				covariance(2,2)     = carmen_square( fabs(delta_z * tran_noise) +  additional_noise_odom );


				//for roll pitch and yaw
				covariance(3,3) = carmen_square( fabs(delta_roll * rotation_noise) + additional_noise_odom*M_PI/180.0 );
				covariance(4,4) = carmen_square( fabs(delta_pitch * rotation_noise) + additional_noise_odom*M_PI/180.0 );		
				covariance(5,5) = carmen_square( fabs(delta_yaw * rotation_noise) + fabs(delta_d*rotation_noise_tran )+ additional_noise_odom*M_PI/180.0 );


				Eigen::Matrix<double,6,6> information=Eigen::Matrix<double,6,6>::Identity();

				information(0,0)=1.0/covariance(0,0);
				information(1,1)=1.0/covariance(1,1);
				information(2,2)=1.0/covariance(2,2);				

				information(3,3)=1.0/covariance(3,3);
				information(4,4)=1.0/covariance(4,4);
				information(5,5)=1.0/covariance(5,5);

				odometry->setMeasurement(relative_odom);
				odometry->setInformation(information);

				if(robustKernel>0)
				{
					odometry->setRobustKernel(new g2o::RobustKernelHuber());

					if(bandwidth>0)
					{
						odometry->robustKernel()->setDelta(bandwidth);	
					}			

				}

				optimizer.addEdge(odometry);
				edgesAdded.insert(odometry);				
				std::cout<<"Dog2 odom: "<<previous_id<<" "<<it_odom->first<<std::endl;
			}

		}

	}

	
	//add odometry-based edge for the human
	for(it_odom = v_odom_human.begin(); it_odom != v_odom_human.end(); it_odom++)
	{		
		int previous_id=it_odom->first-1;

		std::map<int, int >::iterator it_current_odom;
		std::map<int, int >::iterator it_previous_odom;

		it_current_odom=m_odom_vertex_map_human.find(it_odom->first);
		it_previous_odom=m_odom_vertex_map_human.find(previous_id);


		if(it_current_odom!=m_odom_vertex_map_human.end()&&it_previous_odom!=m_odom_vertex_map_human.end())
		{

			if(addConstraints(m_vOdomConstraints_human, previous_id, it_odom->first)>0)
			{
				EdgeSE3* odometry = new EdgeSE3;
				odometry->vertices()[0] = optimizer.vertex(it_previous_odom->second);
				odometry->vertices()[1] = optimizer.vertex(it_current_odom->second);

				double previous_x=v_odom_human[previous_id].x;
				double previous_y=v_odom_human[previous_id].y;
				double previous_z=v_odom_human[previous_id].z;

				double previous_qx=v_odom_human[previous_id].qx;
				double previous_qy=v_odom_human[previous_id].qy;
				double previous_qz=v_odom_human[previous_id].qz;
				double previous_qw=v_odom_human[previous_id].qw;

				double x=it_odom->second.x;
				double y=it_odom->second.y;
				double z=it_odom->second.z;

				double qx=it_odom->second.qx;
				double qy=it_odom->second.qy;
				double qz=it_odom->second.qz;
				double qw=it_odom->second.qw;

				Eigen::Vector3d previous_trans(previous_x,previous_y,previous_z);
				Eigen::Quaterniond previous_q(previous_qw,previous_qx,previous_qy,previous_qz);//w,x,y,z

				Eigen::Vector3d trans(x,y,z);
				Eigen::Quaterniond q(qw,qx,qy,qz);//w,x,y,z

				g2o::SE3Quat previous(previous_q,previous_trans);
				g2o::SE3Quat current(q,trans);

				g2o::SE3Quat relative_odom=previous.inverse()*current;//relative transformtion T=inverse(A)*B,
				Eigen::Quaterniond qd_relative_odom= relative_odom.rotation();
				Eigen::Quaternion<float> q_relative_odom=relative_odom.rotation().cast<float>();
				Eigen::Vector3d t_relative_odom=relative_odom.translation();

				//we need to get the rpy from the q
				tf::Quaternion q_ros(qd_relative_odom.x(), qd_relative_odom.y(), qd_relative_odom.z(), qd_relative_odom.w());
				double roll, pitch, yaw;
				tf::Matrix3x3(q_ros).getRPY(roll, pitch, yaw);			

				double delta_x=t_relative_odom.x();
				double delta_y=t_relative_odom.y();
				double delta_z=t_relative_odom.z();

				double delta_roll=fabs(roll);
				double delta_pitch=fabs(pitch);
				double delta_yaw=fabs(yaw);

				double delta_d=sqrt(delta_x*delta_x+delta_y*delta_y+delta_z*delta_z);

				Eigen::Matrix<double,6,6> covariance;
				covariance.fill(0);

				//for x y z
				covariance(0,0)     = carmen_square( fabs(delta_x * tran_noise_human) +  additional_noise_odom_human );
				covariance(1,1)     = carmen_square( fabs(delta_y * tran_noise_human) +  additional_noise_odom_human );
				covariance(2,2)     = carmen_square( fabs(delta_z * tran_noise_human) +  additional_noise_odom_human );				

				//for roll pitch and yaw
				covariance(3,3) = carmen_square( fabs(delta_roll * rotation_noise_human) + additional_noise_odom_human*M_PI/180.0 );
				covariance(4,4) = carmen_square( fabs(delta_pitch * rotation_noise_human) + additional_noise_odom_human*M_PI/180.0 );
				covariance(5,5) = carmen_square( fabs(delta_yaw * rotation_noise_human) + fabs(delta_d*rotation_noise_tran_human )+ additional_noise_odom_human*M_PI/180.0 );

				Eigen::Matrix<double,6,6> information=Eigen::Matrix<double,6,6>::Identity();

				information(0,0)=1.0/covariance(0,0);
				information(1,1)=1.0/covariance(1,1);
				information(2,2)=1.0/covariance(2,2);

				information(3,3)=1.0/covariance(3,3);
				information(4,4)=1.0/covariance(4,4);
				information(5,5)=1.0/covariance(5,5);

				odometry->setMeasurement(relative_odom);
				odometry->setInformation(information);

				if(robustKernel>0)
				{
					odometry->setRobustKernel(new g2o::RobustKernelHuber());
					if(bandwidth>0)
					{
						odometry->robustKernel()->setDelta(bandwidth);	
					}
				}	

				optimizer.addEdge(odometry);
				edgesAdded.insert(odometry);

				std::cout<<"Human odom: "<<previous_id<<" "<<it_odom->first<<std::endl;

			}

		}


	}

	ROS_INFO("First vertex index in the optimiation, dog1: %d,  dog2: %d, human: %d", m_first_odom_vertex_dog1, m_first_odom_vertex_dog2, m_first_odom_vertex_human);
	if(m_first_odom_vertex_dog1>=0 && m_first_odom_vertex_dog2>=0 && m_first_odom_vertex_human>=0 && m_first_connection_between_robots==0)		
	{
		g2o::SE3Quat first_relative_dog1_human_se3=m_first_vertex_dog1_se3.inverse()*m_first_vertex_human_se3;
		
		g2o::SE3Quat first_relative_dog2_human_se3=m_first_vertex_dog2_se3.inverse()*m_first_vertex_human_se3;
		
		g2o::SE3Quat first_relative_dog1_dog2_se3=m_first_vertex_dog1_se3.inverse()*m_first_vertex_dog2_se3;

		//ROS_ERROR("relative x: %f, y: %f, z: %f", first_relative_dog1_human_se3.translation()[0], first_relative_dog1_human_se3.translation()[1],first_relative_dog1_human_se3.translation()[2]);


		//ROS_ERROR("relative x: %f, y: %f, z: %f", first_relative_dog2_human_se3.translation()[0], first_relative_dog2_human_se3.translation()[1],first_relative_dog2_human_se3.translation()[2]);

		//ROS_ERROR("relative x: %f, y: %f, z: %f", first_relative_dog1_dog2_se3.translation()[0], first_relative_dog1_dog2_se3.translation()[1],first_relative_dog1_dog2_se3.translation()[2]);


		EdgeSE3* odometry_dog1_human = new EdgeSE3;
		EdgeSE3* odometry_dog2_human = new EdgeSE3;
		EdgeSE3* odometry_dog1_dog2 = new EdgeSE3;

		odometry_dog1_human->vertices()[0] = optimizer.vertex(m_first_odom_vertex_dog1);
		odometry_dog1_human->vertices()[1] = optimizer.vertex(m_first_odom_vertex_human);
				
		odometry_dog2_human->vertices()[0] = optimizer.vertex(m_first_odom_vertex_dog2);
		odometry_dog2_human->vertices()[1] = optimizer.vertex(m_first_odom_vertex_human);
		
		odometry_dog1_dog2->vertices()[0] = optimizer.vertex(m_first_odom_vertex_dog1);
		odometry_dog1_dog2->vertices()[1] = optimizer.vertex(m_first_odom_vertex_dog2);

		Eigen::Matrix<double,6,6> information=Eigen::Matrix<double,6,6>::Identity();
		
		information(0,0)=m_initial_guess_information;
		information(1,1)=m_initial_guess_information;
		information(2,2)=m_initial_guess_information;

		information(3,3)=m_initial_guess_information*10.0;
		information(4,4)=m_initial_guess_information*10.0;
		information(5,5)=m_initial_guess_information*10.0;

		odometry_dog1_human->setMeasurement(first_relative_dog1_human_se3);
		odometry_dog1_human->setInformation(information);
		
		odometry_dog2_human->setMeasurement(first_relative_dog2_human_se3);
		odometry_dog2_human->setInformation(information);
		
		odometry_dog1_dog2->setMeasurement(first_relative_dog1_dog2_se3);
		odometry_dog1_dog2->setInformation(information);
						
				

		if(robustKernel>0)
		{
			odometry_dog1_human->setRobustKernel(new g2o::RobustKernelHuber());
			odometry_dog2_human->setRobustKernel(new g2o::RobustKernelHuber());
			odometry_dog1_dog2->setRobustKernel(new g2o::RobustKernelHuber());

			if(bandwidth>0)
			{
				odometry_dog1_human->robustKernel()->setDelta(bandwidth);
				odometry_dog2_human->robustKernel()->setDelta(bandwidth);	
				odometry_dog1_dog2->robustKernel()->setDelta(bandwidth);	

			}
		}

		optimizer.addEdge(odometry_dog1_human);
		edgesAdded.insert(odometry_dog1_human);
		
		optimizer.addEdge(odometry_dog2_human);
		edgesAdded.insert(odometry_dog2_human);
		
		optimizer.addEdge(odometry_dog1_dog2);
		edgesAdded.insert(odometry_dog1_dog2);
			

		m_first_connection_between_robots=1;
		ROS_WARN("Setting the first node connection between robot and dog");
	}


	if(optimized_peer_uwb>0)
	{
		double time_limits=1;
		//for the dog1 and human
		if(1)
		for(it_odom = v_odom_dog1.begin(); it_odom != v_odom_dog1.end(); it_odom++)
		{
			int odom_index_dog=it_odom->first;
			int odom_index_human=-1;

			double min_time_diff_human=INFINITY;			
			std::map<int, RobotPose >::iterator it_ref_odom;

			for(it_ref_odom = v_odom_human.begin(); it_ref_odom != v_odom_human.end(); it_ref_odom++)
			{
				double time_diff=fabs(it_ref_odom->second.timestamp-it_odom->second.timestamp);
				if(time_diff<=min_time_diff_human)
				{
					min_time_diff_human=time_diff;
					odom_index_human=it_ref_odom->first;
				}
			}

			std::vector<double> v_ranging_values;

			//We get the mimimum ranging to human
			std::map<std::string, RangingMeasurement> v_uwb_measure=it_odom->second.v_uwb_measure;
			std::map<std::string, RangingMeasurement>::iterator it_uwb_measure;
			for(it_uwb_measure = v_uwb_measure.begin(); it_uwb_measure != v_uwb_measure.end(); it_uwb_measure++)
			{
				std::map<std::string, double> v_range_of_tag=it_uwb_measure->second.v_ranging;
				std::map<std::string, double>::iterator it_ranging;
				for(it_ranging = v_range_of_tag.begin(); it_ranging != v_range_of_tag.end(); it_ranging++)
				{
					std::map<std::string, int >::iterator it_human;
					it_human=m_uwb_nodes_human.find(it_ranging->first);
					if(it_human!=m_uwb_nodes_human.end())
					{
						//std::cout<<it_uwb_measure->first<<" "<<it_ranging->first<<" "<<it_ranging->second<<std::endl;
						v_ranging_values.push_back(it_ranging->second);
					}

				}
				

			}


			double sum_ranging=0;
			double count_ranging=0;
			double min_ranging=INFINITY;

			for(int r=0;r<v_ranging_values.size();r++)
			{
				double uwb_ranging=v_ranging_values[r];
				sum_ranging=sum_ranging+uwb_ranging;
				count_ranging=count_ranging+1;
				if(uwb_ranging<=min_ranging)
				{
					min_ranging=uwb_ranging;
				}
			}

			if(count_ranging>0)
			{
				double mean_ranging=sum_ranging/((double)(count_ranging));

				std::map<int, int >::iterator it_odom_dog;
				std::map<int, int >::iterator it_odom_human;

				it_odom_dog=m_odom_vertex_map_dog1.find(odom_index_dog);
				it_odom_human=m_odom_vertex_map_human.find(odom_index_human);

				if(it_odom_dog!=m_odom_vertex_map_dog1.end() && it_odom_human!=m_odom_vertex_map_human.end())
				{

					if(mean_ranging<=m_max_peer_range && min_time_diff_human<time_limits)
					{		

						if(addConstraints(m_v_connections_dog1_human, odom_index_dog, odom_index_human)>0)
						{
							//this is a ranging to from  dog to human
							g2o::EdgeSE3Range* observation = new EdgeSE3Range;							
							observation->vertices()[0] = optimizer.vertex(it_odom_dog->second);
							observation->vertices()[1] = optimizer.vertex(it_odom_human->second);
							observation->setParameterId(0, 0);

							if(m_use_mean_ranging==true)
							{
								observation->setMeasurement(mean_ranging);
								
								if(mean_ranging<=m_close_range)
								{
									Eigen::Matrix<double,1,1> information_matrix=m_close_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
								else
								{
									Eigen::Matrix<double,1,1> information_matrix=m_far_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
	
							}
							else
							{
								observation->setMeasurement(min_ranging);

								if(min_ranging<=m_close_range)
								{
									Eigen::Matrix<double,1,1> information_matrix=m_close_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
								else
								{
									Eigen::Matrix<double,1,1> information_matrix=m_far_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
							}
							

							if(robustKernel>0)
							{
								observation->setRobustKernel(new g2o::RobustKernelHuber());
								if(bandwidth>0)
								{
									observation->robustKernel()->setDelta(bandwidth);	
								}
							}



							optimizer.addEdge(observation);
							m_num_connection_dog1_human=m_num_connection_dog1_human+1;
							edgesAdded.insert(observation);									
							std::cout<<"Edge (dog1->human): "<<odom_index_dog<<" "<<odom_index_human<<", ranging: "<<mean_ranging<<", min: "<<min_ranging<<", time diff: "<<min_time_diff_human<<std::endl;
						}


					}

				}

			}
		}	


		//for the human to dog1
		if(1)
		for(it_odom = v_odom_human.begin(); it_odom != v_odom_human.end(); it_odom++)
		{
			int odom_index_human=it_odom->first;
			int odom_index_dog=-1;

			double min_time_diff_dog=INFINITY;			
			std::map<int, RobotPose >::iterator it_ref_odom;
			for(it_ref_odom = v_odom_dog1.begin(); it_ref_odom != v_odom_dog1.end(); it_ref_odom++)
			{
				double time_diff=fabs(it_ref_odom->second.timestamp-it_odom->second.timestamp);
				if(time_diff<=min_time_diff_dog)
				{
					min_time_diff_dog=time_diff;
					odom_index_dog=it_ref_odom->first;
				}
			}

			std::vector<double> v_ranging_values;

			//We get the mimimum ranging to human
			std::map<std::string, RangingMeasurement> v_uwb_measure=it_odom->second.v_uwb_measure;
			std::map<std::string, RangingMeasurement>::iterator it_uwb_measure;
			for(it_uwb_measure = v_uwb_measure.begin(); it_uwb_measure != v_uwb_measure.end(); it_uwb_measure++)
			{
				std::map<std::string, double> v_range_of_tag=it_uwb_measure->second.v_ranging;
				std::map<std::string, double>::iterator it_ranging;
				for(it_ranging = v_range_of_tag.begin(); it_ranging != v_range_of_tag.end(); it_ranging++)
				{
					std::map<std::string, int >::iterator it_dog;
					it_dog=m_uwb_nodes_dog1.find(it_ranging->first);
					if(it_dog!=m_uwb_nodes_dog1.end())
					{
						//std::cout<<it_uwb_measure->first<<" "<<it_ranging->first<<" "<<it_ranging->second<<std::endl;
						v_ranging_values.push_back(it_ranging->second);
					}

				}
				

			}


			double sum_ranging=0;
			double count_ranging=0;
			double min_ranging=INFINITY;

			for(int r=0;r<v_ranging_values.size();r++)
			{
				double uwb_ranging=v_ranging_values[r];
				sum_ranging=sum_ranging+uwb_ranging;
				count_ranging=count_ranging+1;
				if(uwb_ranging<=min_ranging)
				{
					min_ranging=uwb_ranging;
				}
			}

			if(count_ranging>0)
			{
				double mean_ranging=sum_ranging/((double)(count_ranging));

				std::map<int, int >::iterator it_odom_dog;
				std::map<int, int >::iterator it_odom_human;


				it_odom_human=m_odom_vertex_map_human.find(odom_index_human);
				it_odom_dog=m_odom_vertex_map_dog1.find(odom_index_dog);


				if(it_odom_dog!=m_odom_vertex_map_dog1.end() && it_odom_human!=m_odom_vertex_map_human.end())
				{					
					if(mean_ranging<=m_max_peer_range && min_time_diff_dog<time_limits)
					{		

						if(addConstraints(m_v_connections_dog1_human, odom_index_dog, odom_index_human)>0)
						{
							//this is a ranging to from human to dog
							g2o::EdgeSE3Range* observation = new EdgeSE3Range;							
							observation->vertices()[0] = optimizer.vertex(it_odom_dog->second);
							observation->vertices()[1] = optimizer.vertex(it_odom_human->second);
							observation->setParameterId(0, 0);

							if(m_use_mean_ranging==true)
							{
								observation->setMeasurement(mean_ranging);
								
								if(mean_ranging<=m_close_range)
								{
									Eigen::Matrix<double,1,1> information_matrix=m_close_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
								else
								{
									Eigen::Matrix<double,1,1> information_matrix=m_far_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
	
							}
							else
							{
								observation->setMeasurement(min_ranging);

								if(min_ranging<=m_close_range)
								{
									Eigen::Matrix<double,1,1> information_matrix=m_close_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
								else
								{
									Eigen::Matrix<double,1,1> information_matrix=m_far_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
							}


							if(robustKernel>0)
							{
								observation->setRobustKernel(new g2o::RobustKernelHuber());
								if(bandwidth>0)
								{
									observation->robustKernel()->setDelta(bandwidth);	
								}
							}

							optimizer.addEdge(observation);
							m_num_connection_human_dog1=m_num_connection_human_dog1+1;
							edgesAdded.insert(observation);									
							std::cout<<"Edge (human->dog1): "<<odom_index_human<<" "<<odom_index_dog<<", ranging: "<<mean_ranging<<", min range: "<<min_ranging<<", time diff: "<<min_time_diff_dog<<std::endl;
						}
					}
				}

			}

		}	


		//for the dog2 and human
		if(1)
		for(it_odom = v_odom_dog2.begin(); it_odom != v_odom_dog2.end(); it_odom++)
		{
			int odom_index_dog=it_odom->first;
			int odom_index_human=-1;

			double min_time_diff_human=INFINITY;			
			std::map<int, RobotPose >::iterator it_ref_odom;

			for(it_ref_odom = v_odom_human.begin(); it_ref_odom != v_odom_human.end(); it_ref_odom++)
			{
				double time_diff=fabs(it_ref_odom->second.timestamp-it_odom->second.timestamp);
				if(time_diff<=min_time_diff_human)
				{
					min_time_diff_human=time_diff;
					odom_index_human=it_ref_odom->first;
				}
			}

			std::vector<double> v_ranging_values;

			//We get the mimimum ranging to human
			std::map<std::string, RangingMeasurement> v_uwb_measure=it_odom->second.v_uwb_measure;
			std::map<std::string, RangingMeasurement>::iterator it_uwb_measure;
			for(it_uwb_measure = v_uwb_measure.begin(); it_uwb_measure != v_uwb_measure.end(); it_uwb_measure++)
			{
				std::map<std::string, double> v_range_of_tag=it_uwb_measure->second.v_ranging;
				std::map<std::string, double>::iterator it_ranging;
				for(it_ranging = v_range_of_tag.begin(); it_ranging != v_range_of_tag.end(); it_ranging++)
				{
					std::map<std::string, int >::iterator it_human;
					it_human=m_uwb_nodes_human.find(it_ranging->first);
					if(it_human!=m_uwb_nodes_human.end())
					{
						//std::cout<<it_uwb_measure->first<<" "<<it_ranging->first<<" "<<it_ranging->second<<std::endl;
						v_ranging_values.push_back(it_ranging->second);
					}

				}
				

			}


			double sum_ranging=0;
			double count_ranging=0;
			double min_ranging=INFINITY;

			for(int r=0;r<v_ranging_values.size();r++)
			{
				double uwb_ranging=v_ranging_values[r];
				sum_ranging=sum_ranging+uwb_ranging;
				count_ranging=count_ranging+1;
				if(uwb_ranging<=min_ranging)
				{
					min_ranging=uwb_ranging;
				}
			}

			if(count_ranging>0)
			{
				double mean_ranging=sum_ranging/((double)(count_ranging));

				std::map<int, int >::iterator it_odom_dog;
				std::map<int, int >::iterator it_odom_human;

				it_odom_dog=m_odom_vertex_map_dog2.find(odom_index_dog);
				it_odom_human=m_odom_vertex_map_human.find(odom_index_human);

				if(it_odom_dog!=m_odom_vertex_map_dog2.end() && it_odom_human!=m_odom_vertex_map_human.end())
				{

					if(mean_ranging<=m_max_peer_range && min_time_diff_human<time_limits)
					{		


						if(addConstraints(m_v_connections_dog2_human, odom_index_dog, odom_index_human)>0)
						{
							//this is a ranging to from  dog to human
							g2o::EdgeSE3Range* observation = new EdgeSE3Range;							
							observation->vertices()[0] = optimizer.vertex(it_odom_dog->second);
							observation->vertices()[1] = optimizer.vertex(it_odom_human->second);
							observation->setParameterId(0, 0);
							

							if(m_use_mean_ranging==true)
							{
								observation->setMeasurement(mean_ranging);
								
								if(mean_ranging<=m_close_range)
								{
									Eigen::Matrix<double,1,1> information_matrix=m_close_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
								else
								{
									Eigen::Matrix<double,1,1> information_matrix=m_far_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
	
							}
							else
							{
								observation->setMeasurement(min_ranging);

								if(min_ranging<=m_close_range)
								{
									Eigen::Matrix<double,1,1> information_matrix=m_close_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
								else
								{
									Eigen::Matrix<double,1,1> information_matrix=m_far_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
							}

							if(robustKernel>0)
							{
								observation->setRobustKernel(new g2o::RobustKernelHuber());
								if(bandwidth>0)
								{
									observation->robustKernel()->setDelta(bandwidth);	
								}
							}



							optimizer.addEdge(observation);
							m_num_connection_dog2_human=m_num_connection_dog2_human+1;
							edgesAdded.insert(observation);									
							std::cout<<"Edge (dog2->human): "<<odom_index_dog<<" "<<odom_index_human<<", ranging: "<<mean_ranging<<", min range: "<<min_ranging<<", time diff: "<<min_time_diff_human<<std::endl;
						}


					}

				}

			}
		}	

		//for the human to dog2
		if(1)
		for(it_odom = v_odom_human.begin(); it_odom != v_odom_human.end(); it_odom++)
		{
			int odom_index_human=it_odom->first;
			int odom_index_dog=-1;

			double min_time_diff_dog=INFINITY;			
			std::map<int, RobotPose >::iterator it_ref_odom;
			for(it_ref_odom = v_odom_dog2.begin(); it_ref_odom != v_odom_dog2.end(); it_ref_odom++)
			{
				double time_diff=fabs(it_ref_odom->second.timestamp-it_odom->second.timestamp);
				if(time_diff<=min_time_diff_dog)
				{
					min_time_diff_dog=time_diff;
					odom_index_dog=it_ref_odom->first;
				}
			}


			std::vector<double> v_ranging_values;

			//We get the mimimum ranging to human
			std::map<std::string, RangingMeasurement> v_uwb_measure=it_odom->second.v_uwb_measure;
			std::map<std::string, RangingMeasurement>::iterator it_uwb_measure;
			for(it_uwb_measure = v_uwb_measure.begin(); it_uwb_measure != v_uwb_measure.end(); it_uwb_measure++)
			{
				std::map<std::string, double> v_range_of_tag=it_uwb_measure->second.v_ranging;
				std::map<std::string, double>::iterator it_ranging;
				for(it_ranging = v_range_of_tag.begin(); it_ranging != v_range_of_tag.end(); it_ranging++)
				{
					std::map<std::string, int >::iterator it_dog;
					it_dog=m_uwb_nodes_dog2.find(it_ranging->first);
					if(it_dog!=m_uwb_nodes_dog2.end())
					{
						//std::cout<<it_uwb_measure->first<<" "<<it_ranging->first<<" "<<it_ranging->second<<std::endl;
						v_ranging_values.push_back(it_ranging->second);
					}

				}
				

			}


			double sum_ranging=0;
			double count_ranging=0;
			double min_ranging=INFINITY;

			for(int r=0;r<v_ranging_values.size();r++)
			{
				double uwb_ranging=v_ranging_values[r];
				sum_ranging=sum_ranging+uwb_ranging;
				count_ranging=count_ranging+1;
				if(uwb_ranging<=min_ranging)
				{
					min_ranging=uwb_ranging;
				}
			}

			if(count_ranging>0)
			{
				double mean_ranging=sum_ranging/((double)(count_ranging));

				std::map<int, int >::iterator it_odom_dog;
				std::map<int, int >::iterator it_odom_human;


				it_odom_human=m_odom_vertex_map_human.find(odom_index_human);
				it_odom_dog=m_odom_vertex_map_dog2.find(odom_index_dog);


				if(it_odom_dog!=m_odom_vertex_map_dog2.end() && it_odom_human!=m_odom_vertex_map_human.end())
				{					
					if(mean_ranging<=m_max_peer_range &&min_time_diff_dog<time_limits)
					{			

						if(addConstraints(m_v_connections_dog2_human, odom_index_dog, odom_index_human)>0)
						{
							//this is a ranging to from human to dog
							g2o::EdgeSE3Range* observation = new EdgeSE3Range;							
							observation->vertices()[0] = optimizer.vertex(it_odom_dog->second);
							observation->vertices()[1] = optimizer.vertex(it_odom_human->second);
							observation->setParameterId(0, 0);


							if(m_use_mean_ranging==true)
							{
								observation->setMeasurement(mean_ranging);
								
								if(mean_ranging<=m_close_range)
								{
									Eigen::Matrix<double,1,1> information_matrix=m_close_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
								else
								{
									Eigen::Matrix<double,1,1> information_matrix=m_far_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
	
							}
							else
							{
								observation->setMeasurement(min_ranging);

								if(min_ranging<=m_close_range)
								{
									Eigen::Matrix<double,1,1> information_matrix=m_close_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
								else
								{
									Eigen::Matrix<double,1,1> information_matrix=m_far_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
							}

							if(robustKernel>0)
							{
								observation->setRobustKernel(new g2o::RobustKernelHuber());
								if(bandwidth>0)
								{
									observation->robustKernel()->setDelta(bandwidth);	
								}
							}


							optimizer.addEdge(observation);
							m_num_connection_human_dog2=m_num_connection_human_dog2+1;
							edgesAdded.insert(observation);									
							std::cout<<"Edge (human->dog2): "<<odom_index_human<<" "<<odom_index_dog<<", ranging: "<<mean_ranging<<", min range: "<<min_ranging<<", time diff: "<<min_time_diff_dog<<std::endl;
						}
					}
				}

			}

		}


		//for the dog1 to dog2
		if(1)
		for(it_odom = v_odom_dog1.begin(); it_odom != v_odom_dog1.end(); it_odom++)
		{
			int odom_index_dog1=it_odom->first;
			int odom_index_dog=-1;

			double min_time_diff_dog=INFINITY;			
			std::map<int, RobotPose >::iterator it_ref_odom;
			for(it_ref_odom = v_odom_dog2.begin(); it_ref_odom != v_odom_dog2.end(); it_ref_odom++)
			{
				double time_diff=fabs(it_ref_odom->second.timestamp-it_odom->second.timestamp);
				if(time_diff<=min_time_diff_dog)
				{
					min_time_diff_dog=time_diff;
					odom_index_dog=it_ref_odom->first;
				}
			}

			std::vector<double> v_ranging_values;

			//We get the mimimum ranging to human
			std::map<std::string, RangingMeasurement> v_uwb_measure=it_odom->second.v_uwb_measure;
			std::map<std::string, RangingMeasurement>::iterator it_uwb_measure;
			for(it_uwb_measure = v_uwb_measure.begin(); it_uwb_measure != v_uwb_measure.end(); it_uwb_measure++)
			{
				std::map<std::string, double> v_range_of_tag=it_uwb_measure->second.v_ranging;
				std::map<std::string, double>::iterator it_ranging;
				for(it_ranging = v_range_of_tag.begin(); it_ranging != v_range_of_tag.end(); it_ranging++)
				{
					std::map<std::string, int >::iterator it_dog;
					it_dog=m_uwb_nodes_dog2.find(it_ranging->first);
					if(it_dog!=m_uwb_nodes_dog2.end())
					{
						//std::cout<<it_uwb_measure->first<<" "<<it_ranging->first<<" "<<it_ranging->second<<std::endl;
						v_ranging_values.push_back(it_ranging->second);
					}

				}
				

			}


			double sum_ranging=0;
			double count_ranging=0;
			double min_ranging=INFINITY;

			for(int r=0;r<v_ranging_values.size();r++)
			{
				double uwb_ranging=v_ranging_values[r];
				sum_ranging=sum_ranging+uwb_ranging;
				count_ranging=count_ranging+1;
				if(uwb_ranging<=min_ranging)
				{
					min_ranging=uwb_ranging;
				}
			}

			if(count_ranging>0)
			{
				double mean_ranging=sum_ranging/((double)(count_ranging));

				std::map<int, int >::iterator it_odom_dog1;
				std::map<int, int >::iterator it_odom_dog;


				it_odom_dog1=m_odom_vertex_map_dog1.find(odom_index_dog1);
				it_odom_dog=m_odom_vertex_map_dog2.find(odom_index_dog);


				if(it_odom_dog!=m_odom_vertex_map_dog2.end() && it_odom_dog1!=m_odom_vertex_map_dog1.end())
				{					
					if(mean_ranging<=m_max_peer_range && min_time_diff_dog<time_limits)
					{			

						if(addConstraints(m_v_connections_dog1_dog2, odom_index_dog1, odom_index_dog)>0)
						{
							//this is a ranging to from human to dog
							g2o::EdgeSE3Range* observation = new EdgeSE3Range;							
							observation->vertices()[0] = optimizer.vertex(it_odom_dog1->second);
							observation->vertices()[1] = optimizer.vertex(it_odom_dog->second);
							observation->setParameterId(0, 0);

							if(m_use_mean_ranging==true)
							{
								observation->setMeasurement(mean_ranging);
								
								if(mean_ranging<=m_close_range)
								{
									Eigen::Matrix<double,1,1> information_matrix=m_close_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
								else
								{
									Eigen::Matrix<double,1,1> information_matrix=m_far_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
	
							}
							else
							{
								observation->setMeasurement(min_ranging);

								if(min_ranging<=m_close_range)
								{
									Eigen::Matrix<double,1,1> information_matrix=m_close_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
								else
								{
									Eigen::Matrix<double,1,1> information_matrix=m_far_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
							}
							
							if(robustKernel>0)
							{
								observation->setRobustKernel(new g2o::RobustKernelHuber());
								if(bandwidth>0)
								{
									observation->robustKernel()->setDelta(bandwidth);	
								}
							}


							optimizer.addEdge(observation);
							m_num_connection_dog1_dog2=m_num_connection_dog1_dog2+1;
							edgesAdded.insert(observation);									
							std::cout<<"Edge (dog1->dog2): "<<odom_index_dog1<<" "<<odom_index_dog<<", ranging: "<<mean_ranging<<", min range: "<<min_ranging<<", time diff: "<<min_time_diff_dog<<std::endl;
						}
					}
				}

			}

		}	


		//for the dog2 to dog1
		if(1)
		for(it_odom = v_odom_dog2.begin(); it_odom != v_odom_dog2.end(); it_odom++)
		{
			int odom_index_dog2=it_odom->first;
			int odom_index_dog=-1;

			double min_time_diff_dog=INFINITY;			
			std::map<int, RobotPose >::iterator it_ref_odom;
			for(it_ref_odom = v_odom_dog1.begin(); it_ref_odom != v_odom_dog1.end(); it_ref_odom++)
			{
				double time_diff=fabs(it_ref_odom->second.timestamp-it_odom->second.timestamp);
				if(time_diff<=min_time_diff_dog)
				{
					min_time_diff_dog=time_diff;
					odom_index_dog=it_ref_odom->first;
				}
			}



			std::vector<double> v_ranging_values;

			//We get the mimimum ranging to human
			std::map<std::string, RangingMeasurement> v_uwb_measure=it_odom->second.v_uwb_measure;
			std::map<std::string, RangingMeasurement>::iterator it_uwb_measure;
			for(it_uwb_measure = v_uwb_measure.begin(); it_uwb_measure != v_uwb_measure.end(); it_uwb_measure++)
			{
				std::map<std::string, double> v_range_of_tag=it_uwb_measure->second.v_ranging;
				std::map<std::string, double>::iterator it_ranging;
				for(it_ranging = v_range_of_tag.begin(); it_ranging != v_range_of_tag.end(); it_ranging++)
				{
					std::map<std::string, int >::iterator it_dog;
					it_dog=m_uwb_nodes_dog1.find(it_ranging->first);
					if(it_dog!=m_uwb_nodes_dog1.end())
					{
						//std::cout<<it_uwb_measure->first<<" "<<it_ranging->first<<" "<<it_ranging->second<<std::endl;
						v_ranging_values.push_back(it_ranging->second);
					}

				}
				

			}


			double sum_ranging=0;
			double count_ranging=0;
			double min_ranging=INFINITY;

			for(int r=0;r<v_ranging_values.size();r++)
			{
				double uwb_ranging=v_ranging_values[r];
				sum_ranging=sum_ranging+uwb_ranging;
				count_ranging=count_ranging+1;
				if(uwb_ranging<=min_ranging)
				{
					min_ranging=uwb_ranging;
				}
			}
			if(count_ranging>0)
			{
				double mean_ranging=sum_ranging/((double)(count_ranging));

				std::map<int, int >::iterator it_odom_dog2;
				std::map<int, int >::iterator it_odom_dog;


				it_odom_dog=m_odom_vertex_map_dog1.find(odom_index_dog);
				it_odom_dog2=m_odom_vertex_map_dog2.find(odom_index_dog2);


				if(it_odom_dog!=m_odom_vertex_map_dog1.end() && it_odom_dog2!=m_odom_vertex_map_dog2.end())
				{					
					if(mean_ranging<=m_max_peer_range && min_time_diff_dog<time_limits)
					{
						if(addConstraints(m_v_connections_dog1_dog2, odom_index_dog, odom_index_dog2)>0)
						{
							//this is a ranging to from human to dog
							g2o::EdgeSE3Range* observation = new EdgeSE3Range;							
							observation->vertices()[0] = optimizer.vertex(it_odom_dog->second);
							observation->vertices()[1] = optimizer.vertex(it_odom_dog2->second);
							observation->setParameterId(0, 0);

							if(m_use_mean_ranging==true)
							{
								observation->setMeasurement(mean_ranging);
								
								if(mean_ranging<=m_close_range)
								{
									Eigen::Matrix<double,1,1> information_matrix=m_close_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
								else
								{
									Eigen::Matrix<double,1,1> information_matrix=m_far_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
	
							}
							else
							{
								observation->setMeasurement(min_ranging);

								if(min_ranging<=m_close_range)
								{
									Eigen::Matrix<double,1,1> information_matrix=m_close_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
								else
								{
									Eigen::Matrix<double,1,1> information_matrix=m_far_uwb_information*Eigen::Matrix<double,1,1>::Identity();
									observation->setInformation(information_matrix);
								}
							}							

							if(robustKernel>0)
							{
								observation->setRobustKernel(new g2o::RobustKernelHuber());
								if(bandwidth>0)
								{
									observation->robustKernel()->setDelta(bandwidth);	
								}
							}


							optimizer.addEdge(observation);
							m_num_connection_dog2_dog1=m_num_connection_dog2_dog1+1;
							edgesAdded.insert(observation);									
							std::cout<<"Edge (dog2->dog1): "<<odom_index_dog2<<" "<<odom_index_dog<<", ranging: "<<mean_ranging<<", min range: "<<min_ranging<<", time diff: "<<min_time_diff_dog<<std::endl;
						}
					}
				}

			}

		}	



	}


	if(m_first_node_fixed==0)
	{
		/*
		if(m_first_odom_vertex_dog1>=0 && m_first_odom_vertex_dog2>=0 && m_first_odom_vertex_human>=0)
		{
			VertexSE3* firstPoseHuman = dynamic_cast<VertexSE3*>(optimizer.vertex(m_first_odom_vertex_human));
			VertexSE3* firstPoseDog1 = dynamic_cast<VertexSE3*>(optimizer.vertex(m_first_odom_vertex_dog1));
			VertexSE3* firstPoseDog2 = dynamic_cast<VertexSE3*>(optimizer.vertex(m_first_odom_vertex_dog2));

			firstPoseHuman->setFixed(true);
			firstPoseDog1->setFixed(true);
			firstPoseDog2->setFixed(true);
			m_first_node_fixed=1;
			ROS_INFO("First node fixed for all");


		}*/
		
		if(m_first_odom_vertex_dog1>=0)
		{
			VertexSE3* firstPose = dynamic_cast<VertexSE3*>(optimizer.vertex(m_first_odom_vertex_dog1));
			firstPose->setFixed(true);
			m_first_node_fixed=1;
			ROS_INFO("First node fixed for dog");
		}
		else
		{
			if(m_first_odom_vertex_dog2>=0)
			{
				VertexSE3* firstPose = dynamic_cast<VertexSE3*>(optimizer.vertex(m_first_odom_vertex_dog2));
				firstPose->setFixed(true);
				m_first_node_fixed=1;
				ROS_INFO("First node fixed for dog");
				
			}
			else
			{
				if(m_first_odom_vertex_human>=0)
				{
					VertexSE3* firstPose = dynamic_cast<VertexSE3*>(optimizer.vertex(m_first_odom_vertex_human));
					firstPose->setFixed(true);
					m_first_node_fixed=1;
					ROS_INFO("First node fixed for human");
				}
				
			}

		}
		

	}


	ROS_INFO("Connections: dog1->human: %d, human->dog1: %d, dog2->human: %d, human->dog2: %d, dog1->dog2: %d, dog2->dog1: %d, fixed human: %d, fixed dog1: %d, fixed dog2: %d", 
			m_num_connection_dog1_human, m_num_connection_human_dog1,m_num_connection_dog2_human, m_num_connection_human_dog2, m_num_connection_dog1_dog2, m_num_connection_dog2_dog1, m_num_connection_fixed_human, m_num_connection_fixed_dog1, m_num_connection_fixed_dog2);

	//save the raw pose before optimization
	OptimizableGraph::VertexIDMap graph_vertices=optimizer.vertices();
	OptimizableGraph::VertexIDMap::iterator it_map;

	//raw robot pose
	std::map < int, RobotPose > v_raw_pose_dog1;
	std::map < int, RobotPose > v_raw_pose_dog2;
	std::map < int, RobotPose > v_raw_pose_human;


	//optimized pose by UWB
	std::map < int, RobotPose > v_uwb_optimized_pose_dog1;
	std::map < int, RobotPose > v_uwb_optimized_pose_dog2;
	std::map < int, RobotPose > v_uwb_optimized_pose_human;
	std::map < std::string, carmen_point_t > v_uwb_optimized_landmark_pose;


	for(it_map = graph_vertices.begin(); it_map != graph_vertices.end(); ++it_map)
	{
		std::map<int, int >::iterator it_odom_dog1;
		it_odom_dog1=m_vertex_odom_map_dog1.find(it_map->first);

		if(it_odom_dog1!=m_vertex_odom_map_dog1.end())
		{
			VertexSE3* vertex = dynamic_cast<VertexSE3*>(it_map->second);
			g2o::SE3Quat pose_se3quat=vertex->estimateAsSE3Quat();
			Eigen::Quaterniond q=pose_se3quat.rotation();

			//we need to get the rpy from the q
			tf::Quaternion q_ros(q.x(), q.y(), q.z(), q.w());
			double roll, pitch, yaw;
			tf::Matrix3x3(q_ros).getRPY(roll, pitch, yaw);			


			RobotPose pose;
			pose.x=pose_se3quat.translation()[0];
			pose.y=pose_se3quat.translation()[1];
			pose.z=pose_se3quat.translation()[2];

			pose.roll=roll;
			pose.pitch=pitch;
			pose.yaw=yaw;

			pose.qx=q.x();
			pose.qy=q.y();
			pose.qz=q.z();
			pose.qw=q.w();

			v_raw_pose_dog1[it_odom_dog1->second]=pose;

		}
		else
		{
			std::map<int, int >::iterator it_odom_dog2;
			it_odom_dog2=m_vertex_odom_map_dog2.find(it_map->first);
			
			if(it_odom_dog2!=m_vertex_odom_map_dog2.end())
			{
				VertexSE3* vertex = dynamic_cast<VertexSE3*>(it_map->second);
				g2o::SE3Quat pose_se3quat=vertex->estimateAsSE3Quat();
				Eigen::Quaterniond q=pose_se3quat.rotation();

				//we need to get the rpy from the q
				tf::Quaternion q_ros(q.x(), q.y(), q.z(), q.w());
				double roll, pitch, yaw;
				tf::Matrix3x3(q_ros).getRPY(roll, pitch, yaw);			


				RobotPose pose;
				pose.x=pose_se3quat.translation()[0];
				pose.y=pose_se3quat.translation()[1];
				pose.z=pose_se3quat.translation()[2];

				pose.roll=roll;
				pose.pitch=pitch;
				pose.yaw=yaw;

				pose.qx=q.x();
				pose.qy=q.y();
				pose.qz=q.z();
				pose.qw=q.w();

				v_raw_pose_dog2[it_odom_dog2->second]=pose;

				
			}
			else
			{
				std::map<int, int >::iterator it_odom_human;
				it_odom_human=m_vertex_odom_map_human.find(it_map->first);


				if(it_odom_human!=m_vertex_odom_map_human.end())
				{
					VertexSE3* vertex = dynamic_cast<VertexSE3*>(it_map->second);
					g2o::SE3Quat pose_se3quat=vertex->estimateAsSE3Quat();
					Eigen::Quaterniond q=pose_se3quat.rotation();

					//we need to get the rpy from the q
					tf::Quaternion q_ros(q.x(), q.y(), q.z(), q.w());
					double roll, pitch, yaw;
					tf::Matrix3x3(q_ros).getRPY(roll, pitch, yaw);			


					RobotPose pose;
					pose.x=pose_se3quat.translation()[0];
					pose.y=pose_se3quat.translation()[1];
					pose.z=pose_se3quat.translation()[2];

					pose.roll=roll;
					pose.pitch=pitch;
					pose.yaw=yaw;

					pose.qx=q.x();
					pose.qy=q.y();
					pose.qz=q.z();
					pose.qw=q.w();

					v_raw_pose_human[it_odom_human->second]=pose;

				}	
				
			}

	
		}
	}


	if(m_first_node_fixed==1)
	{
		if(m_optimizer_first_time==1)
		{
			optimizer.initializeOptimization();	
			optimizer.optimize(m_max_iterations);
			m_optimizer_first_time=0;
		}
		else
		{
			ROS_INFO("verticesAdded: %d, edgesAdded: %d", verticesAdded.size(), edgesAdded.size());

			if(verticesAdded.size()>0 || edgesAdded.size()>0)
			{
				optimizer.initializeOptimization();	
				optimizer.optimize(m_incremental_iterations);

			}

		}

	}		


	//get the optimized pose
	graph_vertices=optimizer.vertices();

	//we add the locker here
	m_pose_update_mutex.lock();

	for(it_map = graph_vertices.begin(); it_map != graph_vertices.end(); ++it_map)
	{
		std::map<int, int >::iterator it_odom_dog1;
		it_odom_dog1=m_vertex_odom_map_dog1.find(it_map->first);

		if(it_odom_dog1!=m_vertex_odom_map_dog1.end())
		{
			VertexSE3* vertex = dynamic_cast<VertexSE3*>(it_map->second);
			g2o::SE3Quat pose_se3quat=vertex->estimateAsSE3Quat();
			Eigen::Quaterniond q=pose_se3quat.rotation();

			//we need to get the rpy from the q
			tf::Quaternion q_ros(q.x(), q.y(), q.z(), q.w());
			double roll, pitch, yaw;
			tf::Matrix3x3(q_ros).getRPY(roll, pitch, yaw);			


			RobotPose pose;
			pose.x=pose_se3quat.translation()[0];
			pose.y=pose_se3quat.translation()[1];
			pose.z=pose_se3quat.translation()[2];

			pose.roll=roll;
			pose.pitch=pitch;
			pose.yaw=yaw;

			pose.qx=q.x();
			pose.qy=q.y();
			pose.qz=q.z();
			pose.qw=q.w();

			v_uwb_optimized_pose_dog1[it_odom_dog1->second]=pose;

			if(it_odom_dog1->second>=m_index_updated_pose_dog1)
			{
				m_index_updated_pose_dog1=it_odom_dog1->second;

				m_updated_pose_dog1.x=pose_se3quat.translation()[0];
				m_updated_pose_dog1.y=pose_se3quat.translation()[1];
				m_updated_pose_dog1.z=pose_se3quat.translation()[2];

				m_updated_pose_dog1.roll=roll;
				m_updated_pose_dog1.pitch=pitch;
				m_updated_pose_dog1.yaw=yaw;

				m_updated_pose_dog1.qx=q.x();
				m_updated_pose_dog1.qy=q.y();
				m_updated_pose_dog1.qz=q.z();
				m_updated_pose_dog1.qw=q.w();				
			}

		}
		else
		{
			std::map<int, int >::iterator it_odom_dog2;
			it_odom_dog2=m_vertex_odom_map_dog2.find(it_map->first);
			
			if(it_odom_dog2!=m_vertex_odom_map_dog2.end())
			{
				VertexSE3* vertex = dynamic_cast<VertexSE3*>(it_map->second);
				g2o::SE3Quat pose_se3quat=vertex->estimateAsSE3Quat();
				Eigen::Quaterniond q=pose_se3quat.rotation();

				//we need to get the rpy from the q
				tf::Quaternion q_ros(q.x(), q.y(), q.z(), q.w());
				double roll, pitch, yaw;
				tf::Matrix3x3(q_ros).getRPY(roll, pitch, yaw);			


				RobotPose pose;
				pose.x=pose_se3quat.translation()[0];
				pose.y=pose_se3quat.translation()[1];
				pose.z=pose_se3quat.translation()[2];

				pose.roll=roll;
				pose.pitch=pitch;
				pose.yaw=yaw;

				pose.qx=q.x();
				pose.qy=q.y();
				pose.qz=q.z();
				pose.qw=q.w();

				v_uwb_optimized_pose_dog2[it_odom_dog2->second]=pose;

				if(it_odom_dog2->second>=m_index_updated_pose_dog2)
				{
					m_index_updated_pose_dog2=it_odom_dog2->second;

					m_updated_pose_dog2.x=pose_se3quat.translation()[0];
					m_updated_pose_dog2.y=pose_se3quat.translation()[1];
					m_updated_pose_dog2.z=pose_se3quat.translation()[2];

					m_updated_pose_dog2.roll=roll;
					m_updated_pose_dog2.pitch=pitch;
					m_updated_pose_dog2.yaw=yaw;

					m_updated_pose_dog2.qx=q.x();
					m_updated_pose_dog2.qy=q.y();
					m_updated_pose_dog2.qz=q.z();
					m_updated_pose_dog2.qw=q.w();				
				}
				
			}
			else
			{
				std::map<int, int >::iterator it_odom_human;
				it_odom_human=m_vertex_odom_map_human.find(it_map->first);


				if(it_odom_human!=m_vertex_odom_map_human.end())
				{
					VertexSE3* vertex = dynamic_cast<VertexSE3*>(it_map->second);
					g2o::SE3Quat pose_se3quat=vertex->estimateAsSE3Quat();
					Eigen::Quaterniond q=pose_se3quat.rotation();

					//we need to get the rpy from the q
					tf::Quaternion q_ros(q.x(), q.y(), q.z(), q.w());
					double roll, pitch, yaw;
					tf::Matrix3x3(q_ros).getRPY(roll, pitch, yaw);			


					RobotPose pose;
					pose.x=pose_se3quat.translation()[0];
					pose.y=pose_se3quat.translation()[1];
					pose.z=pose_se3quat.translation()[2];

					pose.roll=roll;
					pose.pitch=pitch;
					pose.yaw=yaw;

					pose.qx=q.x();
					pose.qy=q.y();
					pose.qz=q.z();
					pose.qw=q.w();


					v_uwb_optimized_pose_human[it_odom_human->second]=pose;

					if(it_odom_human->second>=m_index_updated_pose_human)
					{
						m_index_updated_pose_human=it_odom_human->second;

						m_updated_pose_human.x=pose_se3quat.translation()[0];
						m_updated_pose_human.y=pose_se3quat.translation()[1];
						m_updated_pose_human.z=pose_se3quat.translation()[2];

						m_updated_pose_human.roll=roll;
						m_updated_pose_human.pitch=pitch;
						m_updated_pose_human.yaw=yaw;

						m_updated_pose_human.qx=q.x();
						m_updated_pose_human.qy=q.y();
						m_updated_pose_human.qz=q.z();
						m_updated_pose_human.qw=q.w();				
					}



				}
				else
				{
					std::map<int, std::string >::iterator it_uwb_node;
					it_uwb_node=m_vertex_uwb_map.find(it_map->first);
					if(it_uwb_node!=m_vertex_uwb_map.end())
					{
						VertexSE3* vertex = dynamic_cast<VertexSE3*>(it_map->second);
						g2o::SE3Quat pose_se3quat=vertex->estimateAsSE3Quat();
						carmen_point_t pose;
						pose.x=pose_se3quat.translation()[0];
						pose.y=pose_se3quat.translation()[1];	
						if(optimized_external_uwb>0)
						{
							v_uwb_optimized_landmark_pose[it_uwb_node->second]=pose;

							std::cout<<"External UWB optimized position: "<<it_uwb_node->second<<" "<<pose_se3quat.translation()[0]<<" "<<pose_se3quat.translation()[1]<<" "<<pose_se3quat.translation()[2]<<std::endl;
						}
					}
				}
				
			}

			


		}

	}
	m_pose_update_mutex.unlock();


	if(m_enable_gnuplot)
	{
		plotOptimizedTrack(v_uwb_optimized_pose_dog1, v_uwb_optimized_pose_dog2, v_uwb_optimized_pose_human, v_uwb_optimized_landmark_pose, m_v_connections_dog1_human, m_v_connections_dog2_human,m_v_connections_dog1_dog2, m_v_uwb_connections_dog1, m_v_uwb_connections_dog2, m_v_uwb_connections_human);	
	}	
}


// --------------------------------------------------------------------------
void plotPublishedPose(std::vector < RobotPose > v_raw_pose_dog, 
		std::vector < RobotPose > v_raw_pose_human)
// --------------------------------------------------------------------------
{
	std::string cmd;//( "set size ratio 1\n");
	cmd+="set grid\n";
	cmd+="set xlabel 'x(m)'\n";
	cmd+="set ylabel 'y(m)'\n";
	cmd+="set xrange ["+toString(m_plot_min_x)+":"+toString(m_plot_max_x)+"]\n";
	cmd+="set yrange ["+toString(m_plot_min_y)+":"+toString(m_plot_max_y)+"]\n";
	cmd+="set title 'Published pose'\n";
	cmd+="set size ratio -1\n";

	cmd+="plot '-' u 1:2 w l pt 1 lw 3 lt 1 lc rgb '#000000' ti 'Dog', '-' u 1:2 w l pt 2 lw 3 lt 2 lc rgb '#0000FF' ti 'Human',\n";

	for(int i=0; i< v_raw_pose_dog.size(); i++)	
	{
		cmd += toString( v_raw_pose_dog[i].x ) + ' ' + toString( v_raw_pose_dog[i].y) + ' ' + toString(0) + '\n';
	}

	cmd += "e\n";

	for(int i=0; i< v_raw_pose_human.size(); i++)	
	{
		cmd += toString( v_raw_pose_human[i].x ) + ' ' + toString( v_raw_pose_human[i].y) + ' ' + toString(0) + '\n';
	}

	cmd += "e\n";

	m_plot_published_pose->commandStr( cmd);
}


//--------------------------------------------------------------------------
void sendPose(ros::Publisher pub, g2o::SE3Quat pose_se3, double timestamp, std::string str_frame_id)
//--------------------------------------------------------------------------
{
	geometry_msgs::PoseWithCovarianceStamped pose_with_timestamp;
	Eigen::Quaterniond pose_q=pose_se3.rotation();

	//position in x,y, and z
	pose_with_timestamp.pose.pose.position.x = pose_se3.translation()[0];
	pose_with_timestamp.pose.pose.position.y = pose_se3.translation()[1];
	pose_with_timestamp.pose.pose.position.z = pose_se3.translation()[2];

    //rotation
	pose_with_timestamp.pose.pose.orientation.x = pose_q.x();
	pose_with_timestamp.pose.pose.orientation.y = pose_q.y();
	pose_with_timestamp.pose.pose.orientation.z = pose_q.z();
	pose_with_timestamp.pose.pose.orientation.w = pose_q.w();

	pose_with_timestamp.header.stamp=ros::Time(timestamp);
	//pose_with_timestamp.header.stamp =  ros::Time::now();

	pose_with_timestamp.header.frame_id=str_frame_id;
	pub.publish(pose_with_timestamp);

}


// --------------------------------------------------------------------------
void sendTransformation(g2o::SE3Quat pose_se3, double timestamp, std::string str_frame_id, std::string str_child_frame_id)
// --------------------------------------------------------------------------
{
	static tf2_ros::TransformBroadcaster pose_broadcaster;

	geometry_msgs::TransformStamped pose_tf_stamped;	
	Eigen::Quaterniond pose_q=pose_se3.rotation();

	//fill in human tf		
	pose_tf_stamped.transform.translation.x = pose_se3.translation()[0];
	pose_tf_stamped.transform.translation.y = pose_se3.translation()[1];
	pose_tf_stamped.transform.translation.z = pose_se3.translation()[2];
	//human's heading in quaternion
	pose_tf_stamped.transform.rotation.x = pose_q.x();
	pose_tf_stamped.transform.rotation.y = pose_q.y();
	pose_tf_stamped.transform.rotation.z = pose_q.z();
	pose_tf_stamped.transform.rotation.w = pose_q.w();
	pose_tf_stamped.header.stamp = ros::Time(timestamp);
	//pose_tf_stamped.header.stamp =  ros::Time::now();
	pose_tf_stamped.header.frame_id = str_frame_id;
	pose_tf_stamped.child_frame_id = str_child_frame_id;
	pose_broadcaster.sendTransform(pose_tf_stamped);
}

// --------------------------------------------------------------------------
void publishPose(std::map<int, RobotPose > v_odom_dog1,
		std::map<int, RobotPose > v_odom_dog2,
		std::map<int, RobotPose > v_odom_human)
// --------------------------------------------------------------------------
{

	m_pose_update_mutex.lock();
	if(m_index_updated_pose_dog1>=0 && m_index_updated_pose_dog2>=0 && m_index_updated_pose_human>=0)
	{
		
		//get the dog1 pose based on the last updated SLAM pose and the odometry
		g2o::SE3Quat dog1_pose=getUpdatedPose(v_odom_dog1[m_index_updated_pose_dog1], m_updated_pose_dog1, m_latest_dog1_odom);
		//get the dog2 pose based on the last updated SLAM pose and the odometry
		g2o::SE3Quat dog2_pose=getUpdatedPose(v_odom_dog2[m_index_updated_pose_dog2], m_updated_pose_dog2, m_latest_dog2_odom);
		//get the human pose based on the last updated SLAM pose and the odometry
		g2o::SE3Quat human_pose=getUpdatedPose(v_odom_human[m_index_updated_pose_human], m_updated_pose_human, m_latest_human_odom);
		
		g2o::SE3Quat human_pose_in_human_frame=m_human_init_se3.inverse()*human_pose;//human pose in human local frame
		g2o::SE3Quat dog1_pose_in_human_frame=m_human_init_se3.inverse()*dog1_pose;//dog1 pose in human local frame
		g2o::SE3Quat dog2_pose_in_human_frame=m_human_init_se3.inverse()*dog2_pose;//dog2 pose in human local frame

		g2o::SE3Quat human_pose_in_dog1_frame=m_dog1_init_se3.inverse()*human_pose;//human pose in dog1 local frame
		g2o::SE3Quat dog1_pose_in_dog1_frame=m_dog1_init_se3.inverse()*dog1_pose;//dog1 pose in dog1 local frame
		g2o::SE3Quat dog2_pose_in_dog1_frame=m_dog1_init_se3.inverse()*dog2_pose;//dog2 pose in dog1 local frame

		g2o::SE3Quat human_pose_in_dog2_frame=m_dog2_init_se3.inverse()*human_pose;//human pose in dog2 local frame
		g2o::SE3Quat dog1_pose_in_dog2_frame=m_dog2_init_se3.inverse()*dog1_pose;//dog1 pose in dog2 local frame
		g2o::SE3Quat dog2_pose_in_dog2_frame=m_dog2_init_se3.inverse()*dog2_pose;//dog2 pose in dog2 local frame

        //pose of dog1 wrt human
		g2o::SE3Quat human_dog1_relative_pose=human_pose.inverse()*dog1_pose;
		//pose of dog2 wrt human
		g2o::SE3Quat human_dog2_relative_pose=human_pose.inverse()*dog2_pose;

		//pose of human wrt dog1
		g2o::SE3Quat dog1_human_relative_pose=dog1_pose.inverse()*human_pose;
		//pose of dog2 wrt dog1
		g2o::SE3Quat dog1_dog2_relative_pose=dog1_pose.inverse()*dog2_pose;

		//pose of human wrt dog2
		g2o::SE3Quat dog2_human_relative_pose=dog2_pose.inverse()*human_pose;
		//pose of dog1 wrt dog2
		g2o::SE3Quat dog2_dog1_relative_pose=dog2_pose.inverse()*dog1_pose;

		sendPose(m_pub_human_pose, human_pose, m_latest_human_odom.timestamp, m_str_human_frame_id);
		sendPose(m_pub_dog1_pose, dog1_pose, m_latest_dog1_odom.timestamp, m_str_dog1_frame_id);
		sendPose(m_pub_dog2_pose, dog2_pose, m_latest_dog2_odom.timestamp, m_str_dog2_frame_id);

		sendPose(m_pub_human_human_pose, human_pose_in_human_frame, m_latest_human_odom.timestamp, m_str_human_frame_id);
		sendPose(m_pub_human_dog1_pose, dog1_pose_in_human_frame, m_latest_dog1_odom.timestamp, m_str_dog1_frame_id);
		sendPose(m_pub_human_dog2_pose, dog2_pose_in_human_frame, m_latest_dog2_odom.timestamp, m_str_dog2_frame_id);

		sendPose(m_pub_dog1_human_pose, human_pose_in_dog1_frame, m_latest_human_odom.timestamp, m_str_human_frame_id);
		sendPose(m_pub_dog1_dog1_pose, dog1_pose_in_dog1_frame, m_latest_dog1_odom.timestamp, m_str_dog1_frame_id);
		sendPose(m_pub_dog1_dog2_pose, dog2_pose_in_dog1_frame, m_latest_dog2_odom.timestamp, m_str_dog2_frame_id);

		sendPose(m_pub_dog2_human_pose, human_pose_in_dog2_frame, m_latest_human_odom.timestamp, m_str_human_frame_id);
		sendPose(m_pub_dog2_dog1_pose, dog1_pose_in_dog2_frame, m_latest_dog1_odom.timestamp, m_str_dog1_frame_id);
		sendPose(m_pub_dog2_dog2_pose, dog2_pose_in_dog2_frame, m_latest_dog2_odom.timestamp, m_str_dog2_frame_id);


		sendTransformation(human_pose, m_latest_human_odom.timestamp, m_str_human_frame_id,m_str_human_child_frame_id);
		sendTransformation(dog1_pose, m_latest_dog1_odom.timestamp, m_str_dog1_frame_id,m_str_dog1_child_frame_id);
		sendTransformation(dog2_pose, m_latest_dog2_odom.timestamp, m_str_dog2_frame_id,m_str_dog2_child_frame_id);

        //relative transform (pose of dog1 wrt human)
		//sendTransformation(human_dog1_relative_pose, m_latest_dog1_odom.timestamp, m_str_human_child_frame_id,m_str_dog1_child_frame_id);
		//relative transform (pose of dog2 wrt human)
		//sendTransformation(human_dog2_relative_pose, m_latest_dog2_odom.timestamp, m_str_human_child_frame_id,m_str_dog2_child_frame_id);


		//relative transform (pose of human wrt dog1)
		//sendTransformation(dog1_human_relative_pose, m_latest_human_odom.timestamp, m_str_dog1_child_frame_id,m_str_human_child_frame_id);
		//relative transform (pose of dog2 wrt dog1)
		//sendTransformation(dog1_dog2_relative_pose, m_latest_dog2_odom.timestamp, m_str_dog1_child_frame_id,m_str_dog2_child_frame_id);


		//relative transform (pose of human wrt dog2)
		//sendTransformation(dog2_human_relative_pose, m_latest_human_odom.timestamp, m_str_dog2_child_frame_id,m_str_human_child_frame_id);
		//relative transform (pose of dog1 wrt dog2)
		//sendTransformation(dog2_dog1_relative_pose, m_latest_dog1_odom.timestamp, m_str_dog2_child_frame_id,m_str_dog1_child_frame_id);
		





	}

	m_pose_update_mutex.unlock();


}

// --------------------------------------------------------------------------
void* poseUpdateThread(void* args)
// --------------------------------------------------------------------------
{	
	while(1)
	{
		//update and publish pose at 20 HZ
		publishPose(m_vDog1Odometry, m_vDog2Odometry, m_vHumanOdometry);		
		usleep(50000);
	}
}


// --------------------------------------------------------------------------
void* poseGraphOptimizationThread(void* args)
// --------------------------------------------------------------------------
{	
	// Create the block solver
	typedef BlockSolver<BlockSolverX > SlamBlockSolver;
	OptimizationAlgorithmLevenberg* solver = new OptimizationAlgorithmLevenberg(
			g2o::make_unique<SlamBlockSolver>(
					g2o::make_unique<LinearSolverCholmod<SlamBlockSolver::PoseMatrixType>>()));

	m_optimizer.setAlgorithm(solver);
	m_optimizer.setVerbose(true);

	// add the parameter representing the sensor offset
	ParameterSE3Offset* sensorOffset = new ParameterSE3Offset;
	sensorOffset->setId(0);
	m_optimizer.addParameter(sensorOffset);

	SparseOptimizerTerminateAction* terminateAction = new SparseOptimizerTerminateAction;
	terminateAction->setGainThreshold(m_information_gain);
	terminateAction->setMaxIterations(m_incremental_iterations);
	m_optimizer.addPostIterationAction(terminateAction);


	int robustKernel=1;
	double bandwidth=1.0;

	while(1)
	{
		ros::Time begin = ros::Time::now();
		performPoseGraphOptimizationOnline(m_optimizer, m_vDog1Odometry, m_vDog2Odometry, m_vHumanOdometry, m_peer_uwb_optimization,m_external_uwb_optimization,robustKernel,bandwidth);	
		ros::Time end = ros::Time::now();
		double time_consumed =(end - begin).toSec();
		ROS_WARN("Time consumed for pose optimization: %f, dog1 pose sizes: %d, dog2 pose sizes: %d, human pose sizes: %d ",time_consumed, m_vDog1Odometry.size(), m_vDog2Odometry.size(), m_vHumanOdometry.size());		

		if(m_vDog1Odometry.size()==0)
		{
			ROS_ERROR("Please check if the dog1 odom (%s) is available.", m_str_dog1_odom_topic.c_str());
		}
		
		if(m_vDog2Odometry.size()==0)
		{
			ROS_ERROR("Please check if the dog2 odom (%s) is available.", m_str_dog2_odom_topic.c_str());
		}
				
		
		if(m_vHumanOdometry.size()==0)
		{
			ROS_ERROR("Please check if the human odom (%s) is available.", m_str_human_odom_topic.c_str());
		}

		if(m_vDog1UWBMeasurements.size()==0)
		{
			ROS_ERROR("Please check if the dog1 UWB (%s) is available.", m_str_dog1_uwb_topic.c_str());
		}
		
		if(m_vDog2UWBMeasurements.size()==0)
		{
			ROS_ERROR("Please check if the dog2 UWB (%s) is available.", m_str_dog2_uwb_topic.c_str());
		}

		if(m_vHumanUWBMeasurements.size()==0)
		{
			ROS_ERROR("Please check if the human UWB (%s) is available.", m_str_human_uwb_topic.c_str());
		}

		sleep(m_optimization_interval);
	}


}




// --------------------------------------------------------------------------
void generate_time_based_random_seed()
// --------------------------------------------------------------------------
{
	unsigned int seed;
	struct timeval tv;

	if ( gettimeofday(&tv, NULL) < 0 )
		fprintf( stderr, "error in gettimeofday : %s\n", strerror( errno) );
	seed = tv.tv_sec + tv.tv_usec;
	srand( seed );
}


// --------------------------------------------------------------------------
void decodeUWBNodes(std::string str_uwb_nodes, std::map<std::string, int >& v_uwb_nodes)
// --------------------------------------------------------------------------
{
	int end_of_str=false;
	std::string sub_uwb_config=str_uwb_nodes;
	while(end_of_str==false)
	{
		std::size_t found=sub_uwb_config.find(" ");
		if (found!=std::string::npos)
		{
			std::string node_id = sub_uwb_config.substr (0, found);
			sub_uwb_config=sub_uwb_config.substr (found+1);			
			v_uwb_nodes[node_id]=0;
			std::cout<<"uwb node configured:"<<node_id<<std::endl;
		}
		else
		{
			std::string node_id = sub_uwb_config;			
			v_uwb_nodes[node_id]=0;
			std::cout<<"uwb node configured:"<<node_id<<std::endl;
			end_of_str=true;
		}

	}
}

// --------------------------------------------------------------------------
void computeClosestUWBRanging(boost::circular_buffer<RangingMeasurement> vUWBMeasurements, 
		double timestamp, 		
		std::map<std::string, RangingMeasurement>& v_uwb_measure)
// --------------------------------------------------------------------------
{
	//we get the closest UWB (with in a timestamp) and also average them	
	std::map<std::string, std::map<std::string, std::vector<double> > > v_ranging_measurements_all_tags;
	
	for(int i=0;i<vUWBMeasurements.size();i++)	
	{
		double time_diff=fabs(vUWBMeasurements[i].timestamp-timestamp);
		if(time_diff<=m_uwb_range_time_window)
		{
			std::string tag_id=vUWBMeasurements[i].tag_id;
			std::map<std::string, double>::iterator it_ranging;
			std::map<std::string, double> v_ranging=vUWBMeasurements[i].v_ranging;

			std::map<std::string, double>::iterator it_rss;
			std::map<std::string, double> v_rss_difference=vUWBMeasurements[i].v_rss_difference;

			for(it_ranging = v_ranging.begin(); it_ranging != v_ranging.end(); it_ranging++)
			{
				v_ranging_measurements_all_tags[tag_id][it_ranging->first].push_back(it_ranging->second);
			}
			//std::cout<<i<<", Time diff: "<<time_diff<<std::endl;

		}
		

	}

	std::map<std::string, std::map<std::string, std::vector<double> > >::iterator v_tag_ranging_measurements;

	//get the average ranging	
	for(v_tag_ranging_measurements = v_ranging_measurements_all_tags.begin(); v_tag_ranging_measurements != v_ranging_measurements_all_tags.end(); v_tag_ranging_measurements++)
	{
		RangingMeasurement uwb_measure;
		uwb_measure.tag_id=v_tag_ranging_measurements->first;

		std::map<std::string, std::vector<double> > v_ranging_of_tag=v_tag_ranging_measurements->second;

		std::map<std::string, std::vector<double> >::iterator it_ranging_vector;
		for(it_ranging_vector = v_ranging_of_tag.begin(); it_ranging_vector != v_ranging_of_tag.end(); it_ranging_vector++)
		{
			double mean_ranging=-1;
			double sum_ranging=0;
			double sum_deviation_ranging=0;
			double mean_deviation_ranging=0;
			double min_range=INFINITY;
			double max_range=-INFINITY;
			double closest_range=-1;
			double predict_range=-1;
			std::vector<double> rangings=it_ranging_vector->second;			
			for(int i=0; i<rangings.size();i++)
			{
				sum_ranging=sum_ranging+rangings[i];
				if(rangings[i]<min_range)
				{
					min_range=rangings[i];
				}

				if(rangings[i]>max_range)
				{
					max_range=rangings[i];

				}
			}

			if(rangings.size()>0)
			{
				mean_ranging=sum_ranging/((double)(rangings.size()));
				closest_range=rangings[rangings.size()-1];				
				
				for(int i=0; i<rangings.size();i++)
				{
					sum_deviation_ranging=sum_deviation_ranging+(rangings[i]-mean_ranging)*(rangings[i]-mean_ranging);
				}
				mean_deviation_ranging=sqrt(sum_deviation_ranging/((double)(rangings.size())));

				//std::cout<<uwb_measure.tag_id<<" "<<it_ranging_vector->first<<" "<<mean_ranging<<", dev: "<<mean_deviation_ranging<<" "<<rangings.size()<<" "<<min_range<<" "<<max_range<<" "<<closest_range<<std::endl;
				
				//We need to push back the UWB ranging
				if(m_use_closest_ranging==true)
				{
					uwb_measure.v_ranging[it_ranging_vector->first]=closest_range;
				}
				else
				{
					uwb_measure.v_ranging[it_ranging_vector->first]=mean_ranging;
				}
				
		}
		}

	v_uwb_measure[v_tag_ranging_measurements->first]=uwb_measure;
	}
}


// --------------------------------------------------------------------------
void updatePeerOdom(std::map<int, RobotPose > &vOdometry,
boost::circular_buffer<RangingMeasurement> vUWBMeasurements, 
RobotPose &latest_odom,
int &index_odom,
double acc_d,
double acc_orientation,
double &last_acc_d,
double &last_acc_orientation,
double &last_timestamp)
// --------------------------------------------------------------------------
{
	if(latest_odom.timestamp>0)
	{
	RobotPose odom;
	odom.timestamp=latest_odom.timestamp;
	odom.x=latest_odom.x;
	odom.y=latest_odom.y;
	odom.z=latest_odom.z;
	odom.roll=latest_odom.roll;
	odom.pitch=latest_odom.pitch;
	odom.yaw=latest_odom.yaw;

	odom.qx=latest_odom.qx;
	odom.qy=latest_odom.qy;
	odom.qz=latest_odom.qz;
	odom.qw=latest_odom.qw;

	//get the closet UWB ranging and the RSS within a time window
	computeClosestUWBRanging(vUWBMeasurements, odom.timestamp, odom.v_uwb_measure);
	odom.acc_d=acc_d;
	odom.acc_orientation=acc_orientation;
	last_acc_d=acc_d;
	last_acc_orientation=acc_orientation;	
	last_timestamp=odom.timestamp;
	vOdometry[index_odom]=odom;			
	index_odom=index_odom+1;
	}
}


// --------------------------------------------------------------------------
//human odometry callback
void humanOdomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
// --------------------------------------------------------------------------
{
	double timestamp=msg->header.stamp.toSec();
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	double odom_x=msg->pose.pose.position.x;
	double odom_y=msg->pose.pose.position.y;
	double odom_z=msg->pose.pose.position.z;

	double odom_roll=roll;
	double odom_pitch=pitch;
	double odom_yaw=yaw;

	double odom_qx=msg->pose.pose.orientation.x;
	double odom_qy=msg->pose.pose.orientation.y;
	double odom_qz=msg->pose.pose.orientation.z;
	double odom_qw=msg->pose.pose.orientation.w;

	if(m_force_human_2d==true)
	{
		
		odom_z=0;
		odom_roll=0;
		odom_pitch=0;
		
		tf2::Quaternion pose_quaternion_2d;
		pose_quaternion_2d.setRPY( odom_roll, odom_pitch, odom_yaw);
		odom_qx=pose_quaternion_2d.x();
		odom_qy=pose_quaternion_2d.y();
		odom_qz=pose_quaternion_2d.z();
		odom_qw=pose_quaternion_2d.w();

	}	


	RobotPose odom;
	odom.timestamp=timestamp;
	odom.x=odom_x;
	odom.y=odom_y;
	odom.z=odom_z;
	odom.roll=odom_roll;
	odom.pitch=odom_pitch;
	odom.yaw=odom_yaw;

	odom.qx=odom_qx;
	odom.qy=odom_qy;
	odom.qz=odom_qz;
	odom.qw=odom_qw;

	if(m_index_human_odom==0)
	{
		//first human odometry
		//get the closest UWB ranging
		computeClosestUWBRanging(m_vHumanUWBMeasurements, timestamp, odom.v_uwb_measure);

		//we update the initial human odom
		m_initial_human_odom.timestamp=timestamp;
		m_initial_human_odom.x=odom_x;
		m_initial_human_odom.y=odom_y;
		m_initial_human_odom.z=odom_z;
		m_initial_human_odom.roll=odom_roll;
		m_initial_human_odom.pitch=odom_pitch;
		m_initial_human_odom.yaw=odom_yaw;

		m_initial_human_odom.qx=odom_qx;
		m_initial_human_odom.qy=odom_qy;
		m_initial_human_odom.qz=odom_qz;
		m_initial_human_odom.qw=odom_qw;



		odom.acc_d=m_acc_distance_human_odom;
		odom.acc_orientation=m_acc_orientation_human_odom;

		//we update the latest human odom
		m_latest_human_odom.timestamp=timestamp;
		m_latest_human_odom.x=odom_x;
		m_latest_human_odom.y=odom_y;
		m_latest_human_odom.z=odom_z;
		m_latest_human_odom.roll=odom_roll;
		m_latest_human_odom.pitch=odom_pitch;
		m_latest_human_odom.yaw=odom_yaw;

		m_latest_human_odom.qx=odom_qx;
		m_latest_human_odom.qy=odom_qy;
		m_latest_human_odom.qz=odom_qz;
		m_latest_human_odom.qw=odom_qw;

		m_vHumanOdometry[m_index_human_odom]=odom;			
		//we need to fill in the UWB rangings
		m_index_human_odom=m_index_human_odom+1;
		m_last_timestamp_human_odom=timestamp;
		

	}
	else
	{
		double moving_d=sqrt((odom.x-m_latest_human_odom.x)*(odom.x-m_latest_human_odom.x)
				+(odom.y-m_latest_human_odom.y)*(odom.y-m_latest_human_odom.y)
				+(odom.z-m_latest_human_odom.z)*(odom.z-m_latest_human_odom.z));			
		double moving_roll=carmen_normalize_theta(odom.roll-m_latest_human_odom.roll);
		double moving_pitch=carmen_normalize_theta(odom.pitch-m_latest_human_odom.pitch);
		double moving_yaw=carmen_normalize_theta(odom.yaw-m_latest_human_odom.yaw);			
		double moving_orientation=sqrt(moving_roll*moving_roll+moving_pitch*moving_pitch+moving_yaw*moving_yaw);

		double time_since_last_update=fabs(timestamp-m_last_timestamp_human_odom);

		m_acc_distance_human_odom=m_acc_distance_human_odom+moving_d;
		m_acc_orientation_human_odom=m_acc_orientation_human_odom+moving_orientation;


		if(fabs(m_acc_distance_human_odom-m_last_acc_distance_human_odom)>=m_update_min_d || fabs(carmen_normalize_theta(m_acc_orientation_human_odom-m_last_acc_orientation_human_odom))>=m_update_min_a || time_since_last_update>=m_update_min_time)
		{

			//get the closet UWB ranging and the RSS within a time window
			computeClosestUWBRanging(m_vHumanUWBMeasurements, timestamp, odom.v_uwb_measure);
			odom.acc_d=m_acc_distance_human_odom;
			odom.acc_orientation=m_acc_orientation_human_odom;
			m_last_acc_distance_human_odom=m_acc_distance_human_odom;
			m_last_acc_orientation_human_odom=m_acc_orientation_human_odom;	
			m_last_timestamp_human_odom=timestamp;
			m_vHumanOdometry[m_index_human_odom]=odom;			
			m_index_human_odom=m_index_human_odom+1;

			//We update the dog1 odom here and the acc
			updatePeerOdom(m_vDog1Odometry, m_vDog1UWBMeasurements, m_latest_dog1_odom, m_index_dog1_odom,
			m_acc_distance_dog1_odom, m_acc_orientation_dog1_odom,
			m_last_acc_distance_dog1_odom,m_last_acc_orientation_dog1_odom,
			m_last_timestamp_dog1_odom);

			updatePeerOdom(m_vDog2Odometry, m_vDog2UWBMeasurements, m_latest_dog2_odom, m_index_dog2_odom,
			m_acc_distance_dog2_odom, m_acc_orientation_dog2_odom,
			m_last_acc_distance_dog2_odom,m_last_acc_orientation_dog2_odom,
			m_last_timestamp_dog2_odom);			

		}



		//we update the latest human odom
		m_latest_human_odom.timestamp=timestamp;
		m_latest_human_odom.x=odom_x;
		m_latest_human_odom.y=odom_y;
		m_latest_human_odom.z=odom_z;
		m_latest_human_odom.roll=odom_roll;
		m_latest_human_odom.pitch=odom_pitch;
		m_latest_human_odom.yaw=odom_yaw;

		m_latest_human_odom.qx=odom_qx;
		m_latest_human_odom.qy=odom_qy;
		m_latest_human_odom.qz=odom_qz;
		m_latest_human_odom.qw=odom_qw;
	}


}


// --------------------------------------------------------------------------
//dog2 odom
void dog2OdomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
// --------------------------------------------------------------------------
{
	double timestamp=msg->header.stamp.toSec();
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	double odom_x=msg->pose.pose.position.x;
	double odom_y=msg->pose.pose.position.y;
	double odom_z=msg->pose.pose.position.z;

	double odom_roll=roll;
	double odom_pitch=pitch;
	double odom_yaw=yaw;

	double odom_qx=msg->pose.pose.orientation.x;
	double odom_qy=msg->pose.pose.orientation.y;
	double odom_qz=msg->pose.pose.orientation.z;
	double odom_qw=msg->pose.pose.orientation.w;


	if(m_force_robot_2d==true)
	{
		
		odom_z=0;
		odom_roll=0;
		odom_pitch=0;
		
		tf2::Quaternion pose_quaternion_2d;
		pose_quaternion_2d.setRPY( odom_roll, odom_pitch, odom_yaw);
		odom_qx=pose_quaternion_2d.x();
		odom_qy=pose_quaternion_2d.y();
		odom_qz=pose_quaternion_2d.z();
		odom_qw=pose_quaternion_2d.w();

	}	


	RobotPose odom;
	odom.timestamp=timestamp;
	odom.x=odom_x;
	odom.y=odom_y;
	odom.z=odom_z;
	odom.roll=odom_roll;
	odom.pitch=odom_pitch;
	odom.yaw=odom_yaw;

	odom.qx=odom_qx;
	odom.qy=odom_qy;
	odom.qz=odom_qz;
	odom.qw=odom_qw;

	if(m_index_dog2_odom==0)
	{
		computeClosestUWBRanging(m_vDog2UWBMeasurements, timestamp, odom.v_uwb_measure);

		//we update the initial dog odom
		m_initial_dog2_odom.timestamp=timestamp;
		m_initial_dog2_odom.x=odom_x;
		m_initial_dog2_odom.y=odom_y;
		m_initial_dog2_odom.z=odom_z;
		m_initial_dog2_odom.roll=odom_roll;
		m_initial_dog2_odom.pitch=odom_pitch;
		m_initial_dog2_odom.yaw=odom_yaw;

		m_initial_dog2_odom.qx=odom_qx;
		m_initial_dog2_odom.qy=odom_qy;
		m_initial_dog2_odom.qz=odom_qz;
		m_initial_dog2_odom.qw=odom_qw;


		odom.acc_d=m_acc_distance_dog2_odom;
		odom.acc_orientation=m_acc_orientation_dog2_odom;

		//we update the latest dog odom
		m_latest_dog2_odom.timestamp=timestamp;
		m_latest_dog2_odom.x=odom_x;
		m_latest_dog2_odom.y=odom_y;
		m_latest_dog2_odom.z=odom_z;
		m_latest_dog2_odom.roll=odom_roll;
		m_latest_dog2_odom.pitch=odom_pitch;
		m_latest_dog2_odom.yaw=odom_yaw;

		m_latest_dog2_odom.qx=odom_qx;
		m_latest_dog2_odom.qy=odom_qy;
		m_latest_dog2_odom.qz=odom_qz;
		m_latest_dog2_odom.qw=odom_qw;

		m_vDog2Odometry[m_index_dog2_odom]=odom;			
		m_index_dog2_odom=m_index_dog2_odom+1;
		m_last_timestamp_dog2_odom=timestamp;

	}
	else
	{
		double moving_d=sqrt((odom.x-m_latest_dog2_odom.x)*(odom.x-m_latest_dog2_odom.x)
				+(odom.y-m_latest_dog2_odom.y)*(odom.y-m_latest_dog2_odom.y)
				+(odom.z-m_latest_dog2_odom.z)*(odom.z-m_latest_dog2_odom.z));			
		double moving_roll=carmen_normalize_theta(odom.roll-m_latest_dog2_odom.roll);
		double moving_pitch=carmen_normalize_theta(odom.pitch-m_latest_dog2_odom.pitch);
		double moving_yaw=carmen_normalize_theta(odom.yaw-m_latest_dog2_odom.yaw);			
		double moving_orientation=sqrt(moving_roll*moving_roll+moving_pitch*moving_pitch+moving_yaw*moving_yaw);
		double time_since_last_update=fabs(timestamp-m_last_timestamp_dog2_odom);

		m_acc_distance_dog2_odom=m_acc_distance_dog2_odom+moving_d;
		m_acc_orientation_dog2_odom=m_acc_orientation_dog2_odom+moving_orientation;


		if(fabs(m_acc_distance_dog2_odom-m_last_acc_distance_dog2_odom)>=m_update_min_d || fabs(carmen_normalize_theta(m_acc_orientation_dog2_odom-m_last_acc_orientation_dog2_odom))>=m_update_min_a || time_since_last_update>=m_update_min_time)
		{

			//get the closet UWB ranging and the RSS within a time window
			computeClosestUWBRanging(m_vDog2UWBMeasurements, timestamp, odom.v_uwb_measure);

			odom.acc_d=m_acc_distance_dog2_odom;
			odom.acc_orientation=m_acc_orientation_dog2_odom;

			m_last_acc_distance_dog2_odom=m_acc_distance_dog2_odom;
			m_last_acc_orientation_dog2_odom=m_acc_orientation_dog2_odom;	
			m_last_timestamp_dog2_odom=timestamp;

			m_vDog2Odometry[m_index_dog2_odom]=odom;			
			m_index_dog2_odom=m_index_dog2_odom+1;

			updatePeerOdom(m_vHumanOdometry, m_vHumanUWBMeasurements, m_latest_human_odom, m_index_human_odom,
			m_acc_distance_human_odom, m_acc_orientation_human_odom,
			m_last_acc_distance_human_odom,m_last_acc_orientation_human_odom,
			m_last_timestamp_human_odom);

			//We update the dog1 odom here and the acc
			updatePeerOdom(m_vDog1Odometry, m_vDog1UWBMeasurements, m_latest_dog1_odom, m_index_dog1_odom,
			m_acc_distance_dog1_odom, m_acc_orientation_dog1_odom,
			m_last_acc_distance_dog1_odom,m_last_acc_orientation_dog1_odom,
			m_last_timestamp_dog1_odom);

		}



		//we update the latest dog odom
		m_latest_dog2_odom.timestamp=timestamp;
		m_latest_dog2_odom.x=odom_x;
		m_latest_dog2_odom.y=odom_y;
		m_latest_dog2_odom.z=odom_z;
		m_latest_dog2_odom.roll=odom_roll;
		m_latest_dog2_odom.pitch=odom_pitch;
		m_latest_dog2_odom.yaw=odom_yaw;

		m_latest_dog2_odom.qx=odom_qx;
		m_latest_dog2_odom.qy=odom_qy;
		m_latest_dog2_odom.qz=odom_qz;
		m_latest_dog2_odom.qw=odom_qw;
	}

}



// --------------------------------------------------------------------------
//dog1 odom
void dog1OdomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
// --------------------------------------------------------------------------
{
	double timestamp=msg->header.stamp.toSec();
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	double odom_x=msg->pose.pose.position.x;
	double odom_y=msg->pose.pose.position.y;
	double odom_z=msg->pose.pose.position.z;

	double odom_roll=roll;
	double odom_pitch=pitch;
	double odom_yaw=yaw;

	double odom_qx=msg->pose.pose.orientation.x;
	double odom_qy=msg->pose.pose.orientation.y;
	double odom_qz=msg->pose.pose.orientation.z;
	double odom_qw=msg->pose.pose.orientation.w;

	if(m_force_robot_2d==true)
	{
		
		odom_z=0;
		odom_roll=0;
		odom_pitch=0;
		
		tf2::Quaternion pose_quaternion_2d;
		pose_quaternion_2d.setRPY( odom_roll, odom_pitch, odom_yaw);
		odom_qx=pose_quaternion_2d.x();
		odom_qy=pose_quaternion_2d.y();
		odom_qz=pose_quaternion_2d.z();
		odom_qw=pose_quaternion_2d.w();

	}	

	RobotPose odom;
	odom.timestamp=timestamp;
	odom.x=odom_x;
	odom.y=odom_y;
	odom.z=odom_z;
	odom.roll=odom_roll;
	odom.pitch=odom_pitch;
	odom.yaw=odom_yaw;

	odom.qx=odom_qx;
	odom.qy=odom_qy;
	odom.qz=odom_qz;
	odom.qw=odom_qw;

	if(m_index_dog1_odom==0)
	{
		computeClosestUWBRanging(m_vDog1UWBMeasurements, timestamp, odom.v_uwb_measure);

		//we update the initial dog odom
		m_initial_dog1_odom.timestamp=timestamp;
		m_initial_dog1_odom.x=odom_x;
		m_initial_dog1_odom.y=odom_y;
		m_initial_dog1_odom.z=odom_z;
		m_initial_dog1_odom.roll=odom_roll;
		m_initial_dog1_odom.pitch=odom_pitch;
		m_initial_dog1_odom.yaw=odom_yaw;

		m_initial_dog1_odom.qx=odom_qx;
		m_initial_dog1_odom.qy=odom_qy;
		m_initial_dog1_odom.qz=odom_qz;
		m_initial_dog1_odom.qw=odom_qw;


		odom.acc_d=m_acc_distance_dog1_odom;
		odom.acc_orientation=m_acc_orientation_dog1_odom;

		//we update the latest dog odom
		m_latest_dog1_odom.timestamp=timestamp;
		m_latest_dog1_odom.x=odom_x;
		m_latest_dog1_odom.y=odom_y;
		m_latest_dog1_odom.z=odom_z;
		m_latest_dog1_odom.roll=odom_roll;
		m_latest_dog1_odom.pitch=odom_pitch;
		m_latest_dog1_odom.yaw=odom_yaw;

		m_latest_dog1_odom.qx=odom_qx;
		m_latest_dog1_odom.qy=odom_qy;
		m_latest_dog1_odom.qz=odom_qz;
		m_latest_dog1_odom.qw=odom_qw;

		m_vDog1Odometry[m_index_dog1_odom]=odom;			
		m_index_dog1_odom=m_index_dog1_odom+1;
		m_last_timestamp_dog1_odom=timestamp;


	}
	else
	{
		double moving_d=sqrt((odom.x-m_latest_dog1_odom.x)*(odom.x-m_latest_dog1_odom.x)
				+(odom.y-m_latest_dog1_odom.y)*(odom.y-m_latest_dog1_odom.y)
				+(odom.z-m_latest_dog1_odom.z)*(odom.z-m_latest_dog1_odom.z));			
		double moving_roll=carmen_normalize_theta(odom.roll-m_latest_dog1_odom.roll);
		double moving_pitch=carmen_normalize_theta(odom.pitch-m_latest_dog1_odom.pitch);
		double moving_yaw=carmen_normalize_theta(odom.yaw-m_latest_dog1_odom.yaw);			
		double moving_orientation=sqrt(moving_roll*moving_roll+moving_pitch*moving_pitch+moving_yaw*moving_yaw);
		double time_since_last_update=fabs(timestamp-m_last_timestamp_dog1_odom);

		m_acc_distance_dog1_odom=m_acc_distance_dog1_odom+moving_d;
		m_acc_orientation_dog1_odom=m_acc_orientation_dog1_odom+moving_orientation;


		if(fabs(m_acc_distance_dog1_odom-m_last_acc_distance_dog1_odom)>=m_update_min_d || fabs(carmen_normalize_theta(m_acc_orientation_dog1_odom-m_last_acc_orientation_dog1_odom))>=m_update_min_a || time_since_last_update>=m_update_min_time)
		{

			//get the closet UWB ranging and the RSS within a time window
			computeClosestUWBRanging(m_vDog1UWBMeasurements, timestamp, odom.v_uwb_measure);

			odom.acc_d=m_acc_distance_dog1_odom;
			odom.acc_orientation=m_acc_orientation_dog1_odom;

			m_last_acc_distance_dog1_odom=m_acc_distance_dog1_odom;
			m_last_acc_orientation_dog1_odom=m_acc_orientation_dog1_odom;	
			m_last_timestamp_dog1_odom=timestamp;

			m_vDog1Odometry[m_index_dog1_odom]=odom;			
			m_index_dog1_odom=m_index_dog1_odom+1;

			updatePeerOdom(m_vHumanOdometry, m_vHumanUWBMeasurements, m_latest_human_odom, m_index_human_odom,
			m_acc_distance_human_odom, m_acc_orientation_human_odom,
			m_last_acc_distance_human_odom,m_last_acc_orientation_human_odom,
			m_last_timestamp_human_odom);

			updatePeerOdom(m_vDog2Odometry, m_vDog2UWBMeasurements, m_latest_dog2_odom, m_index_dog2_odom,
			m_acc_distance_dog2_odom, m_acc_orientation_dog2_odom,
			m_last_acc_distance_dog2_odom,m_last_acc_orientation_dog2_odom,
			m_last_timestamp_dog2_odom);
			
		}



		//we update the latest dog odom
		m_latest_dog1_odom.timestamp=timestamp;
		m_latest_dog1_odom.x=odom_x;
		m_latest_dog1_odom.y=odom_y;
		m_latest_dog1_odom.z=odom_z;
		m_latest_dog1_odom.roll=odom_roll;
		m_latest_dog1_odom.pitch=odom_pitch;
		m_latest_dog1_odom.yaw=odom_yaw;

		m_latest_dog1_odom.qx=odom_qx;
		m_latest_dog1_odom.qy=odom_qy;
		m_latest_dog1_odom.qz=odom_qz;
		m_latest_dog1_odom.qw=odom_qw;
	}

}


// --------------------------------------------------------------------------
//UWB call back
void uwbCallBack(const nlink_parser::LinktrackNodeframe2 msg)
// --------------------------------------------------------------------------
{
	//We need to divide the UWB measurement into human uwb and dog uwb
	double timestamp=(double)(msg.sec)+(double)(msg.usec)/1000000.0;
	std::string node_id=std::to_string(msg.id);

	//fill in the UWB measurements
	RangingMeasurement uwb_measure;
	uwb_measure.timestamp=timestamp;
	uwb_measure.local_time=msg.localTime;
	uwb_measure.system_time=msg.systemTime;
	uwb_measure.tag_id=node_id;

	for(int i=0;i<msg.node.size();i++)
	{
		std::string anchor_node_id=std::to_string(msg.node[i].id);
		double distance=msg.node[i].dis;
		double rxRSS=msg.node[i].rxRssi;
		double fpRSS=msg.node[i].fpRssi;

		double rss_diff=fabs(fpRSS-rxRSS);
		uwb_measure.v_ranging[anchor_node_id]=distance;
		uwb_measure.v_rss_difference[anchor_node_id]=rss_diff;
	}

	std::map < std::string, int >::iterator it_dog1_uwb;
	it_dog1_uwb=m_uwb_nodes_dog1.find(node_id);

	if(it_dog1_uwb!=m_uwb_nodes_dog1.end())
	{
		//we push it to dog1 uwb buffer
		m_vDog1UWBMeasurements.push_back(uwb_measure);
	}
	else
	{
		std::map < std::string, int >::iterator it_dog2_uwb;
        it_dog2_uwb=m_uwb_nodes_dog2.find(node_id);
        if(it_dog2_uwb!=m_uwb_nodes_dog2.end())
        {
        	//we push it to dog1 uwb buffer
        	m_vDog2UWBMeasurements.push_back(uwb_measure);        	
        }
        else
        {
    		std::map < std::string, int >::iterator it_human_uwb;
    		it_human_uwb=m_uwb_nodes_human.find(node_id);

    		if(it_human_uwb!=m_uwb_nodes_human.end())
    		{

    			//we push it to human uwb buffer
    			m_vHumanUWBMeasurements.push_back(uwb_measure);
    		}
        	
        }


	}


}

// --------------------------------------------------------------------------
void setInitPose(double x, double y, double z, double theta, g2o::SE3Quat &se3_init)
// --------------------------------------------------------------------------
{	
	tf::Quaternion tf_q_init;
	tf_q_init.setRPY(0, 0, theta);
	Eigen::Vector3d trans_init(x,y,z);
	Eigen::Quaterniond q_init(tf_q_init.w(),tf_q_init.x(),tf_q_init.y(),tf_q_init.z());//w,x,y,z
	se3_init.setTranslation(trans_init);
	se3_init.setRotation(q_init);
}

// --------------------------------------------------------------------------
int main(int argc, char **argv)
// --------------------------------------------------------------------------
{

	ros::init(argc, argv, "uwb_SLAM");
	ros::NodeHandle node_handler("~");

	//for the random number generation 
	generate_time_based_random_seed();

	//We get the dog1 UWB
	std::string dog1_uwb_config;	
	if(node_handler.getParam("uwb_nodes_dog1", dog1_uwb_config))
	{
		decodeUWBNodes(dog1_uwb_config, m_uwb_nodes_dog1);
	}
	else
	{
		ROS_WARN("Please provide the node list of dog1 by param uwb_nodes_dog1");
	}


	//We get the dog2 UWB
	std::string dog2_uwb_config;	
	if(node_handler.getParam("uwb_nodes_dog2", dog2_uwb_config))
	{
		decodeUWBNodes(dog2_uwb_config, m_uwb_nodes_dog2);
	}
	else
	{
		ROS_WARN("Please provide the node list of dog2 by param uwb_nodes_dog2");
	}


	//We get the human UWB
	std::string human_uwb_config;		
	if(node_handler.getParam("uwb_nodes_human", human_uwb_config))
	{
		decodeUWBNodes(human_uwb_config, m_uwb_nodes_human);
	}
	else
	{
		ROS_WARN("Please provide the node list of dog by param uwb_nodes_human");
	}

	//We get the external UWB	
	std::string external_uwb_config;		
	if(node_handler.getParam("external_uwb_nodes", external_uwb_config))
	{
		decodeUWBNodes(external_uwb_config, m_uwb_nodes_external);
	}
	else
	{
		ROS_WARN("Please provide the node list of dog by param external_uwb_nodes");
	}


	//the topic config
	node_handler.getParam("human_odom_topic", m_str_human_odom_topic);
	node_handler.getParam("dog1_odom_topic", m_str_dog1_odom_topic);
	node_handler.getParam("dog1_uwb_topic", m_str_dog1_uwb_topic);
	node_handler.getParam("dog2_odom_topic", m_str_dog2_odom_topic);
	node_handler.getParam("dog2_uwb_topic", m_str_dog2_uwb_topic);
	node_handler.getParam("human_uwb_topic", m_str_human_uwb_topic);

	node_handler.getParam("human_frame_id", m_str_human_frame_id);
	node_handler.getParam("dog1_frame_id", m_str_dog1_frame_id);
	node_handler.getParam("dog2_frame_id", m_str_dog2_frame_id);

	node_handler.getParam("human_child_frame_id", m_str_human_child_frame_id);
	node_handler.getParam("dog1_child_frame_id", m_str_dog1_child_frame_id);
	node_handler.getParam("dog2_child_frame_id", m_str_dog2_child_frame_id);




	node_handler.getParam("plot_min_x", m_plot_min_x);
	node_handler.getParam("plot_max_x", m_plot_max_x);
	node_handler.getParam("plot_min_y", m_plot_min_y);
	node_handler.getParam("plot_max_y", m_plot_max_y);

	node_handler.getParam("human_init_x", m_human_init_x);
	node_handler.getParam("human_init_y", m_human_init_y);
	node_handler.getParam("human_init_z", m_human_init_z);
	node_handler.getParam("human_init_theta", m_human_init_theta);

	node_handler.getParam("dog1_init_x", m_dog1_init_x);
	node_handler.getParam("dog1_init_y", m_dog1_init_y);
	node_handler.getParam("dog1_init_z", m_dog1_init_z);
	node_handler.getParam("dog1_init_theta", m_dog1_init_theta);

	node_handler.getParam("dog2_init_x", m_dog2_init_x);
	node_handler.getParam("dog2_init_y", m_dog2_init_y);
	node_handler.getParam("dog2_init_z", m_dog2_init_z);
	node_handler.getParam("dog2_init_theta", m_dog2_init_theta);


	node_handler.getParam("update_min_d", m_update_min_d);
	node_handler.getParam("update_min_a", m_update_min_a);
	node_handler.getParam("update_min_time",m_update_min_time);

	node_handler.getParam("human_odom_tran_noise",m_human_odom_tran_noise);
	node_handler.getParam("human_odom_rot_noise",m_human_odom_rot_noise);
	node_handler.getParam("dog_odom_tran_noise",m_dog_odom_tran_noise);
	node_handler.getParam("dog_odom_rot_noise",m_dog_odom_rot_noise);

	node_handler.getParam("max_peer_range", m_max_peer_range);
	node_handler.getParam("max_external_range", m_max_range_external);	
	node_handler.getParam("close_uwb_noise", m_close_noise);
	node_handler.getParam("far_uwb_noise", m_far_uwb_noise);
	node_handler.getParam("close_range", m_close_range);

	m_far_uwb_information=1.0/(m_far_uwb_noise*m_far_uwb_noise);//UWB information for far range
	m_close_uwb_information=1.0/(m_close_noise*m_close_noise);//define the close range

	//std::cout<<"uwb noise: "<<m_far_uwb_information<<" "<<m_close_uwb_information<<std::endl;


	node_handler.getParam("uwb_ranging_average_time_window", m_uwb_range_time_window);
	node_handler.getParam("initial_guess_information", m_initial_guess_information);
	node_handler.getParam("gain", m_information_gain);

	node_handler.getParam("incremental_iteration", m_incremental_iterations);	
	node_handler.getParam("enable_gnuplot", m_enable_gnuplot);
	node_handler.getParam("use_mean_ranging", m_use_mean_ranging);
	node_handler.getParam("use_closest_ranging", m_use_closest_ranging);
	
	node_handler.getParam("optimization_interval", m_optimization_interval);

	node_handler.getParam("peer_uwb_optimization", m_peer_uwb_optimization);
	node_handler.getParam("external_uwb_optimization", m_external_uwb_optimization);
	node_handler.getParam("external_uwb_init_range", m_external_uwb_init_range);	
	node_handler.getParam("force_human_2d", m_force_human_2d);
	node_handler.getParam("force_robot_2d", m_force_robot_2d);
	node_handler.getParam("human_published_topic", m_str_human_pose_published_topic);	
	node_handler.getParam("dog1_published_topic", m_str_dog1_pose_published_topic);	
	node_handler.getParam("dog2_published_topic", m_str_dog2_pose_published_topic);	

	node_handler.getParam("human_in_human_localframe_published_topic", m_str_human_in_human_localframe_published_topic);	
	node_handler.getParam("dog1_in_human_localframe_published_topic", m_str_dog1_in_human_localframe_published_topic);	
	node_handler.getParam("dog2_in_human_localframe_published_topic", m_str_dog2_in_human_localframe_published_topic);	

	node_handler.getParam("human_in_dog1_localframe_published_topic", m_str_human_in_dog1_localframe_published_topic);	
	node_handler.getParam("dog1_in_dog1_localframe_published_topic", m_str_dog1_in_dog1_localframe_published_topic);	
	node_handler.getParam("dog2_in_dog1_localframe_published_topic", m_str_dog2_in_dog1_localframe_published_topic);	

	node_handler.getParam("human_in_dog2_localframe_published_topic", m_str_human_in_dog2_localframe_published_topic);	
	node_handler.getParam("dog1_in_dog2_localframe_published_topic", m_str_dog1_in_dog2_localframe_published_topic);	
	node_handler.getParam("dog2_in_dog2_localframe_published_topic", m_str_dog2_in_dog2_localframe_published_topic);
	

	std::cout<<m_human_odom_tran_noise<<" "<<m_human_odom_rot_noise<<" "<<m_dog_odom_tran_noise<<" "<<m_dog_odom_rot_noise<<std::endl;

	std::cout<<m_str_human_in_human_localframe_published_topic<<" "<<m_str_dog1_in_human_localframe_published_topic<<" "<<m_str_dog2_in_human_localframe_published_topic<<std::endl;

	std::cout<<m_str_human_in_dog1_localframe_published_topic<<" "<<m_str_dog1_in_dog1_localframe_published_topic<<" "<<m_str_dog2_in_dog1_localframe_published_topic<<std::endl;

	std::cout<<m_str_human_in_dog2_localframe_published_topic<<" "<<m_str_dog1_in_dog2_localframe_published_topic<<" "<<m_str_dog2_in_dog2_localframe_published_topic<<std::endl;

	std::cout<<"initial guess information: "<<m_initial_guess_information<<std::endl;
	setInitPose(m_human_init_x,m_human_init_y,m_human_init_z,m_human_init_theta,m_human_init_se3);
	setInitPose(m_dog1_init_x,m_dog1_init_y,m_dog1_init_z,m_dog1_init_theta,m_dog1_init_se3);
	setInitPose(m_dog2_init_x,m_dog2_init_y,m_dog2_init_z,m_dog2_init_theta,m_dog2_init_se3);

	//publisher of the human pose and uav pose
	m_pub_human_pose = node_handler.advertise<geometry_msgs::PoseWithCovarianceStamped>(m_str_human_pose_published_topic, 1);

	m_pub_dog1_pose = node_handler.advertise<geometry_msgs::PoseWithCovarianceStamped>(m_str_dog1_pose_published_topic, 1);

	m_pub_dog2_pose = node_handler.advertise<geometry_msgs::PoseWithCovarianceStamped>(m_str_dog2_pose_published_topic, 1);

	m_pub_human_human_pose = node_handler.advertise<geometry_msgs::PoseWithCovarianceStamped>(m_str_human_in_human_localframe_published_topic, 1);
	m_pub_human_dog1_pose = node_handler.advertise<geometry_msgs::PoseWithCovarianceStamped>(m_str_dog1_in_human_localframe_published_topic, 1);
	m_pub_human_dog2_pose = node_handler.advertise<geometry_msgs::PoseWithCovarianceStamped>(m_str_dog2_in_human_localframe_published_topic, 1);

	m_pub_dog1_dog1_pose = node_handler.advertise<geometry_msgs::PoseWithCovarianceStamped>(m_str_dog1_in_dog1_localframe_published_topic, 1);
	m_pub_dog1_human_pose = node_handler.advertise<geometry_msgs::PoseWithCovarianceStamped>(m_str_human_in_dog1_localframe_published_topic, 1);
	m_pub_dog1_dog2_pose = node_handler.advertise<geometry_msgs::PoseWithCovarianceStamped>(m_str_dog2_in_dog1_localframe_published_topic, 1);

	m_pub_dog2_dog2_pose = node_handler.advertise<geometry_msgs::PoseWithCovarianceStamped>(m_str_dog2_in_dog2_localframe_published_topic, 1);
	m_pub_dog2_human_pose = node_handler.advertise<geometry_msgs::PoseWithCovarianceStamped>(m_str_human_in_dog2_localframe_published_topic, 1);
	m_pub_dog2_dog1_pose = node_handler.advertise<geometry_msgs::PoseWithCovarianceStamped>(m_str_dog1_in_dog2_localframe_published_topic, 1);


	//subscribe to the odometry and UWB
	ros::Subscriber sub_dog1_odom = node_handler.subscribe(m_str_dog1_odom_topic, 5, dog1OdomCallBack);
	ros::Subscriber sub_dog2_odom = node_handler.subscribe(m_str_dog2_odom_topic, 5, dog2OdomCallBack);
	ros::Subscriber sub_human_odom = node_handler.subscribe(m_str_human_odom_topic, 5, humanOdomCallBack); 
	
	ros::Subscriber sub_dog1_uwb_ranging = node_handler.subscribe(m_str_dog1_uwb_topic, 5, uwbCallBack);
	ros::Subscriber sub_dog2_uwb_ranging = node_handler.subscribe(m_str_dog2_uwb_topic, 5, uwbCallBack);
	ros::Subscriber sub_human_uwb_ranging = node_handler.subscribe(m_str_human_uwb_topic, 5, uwbCallBack);    

	//back end thread for SLAM
	pthread_t tid_optimization;	
	pthread_create(&tid_optimization, NULL, poseGraphOptimizationThread, NULL);

	//back end thread for pose update
	pthread_t tid_pose_update;
	pthread_create(&tid_pose_update, NULL, poseUpdateThread, NULL);

	ros::spin();

	return 0;

}
