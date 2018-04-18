#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <my_action_server/pathAction.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <math.h>
#include <cmath>


double sgn(double x);
double min_spin(double spin_angle);
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);
ros::Publisher g_twist_commander;


void do_halt();
void do_move(double distance);
void do_spin(double spin_ang);


class ExampleActionServer {
private:
    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation
    actionlib::SimpleActionServer<my_action_server::pathAction> as_;
    
    // here are some message types to communicate with our client(s)
    my_action_server::pathGoal goal_; // goal message, received from client
    my_action_server::pathResult result_; // put results here, to be sent back to the client when done w/ goal
    my_action_server::pathFeedback feedback_; // not used in this example; 


public:
    ExampleActionServer(); //define the body of the constructor outside of class definition

    ~ExampleActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<my_action_server::pathAction>::GoalConstPtr& goal);
};


ExampleActionServer::ExampleActionServer() :
   as_(nh_, "example_action", boost::bind(&ExampleActionServer::executeCB, this, _1),false) 
{
    ROS_INFO("in constructor of exampleActionServer...");
    // do any other desired initializations here...specific to your implementation
    
    as_.start(); //start the server running
}


void ExampleActionServer::executeCB(const actionlib::SimpleActionServer<my_action_server::pathAction>::GoalConstPtr& goal) {
	double current_angle = 0.0;
	int npts = goal->x_coordinate.size(); //number of poses (not really necessary other than for cleaner processes
	double distance = 0.0;
	double delta_angle = 0.0;
	
	for(int i=0;i<npts;i++){
		distance = sqrt(pow(goal->x_coordinate[i],2.0)+pow(goal->y_coordinate[i],2.0));
		delta_angle = (goal->angle_value[i]) - current_angle;
		
		do_spin(delta_angle);
		
		current_angle = goal->angle_value[i];
		
		do_move(distance);
		
		if (as_.isPreemptRequested()){	
			ROS_WARN("Goal was cancelled");
			result_.result_completed = false;
			as_.setAborted(result_); 
			return; // done with callback
		}
	}

	result_.result_completed = true;
	as_.setSucceeded(result_);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "example_action_server_node"); // name this node 
    ros::NodeHandle nh_;
    
    g_twist_commander = nh_.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1); //global publisher object
    
    ExampleActionServer as_object;
    
    ROS_INFO("going into spin");
    ros::spin();
    return 0;
}

//All of the functions that I need from path_service2.cpp

#include <ros/ros.h>
#include <example_ros_service/PathSrv.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <math.h>
using namespace std;
//some tunable constants, global
const double g_move_speed = 1.0; // set forward speed to this value, e.g. 1m/s
const double g_spin_speed = 1.0; // set yaw rate to this value, e.g. 1 rad/s
const double g_sample_dt = 0.01;
const double g_dist_tol = 0.01; // 1cm
//global variables, including a publisher object
geometry_msgs::Twist g_twist_cmd;
//ros::Publisher g_twist_commander; //global publisher object
geometry_msgs::Pose g_current_pose; // not really true--should get this from odom 


// here are a few useful utility functions:
double sgn(double x);
double min_spin(double spin_angle);
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

void do_halt();
void do_move(double distance);
void do_spin(double spin_ang);

//signum function: strip off and return the sign of the argument
double sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}

//a function to consider periodicity and find min delta angle
double min_spin(double spin_angle) {
        if (spin_angle>M_PI) {
            spin_angle -= 2.0*M_PI;}
        if (spin_angle< -M_PI) {
            spin_angle += 2.0*M_PI;}
         return spin_angle;   
}            

// a useful conversion function: from quaternion to yaw
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//and the other direction:
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

// a few action functions:
//a function to reorient by a specified angle (in radians), then halt
void do_spin(double spin_ang) {
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(spin_ang)/g_spin_speed;
    g_twist_cmd.angular.z= sgn(spin_ang)*g_spin_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt(); 
}

//a function to move forward by a specified distance (in meters), then halt
void do_move(double distance) { // always assumes robot is already oriented properly
                                // but allow for negative distance to mean move backwards
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(distance)/g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance)*g_move_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt();
}

void do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);   
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i=0;i<10;i++) {
          g_twist_commander.publish(g_twist_cmd);
          loop_timer.sleep(); 
          }   
}
