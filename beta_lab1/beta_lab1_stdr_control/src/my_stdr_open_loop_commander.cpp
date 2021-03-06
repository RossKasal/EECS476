#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

//node to send Twist commands to the Simple 2-Dimensional Robot Simulator via cmd_vel
int main(int argc, char **argv) {
    ros::init(argc, argv, "stdr_commander"); 
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    //some "magic numbers"
    double sample_dt = 0.01; //specify a sample period of 10ms  
    double speed = 0.25; // 0.25m/s speed command
    double yaw_rate = M_PI/6; //0.5 rad/sec yaw rate command
    double time_3s_sec = 3.4; // 3.4 second timer
    double time_4_sec = 4.0; // 4 second timer
    
      
    geometry_msgs::Twist twist_cmd; //this is the message type required to send twist commands to STDR 
    // start with all zeros in the command message; should be the case by default, but just to be safe..
    twist_cmd.linear.x=0.0;
    twist_cmd.linear.y=0.0;    
    twist_cmd.linear.z=0.0;
    twist_cmd.angular.x=0.0;
    twist_cmd.angular.y=0.0;
    twist_cmd.angular.z=0.0;   

    ros::Rate loop_timer(1/sample_dt); //create a ros object from the ros “Rate” class; set 100Hz rate     
    double timer=0.0;
    //start sending some zero-velocity commands, just to warm up communications with STDR
    for (int i=0;i<10;i++) {
      twist_commander.publish(twist_cmd);
      loop_timer.sleep();
    }
		
    
    twist_cmd.linear.x=speed; //command to move forward
    timer=0.0; //reset the timer
    while(timer<time_4_sec) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }
          
        
    twist_cmd.linear.x=0.0; //stop moving forward
    twist_cmd.angular.z=yaw_rate; //and start spinning in place
    timer=0.0; //reset the timer
    while(timer<time_3s_sec) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }
          
    twist_cmd.linear.x=speed; //command to move forward
    twist_cmd.angular.z=0.0;
    timer=0.0; //reset the timer
    while(timer<time_4_sec) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }
          
        
    twist_cmd.linear.x=0.0; //stop moving forward
    twist_cmd.angular.z=yaw_rate; //and start spinning in place
    timer=0.0; //reset the timer
    while(timer<time_3s_sec) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }
          
    twist_cmd.linear.x=speed; //command to move forward
    twist_cmd.angular.z=0.0;
    timer=0.0; //reset the timer
    while(timer<time_4_sec) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }
          
        
    twist_cmd.linear.x=0.0; //stop moving forward
    twist_cmd.angular.z=yaw_rate; //and start spinning in place
    timer=0.0; //reset the timer
    while(timer<time_3s_sec-.4) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }
    
    
    twist_cmd.linear.x=speed; //command to move forward
    twist_cmd.angular.z=0.0;
    timer=0.0; //reset the timer
    while(timer<time_4_sec) {
          twist_commander.publish(twist_cmd);
          timer+=sample_dt;
          loop_timer.sleep();
          }
         
    //halt the motion
    twist_cmd.angular.z=0.0; 
    twist_cmd.linear.x=0.0; 
    for (int i=0;i<10;i++) {
      twist_commander.publish(twist_cmd);
      loop_timer.sleep();
    }               
    //done commanding the robot; node runs to completion
}

