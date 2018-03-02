#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <my_action_server/pathAction.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>


bool g_alarm = false; //alarm information (global)


void doneCb(const actionlib::SimpleClientGoalState& state,
        const my_action_server::pathResultConstPtr& result) {
	ROS_INFO("DONE");
}


void alarm_Callback(const std_msgs::Bool& message_holder){
	if (!g_alarm){
		g_alarm = message_holder.data;
	}
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "example_action_client_node"); // name this node 
    ros::NodeHandle nh_;
    ros::Subscriber alarm_subscriber = nh_.subscribe("lidar_alarm", 1, alarm_Callback); //now we have lidar info
       
    bool cancel = false; //cancellation of a goal
    my_action_server::pathGoal goal; //normal state goal
    my_action_server::pathGoal cancel_goal; //cancelled state goal
    
    actionlib::SimpleActionClient<my_action_server::pathAction> action_client("example_action", true);

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds

    if (!server_exists) {
        ROS_WARN("could not connect to server; halting");
        return 0; // bail out; optionally, could print a warning message and retry
    }

    ROS_INFO("connected to action server"); // if here, then we connected to the server;


    //POSE0 initial state
    double x_movement = 0.0;
    double y_movement = 0.0; 
    double z_movement = 0.0; //here for clarity but is not used
    double orientation = 0.0; //x,y,z,w but we are only using z
    
    //set the initial movements to 0
    goal.x_coordinate.push_back(x_movement);
    goal.y_coordinate.push_back(y_movement);
    goal.angle_value.push_back(orientation);
    
    //POSE1
    x_movement = 3.0;
    y_movement = 0.0;
    orientation = 0;
    
    goal.x_coordinate.push_back(x_movement);
    goal.y_coordinate.push_back(y_movement);
    goal.angle_value.push_back(orientation);
    
    //POSE2
    x_movement = 0.0;
    y_movement = 3.0;
    orientation = 1.57;
    
    goal.x_coordinate.push_back(x_movement);
    goal.y_coordinate.push_back(y_movement);
    goal.angle_value.push_back(orientation);
    
    //POSE3
    x_movement = 7.0;
    y_movement = 0.0;
    orientation = 0;
    
    goal.x_coordinate.push_back(x_movement);
    goal.y_coordinate.push_back(y_movement);
    goal.angle_value.push_back(orientation);
    
    //POSE4 
    x_movement = 0.0;
    y_movement = 6.0;
    orientation = 1.57;
    
    goal.x_coordinate.push_back(x_movement);
    goal.y_coordinate.push_back(y_movement);
    goal.angle_value.push_back(orientation);
    
    //POSE5
    x_movement = 5.0;
    y_movement = 0.0;
    orientation = 3.14;
    
    goal.x_coordinate.push_back(x_movement);
    goal.y_coordinate.push_back(y_movement);
    goal.angle_value.push_back(orientation);
    
    //POSE6
    x_movement = 0.0;
    y_movement = 9.0;
    orientation = 1.57;
    
    goal.x_coordinate.push_back(x_movement);
    goal.y_coordinate.push_back(y_movement);
    goal.angle_value.push_back(orientation);
    
    //POSE7
    x_movement = 2.0;
    y_movement = 0.0;
    orientation = 3.14;
    
    goal.x_coordinate.push_back(x_movement);
    goal.y_coordinate.push_back(y_movement);
    goal.angle_value.push_back(orientation);
    
    //POSE8
    x_movement = 0.0;
    y_movement = 12.0;
    orientation = 1.57;
    
    goal.x_coordinate.push_back(x_movement);
    goal.y_coordinate.push_back(y_movement);
    goal.angle_value.push_back(orientation);
    
    //POSE9
    x_movement = -1.0;
    y_movement = 0.0;
    orientation = 3.14;
    
    goal.x_coordinate.push_back(x_movement);
    goal.y_coordinate.push_back(y_movement);
    goal.angle_value.push_back(orientation);
    
    action_client.sendGoal(goal, &doneCb);
    

    while (true) {
        bool completed = action_client.waitForResult(ros::Duration(5.0));     
        
        if (completed) {
			ROS_INFO("You made it!");
        } 
        
        ROS_INFO("alarm state");
        if(g_alarm) {
			action_client.cancelGoal();
			cancel = true;
			ROS_INFO("Cancelling the goal");
		}
		ros::spinOnce();

	}
	
	if(cancel) { //spin in place
		
		ROS_INFO("Goal was cancelled");
		
		x_movement = 0.0;
		y_movement = 0.0;
		orientation = 0.0;
    
		cancel_goal.x_coordinate.push_back(x_movement);
		cancel_goal.y_coordinate.push_back(y_movement);
		cancel_goal.angle_value.push_back(orientation);
		
		x_movement = 0.0;
		y_movement = 0.0;
		orientation = 3.14;
    
		cancel_goal.x_coordinate.push_back(x_movement);
		cancel_goal.y_coordinate.push_back(y_movement);
		cancel_goal.angle_value.push_back(orientation);
		
		action_client.sendGoal(cancel_goal, &doneCb);
	}
	return 0;
}
