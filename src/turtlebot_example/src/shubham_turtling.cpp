//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various 
// inputs and outputs.
// 
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <math.h>

int stage = 0;
double x, y, yaw;
double x_setpoint = 0;
double y_setpoint = 0;
double start_x = -1;
double start_y = -1;
double yaw_setpoint = 0;
double delta_yaw = 0;
double old_delta_yaw = 100.0;
const double ANGTOL = 0.05;
const double LINTOL = 0.1;

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	//This function is called when a new position message is received

	x = msg->pose.pose.position.x; // Robot X psotition
	y = msg->pose.pose.position.y; // Robot Y psotition
 	yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw

    if (start_x == -1) {
        start_x = x;
    }
    if (start_y == -1) {
        start_y = y;
    }

    x -= start_x;
    y -= start_y;

	ROS_INFO("x: %f", x);
	ROS_INFO("y: %f", y);
	ROS_INFO("yaw: %f", yaw);
}



int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 10, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    
    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Chec66k for new messages
        
       

        if (stage == 0) {
            x_setpoint = 0.6;
            y_setpoint = 0;
            yaw_setpoint = 0;
        } else if (stage == 1) {
            x_setpoint = 0.6;
            y_setpoint = -0.6;
            yaw_setpoint = -1.57;
        } else if (stage == 2) {
            x_setpoint = 0;
            y_setpoint = -0.6;
            yaw_setpoint = -3.14;
        } else if (stage == 3) {
            x_setpoint = 0;
            y_setpoint = 0;
            yaw_setpoint = -4.71;
        }
          
 		if (fabs(x - x_setpoint) < LINTOL && fabs(y - y_setpoint) < LINTOL) {
            if (stage == 4) {
                stage = 0;
            } else {
                stage += 1;
            }
            //ROS_INFO("Stage: %f",stage);
        }
    	//Main loop code goes here:
        if (fabs(yaw - yaw_setpoint) > ANGTOL) { 
            // set angle first
         //    delta_yaw = fabs(yaw - yaw_setpoint);
         //    if(delta_yaw < old_delta_yaw) {

	        vel.linear.x = 0; 
	        vel.angular.z = (yaw_setpoint - yaw)*KP_ANG;
        	// }
        	// else{
        	// 	ROS_INFO("here 3");
        	// 	vel.linear.x = 0;
        	// 	vel.angular.z = -0.1;
        	// }
        	// old_delta_yaw = delta_yaw;
        } else if (fabs(x - x_setpoint) > LINTOL || fabs(y - y_setpoint) > LINTOL){
            // set linear velocity 
            vel.linear.x = 0.1;
            vel.angular.z = 0;
        }

    	velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
