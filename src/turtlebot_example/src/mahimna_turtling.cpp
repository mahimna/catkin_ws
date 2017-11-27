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
double start_x = -1;
double start_y = -1;
double start_yaw = -5;
double x, y, yaw;
double x_setpoint, y_setpoint, yaw_setpoint, delta_yaw, old_delta_yaw = 100.0;
const double ANGTOL = 0.15;
const double LINTOL = 0.1;
const double MAX_ANG_VEL = 0.1;
const double MAX_LIN_VEL = 0.2;
double abs_pos, cur_pos;
const double KP_ANG= 0.1;
const double KP_LIN= 1.2; 


void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	//This function is called when a new position message is received

	x = msg->pose.pose.position.x; // Robot X psotition
	y = msg->pose.pose.position.y; // Robot Y psotition
 	yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw
    if (yaw < 0) {
        yaw += 2*M_PI;
    }

	if (start_x == -1) {
        start_x = x;
    } 
    if (start_y == -1) {
        start_y = y;
    }
    if (start_yaw == -5) {
        start_yaw = yaw;
    }

    x -= start_x;
    y -= start_y;
    // yaw -= start_yaw;

    // if (yaw < 0) {
    //     yaw += 2*M_PI;
    // }

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
    	ros::spinOnce();   //Check for new messages
        

        if (stage == 0) {
            x_setpoint = 0.5;
            y_setpoint = 0;
        } else if (stage == 1) {
            x_setpoint = 0.5;
            y_setpoint = 0.5;
        } else if (stage == 2) {
            x_setpoint = 0;
            y_setpoint = 0.5;
        } else if (stage == 3) {
            x_setpoint = 0;
            y_setpoint = 0;
        }

 		if (fabs(x - x_setpoint) < LINTOL && fabs(y - y_setpoint) < LINTOL) {
            stage = (stage + 1)%4;
            ROS_INFO("Stage: %d",stage);
        }


        yaw_setpoint = atan2(y_setpoint - y, x_setpoint - x);
        if (yaw_setpoint < 0) {
            yaw_setpoint += 2*M_PI;
        }

        // if(stage == 4)
        //     yaw_setpoint = fabs(yaw_setpoint);

    	//Main loop code goes here:
        if (fabs(yaw - yaw_setpoint) > ANGTOL) { 
            // set angle first
	        vel.linear.x = 0; 
	        vel.angular.z = (yaw_setpoint - yaw)*KP_ANG;
            // if(fabs(vel.angular.z)>MAX_ANG_VEL)
            // {
            //     vel.angular.z *= (MAX_ANG_VEL/fabs(vel.angular.z));
            // }

        } else {
            // set linear velocity 
            abs_pos = sqrt(x_setpoint*x_setpoint + y_setpoint*y_setpoint);
            cur_pos = sqrt(x*x + y*y);
            vel.linear.x = fabs(abs_pos - cur_pos)*KP_LIN;
            // if(fabs(vel.linear.x)>MAX_LIN_VEL)
            // {
            //     vel.linear.x *= (MAX_LIN_VEL/fabs(vel.linear.x));
            // }
            vel.angular.z = 0;
        }

    	velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
