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

double des_x = 1.0;
double des_y = 1.0;
double des_yaw = 0.0;
int stage = 0;
int pos = 0;
double cur_x = 0.0, init_x = 0.0;
double cur_y = 0.0, init_y = 0.0;
double cur_yaw = 0.0, init_yaw = 0.0;
double TOL = 0.1;


void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	//This function is called when a new position message is received

	cur_x = msg->pose.pose.position.x; // Robot X psotition
	cur_y = msg->pose.pose.position.y; // Robot Y psotition
 	cur_yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw
	if (cur_yaw < 0)
        cur_yaw += 2*M_PI;
    ROS_INFO("X = %f", cur_x);
	ROS_INFO("Y = %f", cur_y);
	ROS_INFO("YAW =  %f", cur_yaw);
	if (!stage)
	{
		init_x = cur_x;
		init_y = cur_y;
		init_yaw = cur_yaw;
		stage++;
	}
}

void calculateYaw()
{
	des_yaw = atan((des_y-cur_y)/(des_x-cur_x));
    //ROS_INFO("Des yaw = %f", des_yaw);
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

        //ROS_INFO("testing");    
    	//Main loop code goes here:
        if (stage)
        {

            if (fabs(des_yaw - cur_yaw) > TOL)
            {
                vel.angular.z = (des_yaw-cur_yaw)*0.5;
                vel.linear.x = 0.0;
            }
            else
            {
                vel.angular.z = 0.0;
                if (fabs(des_x - cur_x) > TOL || fabs(des_y - cur_y) > TOL)
                {
  //                 ROS_INFO("not yet");
                    calculateYaw();
                    vel.linear.x = (sqrt(des_x*des_x+des_y*des_y)-sqrt(cur_x*cur_x+cur_y*cur_y))*0.5;
                }
            }
        }
        velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
