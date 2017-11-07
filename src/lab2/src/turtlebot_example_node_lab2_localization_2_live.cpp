//  ///////////////////////////////////////////////////////////
//
// turtlebot_example_node_lab2_localization.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various 
// inputs and outputs needed for this lab
// 
// Author: James Servos 
// Edited: Nima Mohajerin
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <random>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


ros::Publisher pose_publisher;
ros::Publisher marker_pub;
ros::Publisher path_publisher;

nav_msgs::Path path;
visualization_msgs::Marker points;

double x_calculated;
double y_calculated;
double yaw_calculated;
double ips_x;
double ips_y;
double ips_yaw;

const int SAMPLES = 1000;
const double STDDEV_LIN = 0.10; // 10 cm
//const double STDDEV_ANG = M_PI/180/10; //0.1 deg
const double STDDEV_ANG = 0.1; //0.1 rads
const double SAMPLING_TIME = 1; // 1 Hz update rate 
ros::Time prev_time;
bool time_set = false;

bool particles_created = false;
double particle_x[SAMPLES];
double particle_y[SAMPLES];
double particle_yaw[SAMPLES];

std::normal_distribution<double> distribution_lin(0, STDDEV_LIN);
std::normal_distribution<double> distribution_ang(0, STDDEV_ANG);

std::default_random_engine generator;

short sgn(int x) { return x >= 0 ? 1 : -1; }

double calcWeight(double est, double meas, double stddev) { return (1/sqrt(2*M_PI*stddev*stddev)*exp(-(meas-est)*(meas-est)/(2*stddev*stddev))) + std::numeric_limits<double>::epsilon();}

double sum (double *arr, int size)
{
    //int size = sizeof(arr)/sizeof(arr[0]);
    double sum = 0;
    for (int i = 0; i < size; i++) sum += arr[i];
    return sum; 
}

double getRandom () { return ((double)1.0*rand()/RAND_MAX);}

//Callback function for the Position topic (SIMULATION)
// void pose_callback(const gazebo_msgs::ModelStates& msg) 
// {

//     int i, j;
//     double err, random;

//     for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

//     ips_x = msg.pose[i].position.x ;
//     ips_y = msg.pose[i].position.y ;
//     ips_yaw = tf::getYaw(msg.pose[i].orientation);
  
//     err = distribution_lin(generator);
//     ips_x += err;
//     err = distribution_lin(generator);
//     ips_y += err;
//     err = distribution_ang(generator);
//     ips_yaw += err;

//     // Random particle prior positions, with respect to initial IPS position.
//     if (!particles_created) {
//         for (i = 0; i < SAMPLES; i++)
//         {
//             err = distribution_lin(generator);
//             particle_x[i] = ips_x+err;
//             err = distribution_lin(generator);
//             particle_y[i] = ips_y+err;
//             err = distribution_ang(generator);
//             particle_yaw[i] = ips_yaw+err;
//         } 
//         particles_created = true;
//         x_calculated = ips_x;
//         y_calculated = ips_y;
//         yaw_calculated = ips_yaw; 
//     } 
//     else {
//         // Calculate weights.
//         double w_x[SAMPLES];
//         double w_y[SAMPLES];
//         double w_yaw[SAMPLES];
//         double temp_x[SAMPLES];
//         double temp_y[SAMPLES];
//         double temp_yaw[SAMPLES];

//         for(i = 0; i < SAMPLES; i++)
//         {
//             w_x[i] = calcWeight(particle_x[i], ips_x, STDDEV_LIN);
//             w_y[i] = calcWeight(particle_y[i], ips_y, STDDEV_LIN);
//             w_yaw[i] = calcWeight(particle_yaw[i], ips_yaw, STDDEV_ANG);
//         }

//         // Normalize weights.
//         double sum_x = sum(w_x, SAMPLES);
//         double sum_y = sum(w_y, SAMPLES);
//         double sum_yaw = sum(w_yaw, SAMPLES);

//         x_calculated = 0;
//         y_calculated = 0;
//         yaw_calculated = 0;

//         for(i = 0; i < SAMPLES; i++) 
//         {
//             x_calculated = w_x[i]/sum_x*particle_x[i];
//             y_calculated = w_y[i]/sum_y*particle_y[i];
//             yaw_calculated = w_yaw[i]/sum_yaw*particle_yaw[i];

//         }
        
//         w_x[0] /= sum_x;
//         w_y[0] /= sum_y;
//         w_yaw[0] = w_yaw[0]/sum_yaw;
//         for(i = 1; i < SAMPLES; i++)
//         {
//             w_x[i] = w_x[i]/sum_x + w_x[i-1];
//             w_y[i] = w_y[i]/sum_y + w_y[i-1];
//             w_yaw[i] = w_yaw[i]/sum_yaw + w_yaw[i-1];
//         }
        
//         for (i = 0; i < SAMPLES; i++) {
//             random = getRandom();
//             for (j = 0; j < SAMPLES; j++)
//             {
//                 if (random <= w_x[j]) {
//                     temp_x[i] = particle_x[j];
//                     break;
//                 }
//             }

//             random = getRandom();
//             for (j = 0; j < SAMPLES; j++)
//             {
//                 if (random <= w_y[j]){
//                     temp_y[i] = particle_y[j];
//                     break;
//                 }
//             }
//             random = getRandom();
//             for (j = 0; j < SAMPLES; j++)
//             {
//                 if (random <= w_yaw[j]) {
//                     temp_yaw[i] = particle_yaw[j];
//                     break;      
//                 }
//             }
//         }
//         for (i = 0; i < SAMPLES; i++)
//         {
//             particle_x[i] = temp_x[i];
//             particle_y[i] = temp_y[i];
//             particle_yaw[i] = temp_yaw[i];
//         }
//     }

//     geometry_msgs::PoseStamped position_calculated;
//     position_calculated.pose.position.x = x_calculated;
//     position_calculated.pose.position.y = y_calculated;
//     position_calculated.header.stamp = ros::Time::now();
//     path.poses.push_back(position_calculated);
//     path_publisher.publish(path);

//     points.points.clear();
//     for (i = 0; i < SAMPLES; i++)
//     {
//         geometry_msgs::Point p;
//         p.x = particle_x[i];
//         p.y = particle_y[i];
//         p.z = 0;

//         points.points.push_back(p);
//     }
//     points.header.stamp = ros::Time::now();
//     marker_pub.publish(points);
// }

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    int i, j;
    double err, random;

    ips_x = msg.pose.pose.position.x; // Robot X psotition
    ips_y = msg.pose.pose.position.y; // Robot Y psotition
    ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

    err = distribution_lin(generator);
    ips_x += err;
    err = distribution_lin(generator);
    ips_y += err;
    err = distribution_ang(generator);
    ips_yaw += err;

    // Random particle prior positions, with respect to initial IPS position.
    if (!particles_created) {
        std::normal_distribution<double> distribution_lin_2(0, 1);
        for (i = 0; i < SAMPLES; i++)
        {
            err = distribution_lin_2(generator);
            particle_x[i] = ips_x+err;
            err = distribution_lin_2(generator);
            particle_y[i] = ips_y+err;
            err = distribution_ang(generator);
            particle_yaw[i] = ips_yaw+err;
        } 
        particles_created = true;
        x_calculated = ips_x;
        y_calculated = ips_y;
        yaw_calculated = ips_yaw; 
    } 
    else {
        // Calculate weights.
        double w_x[SAMPLES];
        double w_y[SAMPLES];
        double w_yaw[SAMPLES];
        double temp_x[SAMPLES];
        double temp_y[SAMPLES];
        double temp_yaw[SAMPLES];

        for(i = 0; i < SAMPLES; i++)
        {
            w_x[i] = calcWeight(particle_x[i], ips_x, STDDEV_LIN);
            w_y[i] = calcWeight(particle_y[i], ips_y, STDDEV_LIN);
            w_yaw[i] = calcWeight(particle_yaw[i], ips_yaw, STDDEV_ANG);
        }

        // Normalize weights.
        double sum_x = sum(w_x, SAMPLES);
        double sum_y = sum(w_y, SAMPLES);
        double sum_yaw = sum(w_yaw, SAMPLES);

        x_calculated = 0;
        y_calculated = 0;
        yaw_calculated = 0;

        for(i = 0; i < SAMPLES; i++) 
        {
            x_calculated = w_x[i]/sum_x*particle_x[i];
            y_calculated = w_y[i]/sum_y*particle_y[i];
            yaw_calculated = w_yaw[i]/sum_yaw*particle_yaw[i];

        }
        
        w_x[0] /= sum_x;
        w_y[0] /= sum_y;
        w_yaw[0] = w_yaw[0]/sum_yaw;
        for(i = 1; i < SAMPLES; i++)
        {
            w_x[i] = w_x[i]/sum_x + w_x[i-1];
            w_y[i] = w_y[i]/sum_y + w_y[i-1];
            w_yaw[i] = w_yaw[i]/sum_yaw + w_yaw[i-1];
        }
        
        for (i = 0; i < SAMPLES; i++) {
            random = getRandom();
            for (j = 0; j < SAMPLES; j++)
            {
                if (random <= w_x[j]) {
                    temp_x[i] = particle_x[j];
                    break;
                }
            }

            random = getRandom();
            for (j = 0; j < SAMPLES; j++)
            {
                if (random <= w_y[j]){
                    temp_y[i] = particle_y[j];
                    break;
                }
            }
            random = getRandom();
            for (j = 0; j < SAMPLES; j++)
            {
                if (random <= w_yaw[j]) {
                    temp_yaw[i] = particle_yaw[j];
                    break;      
                }
            }
        }
        for (i = 0; i < SAMPLES; i++)
        {
            particle_x[i] = temp_x[i];
            particle_y[i] = temp_y[i];
            particle_yaw[i] = temp_yaw[i];
        }
    }

    geometry_msgs::PoseStamped position_calculated;
    position_calculated.pose.position.x = x_calculated;
    position_calculated.pose.position.y = y_calculated;
    position_calculated.header.stamp = ros::Time::now();
    path.poses.push_back(position_calculated);
    path_publisher.publish(path);
}

void odom_callback(const nav_msgs::Odometry& msg)
{
    geometry_msgs::PoseWithCovariance pose = msg.pose;
    geometry_msgs::TwistWithCovariance twist = msg.twist;

    // Add the covariance values to each of the measurements?
    double angular_vel = twist.twist.angular.z;
    double linear_vel = twist.twist.linear.x;
    double linear_error = twist.covariance[0];
    std::normal_distribution<double> dist_vel(0, linear_error);
    double rot_error = twist.covariance[35];
    std::normal_distribution<double> dist_rot(0, rot_error);

    ros::Time cur_time = msg.header.stamp;
    if (time_set)
    {
        ros::Duration duration = cur_time - prev_time;
        double dt = duration.toSec();
        double err = 0;

        if (particles_created)
        {
            for (int i = 0; i < SAMPLES; i++)
            {
                // Calculate next position based on input
                // err = distribution_lin(generator);
                particle_x[i] += linear_vel*cos(particle_yaw[i])*dt+dist_vel(generator);
                // err = distribution_lin(generator);
                particle_y[i] += linear_vel*sin(particle_yaw[i])*dt+dist_vel(generator);
                // err = distribution_ang(generator);
                particle_yaw[i] += angular_vel*dt+dist_rot(generator);
            }

            points.points.clear();
            for (int i = 0; i < SAMPLES; i++)
            {
                geometry_msgs::Point p;
                p.x = particle_x[i];
                p.y = particle_y[i];
                p.z = 0;

                points.points.push_back(p);
            }
            points.header.stamp = ros::Time::now();
            marker_pub.publish(points);

        }
    } else {
        time_set = true;
    }
    prev_time = cur_time;

}

int main(int argc, char **argv)
{
    double err;
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
    ros::Subscriber odom_sub = n.subscribe("/odom", 1, odom_callback);
   
    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    path_publisher = n.advertise<nav_msgs::Path>("/path", 1, true);

    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    path.header.frame_id = "map";

    points.id = 0;
    points.header.frame_id = "map";
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.5;
    points.scale.y = 0.5;
    points.color.b = 1.0;
    points.color.a = 1.0;

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

    	//Main loop code goes here:
    	vel.linear.x = 0.0; // set linear speed
    	vel.angular.z = 0.0; // set angular speed

    }

    return 0;
}
