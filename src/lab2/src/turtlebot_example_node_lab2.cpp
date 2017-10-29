//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
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
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

ros::Publisher pose_publisher;
ros::Publisher marker_pub;
ros::Publisher map_pub;

nav_msgs::OccupancyGrid map_grid;
nav_msgs::MapMetaData map_meta_data;

double ips_x;
double ips_y;
double ips_yaw;
double center_x = NAN;
double center_y = NAN;
const double highprob = 85;
const double lowprob = 15;
const double RESOLUTION = 0.01;
const int HEIGHT = 10;
const int WIDTH = 10;
const int MAP_SIZE = int((HEIGHT/RESOLUTION)*(WIDTH/RESOLUTION));
const int HEIGHT_STEPS = HEIGHT/RESOLUTION;
const int WIDTH_STEPS = WIDTH/RESOLUTION;
const double OCCUPIED_MEASUREMENT_ODD = 0.75;
const double FREE_MEASUREMENT_ODD = -0.75;

double *log_map_vals;
int8_t *map_vals;

short sgn(int x) { return x >= 0 ? 1 : -1; }

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty 
//    vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;
    
    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s) y0+=sgn(dy2); else x0+=sgn(dx2);
        if (d < 0) d += inc1;
        else {
            d += inc2;
            if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg) 
{

    int i;
    for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

    ips_x = msg.pose[i].position.x ;
    ips_y = msg.pose[i].position.y ;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);

    // For the first time, finding the center of the grid as the initial position
    if (isnan(center_x) || isnan(center_y)) {
        center_x = ips_x;
        center_y = ips_y;

        // Setting the origin for the map
        geometry_msgs::Point origin;
        origin.x = center_x - WIDTH/2.0;
        origin.y = center_y - HEIGHT/2.0;
        origin.z = 0;
        geometry_msgs::Pose origin_pose;
        origin_pose.position = origin;
        map_meta_data.origin = origin_pose;
        map_grid.info = map_meta_data;
    }
}

void laser_callback(const sensor_msgs::LaserScan scan)
{
    double cur_angle = scan.angle_min;
    double cur_range, body_angle, body_x, body_y, inertial_x, inertial_y;
    int cur_index1, cur_index2, index1, index2, total_index;
    double angle_increment = scan.angle_increment;
    long int sizes = scan.ranges.size();
    long int bres_size;
    long int i, j;

    cur_index1 = int(round((HEIGHT_STEPS/2) + (ips_y - center_y)/RESOLUTION));
    cur_index2 = int(round((WIDTH_STEPS/2) + (ips_x - center_x)/RESOLUTION));

    if (!isnan(center_x) && !isnan(center_y)) {
        for (i = 0; i < sizes; i++) {
            cur_range = scan.ranges[i];
            if (!isnan(cur_range)) { 
                body_angle = cur_angle;
                body_x = cur_range*cos(body_angle);
                body_y = cur_range*sin(body_angle);
                inertial_x = cos(ips_yaw)*body_x - sin(ips_yaw)*body_y + ips_x;
                inertial_y = sin(ips_yaw)*body_x + cos(ips_yaw)*body_y + ips_y;
                index1 = int(round((HEIGHT_STEPS/2) + (inertial_y - center_y)/RESOLUTION));
                index2 = int(round((WIDTH_STEPS/2) + (inertial_x - center_x)/RESOLUTION));
                if (index1 >= 0 && index1 < HEIGHT_STEPS && index2 >= 0 && index2 <= WIDTH_STEPS) {
                    total_index = index1*WIDTH_STEPS + index2;
                    if (map_vals[total_index] == -1) {
                        map_vals[total_index] = 0;
                    }
                    log_map_vals[total_index] += OCCUPIED_MEASUREMENT_ODD;
                }

                // Using the bresenham line algorithm to get all the free pixels in between
                std::vector<int> indices1;
                std::vector<int> indices2;
                bresenham(cur_index1, cur_index2, index1, index2, indices1, indices2);
            
                bres_size = indices1.size();
                
                for (j = 0; j < bres_size; j++) {
                    total_index = indices1[j]*WIDTH_STEPS + indices2[j];
                    if (map_vals[total_index] == -1) {
                        map_vals[total_index] = 0;
                    }
                    log_map_vals[total_index] += FREE_MEASUREMENT_ODD;
                } 
            }
            cur_angle += angle_increment;
        }

        // Converting back to probability values using p = log/(1+log)
        for (i = 0; i < MAP_SIZE; i++) {
            if (map_vals[i] != -1) {
                map_vals[i] = int(round(exp(log_map_vals[i])/(1+exp(log_map_vals[i]))))*100;
            }
        }
        std::vector<signed char> map_vector(map_vals, map_vals+MAP_SIZE);
        map_grid.data = map_vector;
        map_pub.publish(map_grid);
    }
}

//Callback function for the Position topic (LIVE)
/*
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{

	ips_x X = msg.pose.pose.position.x; // Robot X psotition
	ips_y Y = msg.pose.pose.position.y; // Robot Y psotition
	ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}*/

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    //This function is called when a new map is received
    
    //you probably want to save the map into a form which is easy to work with
}


int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ROS_INFO("here");
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber laser_sub = n.subscribe("/scan", 1, laser_callback);
   
    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    // Need a way to read these values from a config file
    map_meta_data.resolution = RESOLUTION;
    map_meta_data.width = WIDTH_STEPS;
    map_meta_data.height = HEIGHT_STEPS;
    ROS_INFO("RESOLUTION %f", map_meta_data.resolution);
    ROS_INFO("map size %d", MAP_SIZE);
    ROS_INFO("HEIGHT_STEPS %d", HEIGHT_STEPS);
    ROS_INFO("width %d", WIDTH_STEPS);
    map_vals = new int8_t [MAP_SIZE];
    log_map_vals = new double [MAP_SIZE];
    for (int i = 0; i < MAP_SIZE; i++) {
        map_vals[i] = -1;
        log_map_vals[i] = 0;
    }

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

    	//Main loop code goes here:
    	vel.linear.x = 0; // set linear speed
    	vel.angular.z = 0; // set angular speed

    	velocity_publisher.publish(vel); // Publish the command velocity


    }

    return 0;
}
