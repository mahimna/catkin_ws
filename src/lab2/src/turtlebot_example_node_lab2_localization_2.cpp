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
#include <random>


ros::Publisher pose_publisher;
ros::Publisher marker_pub;
ros::Publisher map_pub;

nav_msgs::OccupancyGrid map_grid;
nav_msgs::MapMetaData map_meta_data;

visualization_msgs::Marker points;

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

const int SAMPLES = 10000;
const double STDDEV_LIN = 0.10; // 10 cm
//const double STDDEV_ANG = M_PI/180/10; //0.1 deg
const double STDDEV_ANG = 0.1; //0.1 rads
const double SAMPLING_TIME = 1; // 1 Hz update rate 
// const double dt = 1.0/7.0;
// double prev_time;
ros::Time prev_time;
bool time_set = false;

bool particles_created = false;
double particle_x[SAMPLES];
double particle_y[SAMPLES];
double particle_yaw[SAMPLES];

std::normal_distribution<double> distribution_lin(0, STDDEV_LIN);
std::normal_distribution<double> distribution_ang(0, STDDEV_ANG);

bool first_error = true;

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

    int i, j;
    double err, random;

    for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

    ips_x = msg.pose[i].position.x ;
    ips_y = msg.pose[i].position.y ;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);
    // if (std::isnan(ips_yaw)){
    //     ROS_INFO("ORIENTATION x, y, z, w, %f, %f, %f, %f", msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z, msg.pose[i].orientation.w);    
    // }
    err = distribution_lin(generator);
    ips_x += err;
    err = distribution_lin(generator);
    ips_y += err;
    err = distribution_ang(generator);
    ips_yaw += err;

    // Random particle prior positions, with respect to initial IPS position.
    if (!particles_created) {
        for (i = 0; i < SAMPLES; i++)
        {
            err = distribution_lin(generator);
            particle_x[i] = ips_x+err;
            err = distribution_lin(generator);
            particle_y[i] = ips_y+err;
            err = distribution_ang(generator);
            particle_yaw[i] = ips_yaw+err;
        } 
        particles_created = true;
        // Insert error into IPS position.

    } 
    else {
        // Insert error into IPS position.
        // err = distribution_lin(generator);
        // ips_x += err;
        // err = distribution_lin(generator);
        // ips_y += err;
        // err = distribution_ang(generator);
        // ips_yaw += err;

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
            // if (std::isnan(w_x[i]) || std::isnan(w_y[i]) || std::isnan(w_yaw[i])){
            //     ROS_INFO("Weights x, y, yaw, %f, %f, %f", w_x[i], w_y[i], w_yaw[i]);    
            // }
            //x_w [i] = 1/exp(STDDEV_LIN)*(ips_x - particle_x[i]);
            if (std::isnan(w_yaw[i])){
                ROS_INFO("Yaw weight, %f", w_yaw[i]);    
            }
        }

        // Normalize weights.
        double sum_x = sum(w_x, SAMPLES);
        double sum_y = sum(w_y, SAMPLES);
        double sum_yaw = sum(w_yaw, SAMPLES);

        if (std::isnan(sum_yaw)){
            ROS_INFO("sum, %f", sum_yaw);    
        }
        
        w_x[0] /= sum_x;
        w_y[0] /= sum_y;
        w_yaw[0] = w_yaw[0]/sum_yaw;
        for(i = 1; i < SAMPLES; i++)
        {
            w_x[i] = w_x[i]/sum_x + w_x[i-1];
            w_y[i] = w_y[i]/sum_y + w_y[i-1];
            w_yaw[i] = w_yaw[i]/sum_yaw + w_yaw[i-1];
            // if (std::isnan(w_yaw[i])){
            //     ROS_INFO("Yaw, sum, %f, %f", w_yaw[i], sum_yaw);    
            // }
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
                    if (std::isnan(temp_yaw[i])){
                        ROS_INFO("Here");    
                    }
                    break;      
                }
            }
            if (std::isnan(temp_yaw[i]) && first_error){
                ROS_INFO("Here2");
                ROS_INFO("Sum %f", sum_yaw);
                ROS_INFO("Random: %f", random);
                ROS_INFO("yaw stddev %f", STDDEV_ANG);
                ROS_INFO("particle yaw");
                for (j = 0; j < SAMPLES; j++) {
                    ROS_INFO("%f", particle_yaw[j]);
                }
                ROS_INFO("ips yaw, %f", ips_yaw);
                ROS_INFO("weights"); 
                for (j = 0; j < SAMPLES; j++) {
                    ROS_INFO("%f ", w_yaw[j]);
                }
                first_error = false; 
            }
        }
        for (i = 0; i < SAMPLES; i++)
        {
            particle_x[i] = temp_x[i];
            particle_y[i] = temp_y[i];
            particle_yaw[i] = temp_yaw[i];
            // if (std::isnan(particle_x[i]) || std::isnan(particle_y[i]) || std::isnan(particle_yaw[i])){
            //     ROS_INFO("POSE x, y, yaw, %f, %f, %f", particle_x[i], particle_y[i], particle_yaw[i]);    
            // }
            // ROS_INFO("POSE x, y, yaw, %f, %f, %f", particle_x[i], particle_y[i], particle_yaw[i]);
        }
    }


    points.points.clear();
    for (i = 0; i < SAMPLES; i++)
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

void odom_callback(const nav_msgs::Odometry& msg)
{
    geometry_msgs::PoseWithCovariance pose = msg.pose;
    geometry_msgs::TwistWithCovariance twist = msg.twist;

    // Add the covariance values to each of the measurements?
    double angular_vel = twist.twist.angular.z;
    double linear_vel = twist.twist.linear.x;
    ros::Time cur_time = msg.header.stamp;
    if (time_set)
    {
        ros::Duration duration = cur_time - prev_time;
        double dt = duration.toSec();
        // double cur_time = ros::Time::now().toSec();
        // double dt = cur_time-prev_time;
        double err;

        // ROS_INFO("DT %f", dt);

        if (particles_created)
        {
            for (int i = 0; i < SAMPLES; i++)
            {
                // Calculate next position based on input
                err = distribution_lin(generator);
                // err = 0;
                particle_x[i] += linear_vel*cos(particle_yaw[i])*dt+err;
                err = distribution_lin(generator);
                // err = 0;
                particle_y[i] += linear_vel*sin(particle_yaw[i])*dt+err;
                // if (std::isnan(particle_x[i]) || std::isnan(particle_y[i])){
                //     ROS_INFO("ODOM x, y, yaw, l_v, a_v, dt, err, %f, %f, %f, %f, %f, %f, %f", particle_x[i], particle_y[i], particle_yaw[i], linear_vel, angular_vel, dt, err);    
                // }
                err = distribution_ang(generator);
                err = 0;
                particle_yaw[i] += angular_vel*dt+err;
                // ROS_INFO("ODOM x, y, yaw, %f, %f, %f", particle_x[i], particle_y[i], particle_yaw[i]);
            }
        }
    } else {
        time_set = true;
    }
    prev_time = cur_time;

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

    if (!std::isnan(center_x) && !std::isnan(center_y)) {
        for (i = 0; i < sizes; i++) {
            cur_range = scan.ranges[i];
            if (!std::isnan(cur_range)) { 
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
    double err;
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    //ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    // ros::Subscriber laser_sub = n.subscribe("/scan", 1, laser_callback);
    ros::Subscriber odom_sub = n.subscribe("/odom", 1, odom_callback);
   
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

    points.id = 0;
    points.header.frame_id = "map";
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.color.b = 1.0;
    points.color.a = 1.0;

    ROS_INFO("yaw stddev %f", STDDEV_ANG);

    // prev_time = ros::Time::now().toSec();

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

    	//Main loop code goes here:
    	vel.linear.x = 0.0; // set linear speed
    	vel.angular.z = 0.0; // set angular speed

    	velocity_publisher.publish(vel); // Publish the command velocity


    }

    return 0;
}
