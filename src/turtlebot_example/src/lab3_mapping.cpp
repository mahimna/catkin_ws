//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 3
//
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include <limits>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define	PADDING		2
#define KP		0.1
#define NUM_WAYPOINTS	3
#define NUM_EDGES	15
#define OFFSET_X	1.0f
#define	OFFSET_Y	5.0f

//Sim Only
const double waypointsX[NUM_WAYPOINTS] = {4.0, 8.0, 8.0};
const double waypointsY[NUM_WAYPOINTS] = {0.0, -4.0, 0.0};

/*
//Live Only
const double waypointsX[NUM_WAYPOINTS] = [1.0, 3.0, 4.5];
const double waypointsY[NUM_WAYPOINTS] = [3.0, 3.5, 0.5];
*/

ros::Publisher marker_pub;
visualization_msgs::Marker points;

int8_t *mapData;
uint32_t mapWidth = 0;
uint32_t mapHeight = 0;
double mapRes = 0;
uint8_t foundMap = 0;

uint16_t numNodes = 500;
double *nodesX;
double *nodesY;

uint16_t numEdges = 15;
double *connectedEdges;

double finalX = 0;
double finalY = 0;

double ipsX = 0;
double ipsY = 0; 
double ipsYaw = 0;
double initX = NAN;
double initY = NAN;
double initYaw = NAN;

int plotId = 2;

std::vector<int> shortestPath;
uint8_t pointId = 1;
double carrotAng = 0.0;

#define TAGID 0

short sgn(int x) { return x >= 0 ? 1 : -1; }

void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& indX, std::vector<int>& indY) {

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

    indX.push_back(x0);
    indY.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s) y0+=sgn(dy2); else x0+=sgn(dx2);
        if (d < 0) d += inc1;
        else {
            d += inc2;
            if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
        }

        //Add point to vector
        indX.push_back(x0);
        indY.push_back(y0);
    }
}

//Callback function for the Position topic (LIVE)
/*
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
  //This function is called when a new position message is received
  double X = msg.pose.pose.position.x; // Robot X psotition
  double Y = msg.pose.pose.position.y; // Robot Y psotition
  double Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
  std::cout << "X: " << X << ", Y: " << Y << ", Yaw: " << Yaw << std::endl ;
}
*/

//Callback function for Position topic (SIM)
void pose_callback(const gazebo_msgs::ModelStates& msg)
{
  int i;
  geometry_msgs::Point p;

  for(i = 0; i < msg. name.size(); i++) if(msg.name[i] == "mobile_base") break;

  ipsX = msg.pose[i].position.x;
  ipsY = msg.pose[i].position.y;
  ipsYaw = tf::getYaw(msg.pose[i].orientation);

  //Shift from 0-2PI to -PI-PI
  if (ipsYaw > M_PI)
    ipsYaw -= 2*M_PI;

  if (std::isnan(initX) || std::isnan(initY))
  {
    initX = ipsX;
    initY = ipsY;
    initYaw = ipsYaw;
    ROS_INFO("%f %f %f", initX, initY, initYaw);
  }

  points.header.frame_id = "/map";
  points.id = 1;   //each curve must have a unique id or you will overwrite$
  points.type = visualization_msgs::Marker::POINTS;
  points.action = visualization_msgs::Marker::ADD;
  points.ns = "points_and_lines";
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  points.color.b = 0.0;
  points.color.r = 1.0;
  points.color.a = 1.0;
  points.points.clear();
  p.x = ipsX+OFFSET_X;
  p.y = ipsY+OFFSET_Y;
  p.z = 0; //not used
  points.points.push_back(p);
  marker_pub.publish(points);
}

void plotLines()
{
  geometry_msgs::Point p;
  points.header.frame_id = "/map";
  points.type = visualization_msgs::Marker::LINE_LIST;
  points.action = visualization_msgs::Marker::ADD;
  points.ns = "points_and_lines";
  points.scale.x = 0.1;
  points.scale.y = 0.1;
  points.color.b = 1.0;
  points.color.a = 1.0;
  points.color.r = 0.0;
  points.points.clear();
  points.id = 1;

  for(int i = 0; i < numNodes; i++)
  {
    for(int j = 0; j < numNodes; j++)
    {
      if (connectedEdges[i*numNodes+j] != 0)
      {
        p.x = nodesX[i];
        p.y = nodesY[i];
        p.z = 0.0;
        points.points.push_back(p);

        p.x = nodesX[j];
        p.y = nodesY[j];
        p.z = 0.0;
        points.points.push_back(p);
      }
    }
  }
  marker_pub.publish(points);
}

void plotPoints()
{
  geometry_msgs::Point p;
  points.header.frame_id = "/map";
  points.id = 0;   //each curve must have a unique id or you will overwrite an old ones
  points.type = visualization_msgs::Marker::POINTS;
  points.action = visualization_msgs::Marker::ADD;
  points.ns = "points_and_lines";
  points.scale.x = 0.1;
  points.scale.y = 0.1;
  points.color.b = 1.0;
  points.color.a = 1.0;
  points.color.r = 0.0;
  points.points.clear();
  for(int i = 0; i < numNodes; i++)
  {
    p.x = nodesX[i];
    p.y = nodesY[i];
    p.z = 0; //not used
    points.points.push_back(p);
  }
  marker_pub.publish(points);
}

double getNormDist(int i, int j)
{
  return sqrt((nodesX[j]-nodesX[i])*(nodesX[j]-nodesX[i])+(nodesY[j]-nodesY[i])*(nodesY[j]-nodesY[i]));
}

double getNormDistPoints(double x0, double y0, double x1, double y1)
{
  return sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0));
}

void sort(double arr[], int ind[], int left, int right)
{
  int i = left;
  int j = right; 
  int tmpInd;
  double tmp;
  double pivot = arr[(left+right)/2];

  while (i <= j)
  {
    while (arr[i] < pivot)
      i++;
    while (arr[j] > pivot)
      j--;
    if (i <= j)
    {
      tmp = arr[i];
      arr[i] = arr[j];
      arr[j] = tmp;

      tmpInd = ind[i];
      ind[i] = ind[j];
      ind[j] = tmpInd;

      i++;
      j--;
    }
  }

  if (left < j)
    sort(arr, ind, left, j);
  if (i < right)
    sort(arr, ind, i, right);
}

int8_t checkCollision(int i, int j)
{
  int x0 = (int)(nodesX[i]/mapRes);
  int x1 = (int)(nodesX[j]/mapRes);
  int y0 = (int)(nodesY[i]/mapRes);
  int y1 = (int)(nodesY[j]/mapRes);
  int ind;
  long int size = 0;
  std::vector<int> indX;
  std::vector<int> indY;

  bresenham(x0, y0, x1, y1, indX, indY);

  size = indX.size();

  for (int k = 0; k < size; k++)
  {
    for (int l = indY[k]-PADDING; l <= indY[k]+PADDING; l++)
    {
      for (int m = indX[k]-PADDING; m <= indX[k]+PADDING; m++)
      {
        ind = l*mapHeight+m;
        if (ind >= 0 && ind < mapHeight*mapWidth && mapData[ind] == 100)
        {
            return 1;
        }
      }
    }
  }

  return 0;

}


void connectEdges()
{
  int i = 0;
  int j = 0;
  double dists[numNodes];
  int ind[numNodes];
  connectedEdges = new double[numNodes*numNodes];
  for (i = 0; i < numNodes*numNodes; i++)
    connectedEdges[i] = 0;

  for (i = 0; i < numNodes; i++)
  {
    for (j = 0; j < numNodes; j++)
    {
      dists[j] = getNormDist(i, j);
      ind[j] = j;
    }
    sort(dists, ind, 0, numNodes-1);

    for (j = 1; j <= NUM_EDGES; j++)
    {
      if(!checkCollision(i, ind[j]))
      {
        connectedEdges[i*numNodes+ind[j]] = dists[j];
        connectedEdges[ind[j]*numNodes+i] = dists[j];
      }
    }
  }
  ROS_INFO("Connected Closest Edges & Removed Collisions");
}

void removeCollisionNodes()
{
  double tempX[numNodes];
  double tempY[numNodes];
  int ind;
  int count = 0;
  for (int i = 0; i < numNodes; i++)
  {
    ind = nodesY[i]/mapRes*mapHeight+nodesX[i]/mapRes;
    if (mapData[ind] == 0)
    {
      tempX[count] = nodesX[i];
      tempY[count] = nodesY[i]; 
      count++;
    }
  }

  free(nodesX);
  free(nodesY);

  numNodes = count+NUM_WAYPOINTS+1; //Add one extra for init point
  nodesX = new double[numNodes];
  nodesY = new double[numNodes];

  for (int i = 0; i < count; i++)
  {
    nodesX[i] = tempX[i];
    nodesY[i] = tempY[i];
  }

  nodesX[count] = initX+OFFSET_X;
  nodesY[count++] = initY+OFFSET_Y;

  for (int i = 0; i < NUM_WAYPOINTS; i++)
  {
    nodesX[count] = waypointsX[i]+OFFSET_X;
    nodesY[count++] = waypointsY[i]+OFFSET_Y; 
  }

  ROS_INFO("Number of nodes kept %d.", numNodes);
}

void generateNodes()
{
  nodesX = new double[numNodes];
  nodesY = new double[numNodes];
  for (int i = 0; i < numNodes; i++)
  {
    nodesX[i] = (rand() % mapWidth)*mapRes;
    nodesY[i] = (rand() % mapHeight)*mapRes;
  }
  ROS_INFO("Generated %d random nodes.", numNodes);
}

void aStar(int startingPoint, int endingPoint){
  geometry_msgs::Point p;

  ROS_INFO("Starting Point: %f , %f", nodesX[startingPoint], nodesY[startingPoint]);
  ROS_INFO("Ending Point: %f, %f", nodesX[endingPoint], nodesY[endingPoint]);

  double distsToEnd[numNodes];
  double distances[numNodes];
  double distances2[numNodes];
  int path[numNodes];
  int ind[numNodes];

  for (int i = 0; i < numNodes; i++) {

    distsToEnd[i] = sqrt((nodesX[i]-nodesX[endingPoint])*(nodesX[i]-nodesX[endingPoint]) + (nodesY[i]-nodesY[endingPoint])*(nodesY[i]-nodesY[endingPoint]));
    distances[i] = std::numeric_limits<double>::infinity();
    path[i] = -1;
    ind[i] = i;
  }

  int curPoint = startingPoint;

  std::vector<int> visited;
  distances[curPoint] = 0;

  while(visited.size() != numNodes) {

    visited.push_back(curPoint);

    if (curPoint == endingPoint) {
      break;
    }

    for (int i = 0; i < numNodes; i++) {
      if (connectedEdges[curPoint*numNodes + i] != 0) {
        if (distances[curPoint] + connectedEdges[curPoint*numNodes + i] < distances[i]) {
          distances[i] = distances[curPoint] + connectedEdges[curPoint*numNodes + i];
          path[i] = curPoint;
        }
      }
    }

    for (int i = 0; i < numNodes; i++) {
      distances2[i] = distances[i] + distsToEnd[i];
      ind[i] = i;
    }

    sort(distances2, ind, 0, numNodes-1);

    for (int i = 0; i < numNodes; i++) {
      bool notVisited = true;
      for (int j = 0; j < visited.size() && notVisited; j++) {
        if (ind[i] == visited[j]) {
          notVisited = false;
        }
      }
      if (notVisited) {
        curPoint = ind[i];
        break;
      }
    }
  }

  std::vector<int> finalPath;
  curPoint = endingPoint;
  finalPath.push_back(curPoint);
  while (curPoint != startingPoint) {
    curPoint = path[curPoint];
    finalPath.push_back(curPoint);
  }

  shortestPath.clear();
  points.points.clear();
  points.id = plotId;

  for(int i = finalPath.size()-1; i >= 1; i--)
  {
    p.x = nodesX[finalPath[i]];
    p.y = nodesY[finalPath[i]];
    p.z = 0.0;
    points.points.push_back(p);

    p.x = nodesX[finalPath[i-1]];
    p.y = nodesY[finalPath[i-1]];
    p.z = 0.0;
    points.points.push_back(p);

    shortestPath.push_back(finalPath[i]);
  }

  shortestPath.push_back(finalPath[0]);
  marker_pub.publish(points); 
  plotId++;
  pointId = 1;
}

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    //This function is called when a new map is received
    if (!foundMap)
  {
    std::vector<signed char> map_vector = msg.data;
    nav_msgs::MapMetaData info = msg.info;
    mapWidth = info.width;
    mapHeight = info.height;
    mapRes = info.resolution;
    int8_t* p = map_vector.data();
    mapData = new int8_t[mapWidth*mapHeight];
    for (int i = 0; i < mapWidth*mapHeight; i++)
      mapData[i] = p[i];
    ROS_INFO("Found Map of size %u x %u, %f, %lu", mapWidth, mapHeight, mapRes, map_vector.size());
    foundMap = 1;
    generateNodes();
    plotPoints();
    sleep(2);
  }
}

void carrot()
{
  int ind2 = shortestPath[pointId];
  int ind1 = shortestPath[pointId-1];

  ROS_INFO("carrot %d", ind1);
  ROS_INFO("%d", ind2);

  double x1 = nodesX[ind1]-OFFSET_X;
  double y1 = nodesY[ind1]-OFFSET_Y;
  double x2 = nodesX[ind2]-OFFSET_X;
  double y2 = nodesY[ind2]-OFFSET_Y;

  double dx = x2-x1;
  double dy = y2-y1;
  ROS_INFO("x1, x2, y1, y2  %f %f %f %f", x1, x2, y1, y2);

  double closeX = 0.0;
  double closeY = 0.0;
  double angle = atan2(dy, dx);
  double goalX = 0.0;
  double goalY = 0.0;
  double ld = 0.3;
  double ldDist;
  double remainingDist;

  double p = ((ipsX-x1)*dx+(ipsY-y1)*dy)/(dx*dx+dy*dy);
  if (p >= 0 && p <= 1)
  {
    closeX = x1+p*dx;
    closeY = y1+p*dy;
  }
  else if (p < 0)
  {
    closeX = x1;
    closeY = y1;
  }
  else
  {
    closeX = x2;
    closeY = y2;
  }

  goalX = closeX+cos(angle)*ld;
  goalY = closeY+sin(angle)*ld;
  

  ldDist = getNormDistPoints(closeX, closeY, goalX, goalY);
  remainingDist = getNormDistPoints(closeX, closeY, x2, y2);

  if (ldDist > remainingDist)
  {
    pointId++;
    if (pointId == shortestPath.size()) {
      foundMap++;
    } else {
      carrot();
    }
  }
  else
  {
    carrotAng = atan2((goalY-ipsY), (goalX-ipsX));
    ROS_INFO("Carrot ang 1  %f", carrotAng);

    carrotAng -= ipsYaw;
    ROS_INFO("Carrot ang 1  %f", carrotAng);

    if (carrotAng < -M_PI) {
      carrotAng += 2*M_PI;
    } else if (carrotAng > M_PI) {
      carrotAng -= 2*M_PI;
    }

    carrotAng *= KP;

    ROS_INFO("%f %f %f %d", goalX, goalY, carrotAng, pointId);
  }
}

int main(int argc, char **argv)
{
  //Initialize the ROS framework
  ros::init(argc,argv,"main_control");
  ros::NodeHandle n;
  bool turning = false;
  //Subscribe to the desired topics and assign callbacks
  ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
  // ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
  ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);


  //Setup topics to Publish from this node
  ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

  //Velocity control variable
  geometry_msgs::Twist vel;

  //Set the loop rate
  ros::Rate loop_rate(20);    //20Hz update rate


  while (ros::ok())
  {
  	loop_rate.sleep(); //Maintain the loop rate
  	ros::spinOnce();   //Check for new messages

    if (!foundMap)
      ROS_INFO("Waiting for map..");
    vel.linear.x = 0.0; // set linear speed
    if (foundMap == 1 && !std::isnan(initX) && !std::isnan(initY))
    {
      removeCollisionNodes();
      plotPoints();
      connectEdges();
      foundMap++;
      plotLines();

    }
    if (foundMap > 1)
    {
      if (foundMap == 2)
      {
        aStar(numNodes-4, numNodes-3);
        ROS_INFO("function called..");
        foundMap++;
      }
      if (foundMap == 4)
      {
        ROS_INFO("function2 called..");
        aStar(numNodes-3, numNodes-2);
        foundMap++;
      }
      if (foundMap == 6)
      {
        aStar(numNodes-2, numNodes-1);
        foundMap++;
      }
      if (foundMap == 3 || foundMap == 5 || foundMap == 7)
      {
        carrot();
      }
      if(foundMap == 8)
      {
        ROS_INFO("Finished Waypoints.");
        break;
      }
    }

    	//Main loop code goes here:

      if (fabs(carrotAng) > 0.05) {
        turning = true;
      } else {
        turning = false;
      }

      vel.angular.z = carrotAng; // set angular speed

      if (!turning) {
        vel.linear.x = 0.1;
      }

    	velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
