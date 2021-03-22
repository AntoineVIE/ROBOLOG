#include  <ros/ros.h>
#include  <turtlesim/Pose.h>
#include  <turtlesim/SetPen.h>
#include  <geometry_msgs/Twist.h>
#include  <std_srvs/Empty.h>
#include  <math.h>         //  define  atan2 , sqrt  and  pow
#include  <stdlib.h>      //  define  rand
#include  <iostream>      //  define  cout

//  define  pi as a const  flaot (better  idea  than  using a#define)
const  float  pi = 3.14159265;

//  define  turtle  current  pose  and  goal as  global variable  for  simplicity
turtlesim ::Pose  currentpose;
turtlesim ::Pose  goalpose;

float currentacc[2];


//  construct  Twist  message  and  publish  it
void  setturtlevelocities(ros:: Publisher  twist_pub , float linear , float  angular)
{
	// at  beginning  of a function  it"s a good  idea to send a message , it can  help to debug  code
	std::cout  << "set  turtle  velocity  to x = " << linear<< " rotz = " << angular  << "\n";

	//  create  an  instance  of  geometry_msgs :: Twist  messagetype (it"s a structure)
	geometry_msgs ::Twist  twist;
	// and  fill  usefull  field
	twist.linear.x = linear;
	twist.angular.z = angular;

	//  publish  the  message  on the  topic
	twist_pub.publish(twist);
}

//  callback  function: deal  with  the  turtle1/posesubscription
void  getturtleposecallback(const  turtlesim ::Pose& pose)
{
	std::cout  << "turtle  current  pose:\n" << pose  << "\n";

	//  assigne  the  turtle  position (from  message) to the global  variable  currentpose
	currentpose = pose;
}

//  callback  function: deal  with  the  turtle1/goalsubscription
void  getturtlegoalcallback(const  turtlesim ::Pose& goal)
{
	std::cout  << "turtle  goal:\n" << goal  << "\n";

	//  assigne  the  goal  position (from  message) to theglobal  variable  goalpose
	goalpose = goal;
}

void getturtleacccallback(const std_msgs::Float32MultiArray& msg){
	std::cout  << "turtle  acc:\n" << msg.data[0]  << "\n";

	//  assigne  the  goal  position (from  message) to theglobal  variable  goalpose
	currentacc[0] = msg.data[0];
	currentacc[1] = msg.data[1];
}
//  controller  to  create
void  turtlecontroller(ros:: Publisher  twist_pub)
{
	//  inputs
	float  turtle_x = currentpose.x;
	float  turtle_y = currentpose.y;
	float  goal_x = goalpose.x;
	float  goal_y = goalpose.y;
	//  outputs
	float  forward_velocity = 0.0;
	float  angular_velocity = 0.0;
	
	float dt = 0.01;
	//  write  turtle  controller  here
	
	float  dx = goal_x - turtle_x;
	float  dy = goal_y - turtle_y;
	float goal_theta = atan2(dy,dx);
	float theta = 0;
	
	angular_velocity = currentacc[0];
	forward_velocity = currentacc[1];
	setturtlevelocities(twist_pub, forward_velocity, angular_velocity); 

}

// main  function  is the  starting  point of the written programme
int  main(int argc , char** argv)
{
	// set  initial  goal  position
	goalpose.x = 8;
	goalpose.y = 8;

	//  initialise  ros
	ros::init(argc , argv , "auto_draw");
	// and  create a handle
	ros:: NodeHandle  nh;

	//  define  refresh  rate: 10 hz
	ros::Rate r(10);

	//  Subscription  to  turtle1/pose  topic
	//  function  getturtleposecallback  deal  with it
	//  message  queue is set to 1 message
	ros:: Subscriber  pose_sub = nh.subscribe("turtle1/pose", 1, getturtleposecallback);
	ros:: Subscriber  acc_sub = nh.subscribe("chatter", 1, getturtleacccallback);
	//  Subscription  to  turtledriver/goal  topic
	//  function  getturtlegoalcallback  deal  with it
	//  message  queue is set to 1 message
	ros:: Subscriber  goal_sub = nh.subscribe("turtledriver/goal", 1, getturtlegoalcallback);

	//  Publishing  on  turtle1/cmd_vel  topic
	// with  message  type: geometry_msgs :: Twist
	//  maximum 1 message  in the  queue
	ros:: Publisher  twist_pub = nh.advertise <geometry_msgs::Twist >("turtle1/cmd_vel", 1);

	//  create a client  to  reset  service
	//  reset  service  message  type: std_srvs :: Empty
	ros:: ServiceClient  resetclient = nh.serviceClient <std_srvs ::Empty >("reset");
	resetclient.waitForExistence ();
	//  create  an  instance  of  std_srvs :: Empty  message  type
	std_srvs ::Empty  resetservice;
	// call  reset  service  with  std_srvs :: Empty  messagetype
	resetclient.call(resetservice);

	//  create  one  client  to  reset  serviceClient
	//  reset  service  message  type: turtlesim :: SetPen
	ros:: ServiceClient  penclient = nh.serviceClient <turtlesim ::SetPen >("turtle1/set_pen");
	penclient.waitForExistence ();
	//  create  an  instance  of  turtlesim :: SetPen  messagetype
	// fill it with  some  initial  values
	turtlesim :: SetPen  pen;
	pen.request.r = 255;
	pen.request.g = 255;
	pen.request.b = 255;
	pen.request.width = 5;
	pen.request.off = 0;

	while (ros::ok())
	{
	// fill  turtlesim :: SetPen  instance  with  somme  randomvalues
	pen.request.r = rand() %256;
	pen.request.g = rand() %256;
	pen.request.b = rand() %256;
	// call  setpen  service  with  filled  turtlesim :: SetPen
	penclient.call(pen);

	// call  the  controller  function
	turtlecontroller(twist_pub);

	// ros  send  and  receive  messages
	ros:: spinOnce ();

	// wait  some  time (defined  by rate  value)
	r.sleep ();
	}
}