#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "math.h"

#include "unicyclesim/Spawn.h"
#include "unicyclesim/Kill.h"
#include "unicyclesim/SetPen.h"



#include <sstream>

#define PI 3.14159265359

#define CW 1                 //clockwise
#define CCW 0                //counterclockwise
#define QUARTER_CIRCLE 90
#define SEMICIRCLE 180
#define THREE_QUARTER_CIRCLE 270
#define FULL_CIRCLE 360 


/*
int ends_init(int &end, ros::Publisher &chatter_pub)
{
  ros::Rate loop_rate(10);
  geometry_msgs::Twist msg;

  end = 0;

  msg.angular.z = 0;
  msg.linear.x = 0.01;
  chatter_pub.publish(msg);

  ros::spinOnce();
  loop_rate.sleep();

  return 0;
}
*/


int starts_init(float &x_start, float &y_start, float &theta_start)
{
  int count = 0;
  int duration = 2;

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("spawn_leader", 1000);
  ros::Rate loop_rate(50);

  ros::ServiceClient servSpawn = n.serviceClient<unicyclesim::Spawn>("/sim/spawn");
  ros::ServiceClient servKill = n.serviceClient<unicyclesim::Kill>("/sim/kill");
  ros::ServiceClient servPen = n.serviceClient<unicyclesim::SetPen>("/robot1/set_pen");


  unicyclesim::Kill killMsg;
  killMsg.request.name = "robot1";
  servKill.call(killMsg);

  unicyclesim::Spawn spawnMsg;
  spawnMsg.request.x = x_start;
  spawnMsg.request.y = y_start;
  spawnMsg.request.theta = theta_start;
  spawnMsg.request.name = "robot1";
  servSpawn.call(spawnMsg);

  unicyclesim::SetPen penMsg;
  penMsg.request.r = 255;
  penMsg.request.width = 3;
  servPen.call(penMsg);
  
  
  geometry_msgs::Twist msg;


  //wird das noch benötigt?
  while (ros::ok() && (count < duration))
  {
    if(count < duration)
      msg.linear.x = 1;
    else 
      msg.linear.x = 0;

    count++;

    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  
  }

  msg.linear.x = 0;
  pub.publish(msg);

  return 0;
}


/*
int spawn_robots_init(ros::NodeHandle &m_node)
{
  // client to create a new robot in unicyclesim
  ros::ServiceClient servSpawn2 = m_node.serviceClient<unicyclesim::Spawn>("/sim/spawn");

  // spawn a new robot named robot2
  unicyclesim::Spawn spawnMsg;
  spawnMsg.request.x = 2;
  spawnMsg.request.y = 2;
  spawnMsg.request.name == "robot2";
  servSpawn2.call(spawnMsg);

  ros::ServiceClient servSpawn3 = m_node.serviceClient<unicyclesim::Spawn>("/sim/spawn");

  // spawn a new turtle named robot3
  spawnMsg.request.x = 2;
  spawnMsg.request.y = 2;
  spawnMsg.request.name == "robot3";
  servSpawn3.call(spawnMsg);

  ros::ServiceClient servSpawn4 = m_node.serviceClient<unicyclesim::Spawn>("/sim/spawn");

  // spawn a new turtle named robot4
  spawnMsg.request.x = 2;
  spawnMsg.request.y = 2;
  spawnMsg.request.name == "robot4";
  servSpawn4.call(spawnMsg);


  ros::ServiceClient servPen2 = m_node.serviceClient<unicyclesim::SetPen>("/robot2/set_pen");
  ros::ServiceClient servPen3 = m_node.serviceClient<unicyclesim::SetPen>("/robot3/set_pen");
  ros::ServiceClient servPen4 = m_node.serviceClient<unicyclesim::SetPen>("/robot4/set_pen");
  unicyclesim::SetPen penMsg2, penMsg3, penMsg4;

  //robot2 = lila --- robot3 = grün --- robot4 = gelb
  
  penMsg2.request.r = 204;
  penMsg2.request.g = 153;
  penMsg2.request.b = 255;
  penMsg2.request.width = 3;
  servPen2.call(penMsg2);

  penMsg3.request.g = 255;
  penMsg3.request.width = 3;
  servPen3.call(penMsg3);

  penMsg4.request.r = 255;
  penMsg4.request.g = 255;
  penMsg4.request.width = 3;
  servPen4.call(penMsg4);


  // refresh rate in Hz
  ros::Rate loop_rate(50);

  return 0;
}
*/
