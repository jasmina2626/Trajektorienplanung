#include "init_leader.h"


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