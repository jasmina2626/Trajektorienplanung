#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/String.h"
#include "unicyclesim/Pose.h"

#include <sstream>
#include <string>
#include <vector>
#include <cstring>
#include <math.h>

#include "init.cpp"
#include "li_controller.cpp"
#include "P2P_controller.cpp"
#include "linDinOut.cpp"



std::vector<std::string>& seperateString(std::string toSeperate, const char *seperator) {
    static std::vector<std::string> result;
    result.clear();
    char* dataAsPointer = &toSeperate[0];
    char* token = strtok(dataAsPointer, seperator);
    result.push_back(token);
    
    while(token != NULL) {
        token = strtok(NULL, seperator);
        if(token != NULL) result.push_back(token);
    }
    
    return result;
}

geometry_msgs::Pose2D targetValLeader; //--> wird zum Sollwert von x, y und theta
std::vector<std::string> vecCurrent;
unicyclesim::Pose currentValLeader; //--> wird zum aktuellen Wert von x, y und theta
int ende = 1; //dient zum beenden der while-Schleife unten


void updateString(const std_msgs::String::ConstPtr& msg)
{
  int count = 0;
  vecCurrent = seperateString(msg->data.c_str(), ",");

  targetValLeader.x = std::stof(vecCurrent[0]);
  targetValLeader.y = std::stof(vecCurrent[1]);
  targetValLeader.theta = std::stof(vecCurrent[2]);
}


void updateEnde(const std_msgs::String::ConstPtr& msg)
{
  ende = std::stoi(msg->data.c_str());
  std::cout << "ende = " << ende << std::endl;
}


void callback_robot1pose(const unicyclesim::Pose::ConstPtr& msg)
{
  currentValLeader.x = msg->x;
  currentValLeader.y = msg->y;
  currentValLeader.theta = msg->theta;
}


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "hochzeit_subscribe");
    

    // --------------------> Leader-Koordinaten festlegen zum Spawnen


    float x_leader = 0.884;
    float y_leader = 0.884;
    float theta_leader = (120/180.0)*PI; //in Rad angeben!


    // -------------------->  Ende



    starts_init(x_leader, y_leader, theta_leader);


    ros::NodeHandle node;
    ros::Subscriber subSoll = node.subscribe("soll_publish", 100, updateString);
    
  
    ros::Subscriber endeSoll = node.subscribe("soll_ende", 1, updateEnde);

    // subscribe pose data with type unicyclesim::Pose on topic "/robotX/pose" with queque size 1
    ros::Subscriber subPose1 = node.subscribe("/robot1/pose", 1, callback_robot1pose);

    // publish data with type geometry_msgs::Twist on topic "/robotX/cmd_vel" with queque size 1
    ros::Publisher pubTwistLeader = node.advertise<geometry_msgs::Twist>("/robot1/cmd_vel", 1);

    geometry_msgs::Twist twistLeader;
    ros::Rate loop_rate(50);

    int count = 1;
    float test = 0.1;

    
    
    //currentValLeader = Istposition Leader
    //targetValLeader = Sollposition Leader (kommt aus Trajektorienplanung)

    while (ros::ok() && ende) // while roscore is running
    {

        //Master-Regler
        //twistLeader.linear.x = leader(targetValLeader.theta, targetValLeader.x, targetValLeader.y, currentValLeader.x, currentValLeader.y, currentValLeader.theta, test, test, test, test, 1)[0];
        //twistLeader.angular.z = leader(targetValLeader.theta, targetValLeader.x, targetValLeader.y, currentValLeader.x, currentValLeader.y, currentValLeader.theta, test, test, test, test, 1)[1];
        
        
        //P2P-Regler
        twistLeader.linear.x = (v(10, currentValLeader.x, currentValLeader.y, targetValLeader.x, targetValLeader.y)) * ((float)(count*count)/1'000'000);
        twistLeader.angular.z = w(10, currentValLeader.x, currentValLeader.y, currentValLeader.theta, targetValLeader.x, targetValLeader.y) * ((float)(count*count)/1'000'000);


        //Li-Regler:
        //twistLeader.linear.x = vF(k1, k2, currentValLeader.x, targetValLeader.x, currentValLeader.y, targetValLeader.y, phi, ld, targetValLeader.theta, currentValLeader.theta, distance, twistLeader.angular.z, twistLeader.linear.x);
        //twistLeader.angular.z = wF(k1, k2, currentValLeader.x, targetValLeader.x, currentValLeader.y, targetValLeader.y, phi, ld, targetValLeader.theta, currentValLeader.theta, distance, twistLeader.angular.z, twistLeader.linear.x);


        pubTwistLeader.publish(twistLeader);


        if(count < 1000)
        {
            count++;  //Um die Ausgabe zu verringern in if-Bedingung oben && für sinnvolle Beschleunigung
        }


        ros::spinOnce();
        loop_rate.sleep();
    }
  
    std::cout << "Ist  --> x, y, theta: " << currentValLeader.x << ", " << currentValLeader.y << ", " << currentValLeader.theta << std::endl;


    return 0;
}