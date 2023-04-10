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
#include <cmath>

#include "utility.h"
//#include "utility_hochzeit.h"
#include "init_leader.h"
#include "P2P_controller.h"
#include "linDinOut.h"


//Hier befindet sich die Regelung für den Leader-Roboter
//Außerdem gibt man hier die Start-Koordinaten für den Leader-Roboter vor
//Der Ausführung zum spawnen des Leader-Roboters findet man in "init_leader.h"


//currentValLeader = Istposition Leader
//targetValLeader = Sollposition Leader (kommt aus Trajektorienplanung)
geometry_msgs::Pose2D targetValLeader;
std::vector<std::string> vecCurrent;
unicyclesim::Pose currentValLeader;
int ende = 1; //dient zum Beenden des Reglers nach Trajektorienende


std::vector<std::string>& seperateString(std::string toSeperate, const char *seperator);
void updateString(const std_msgs::String::ConstPtr& msg);
void updateEnde(const std_msgs::String::ConstPtr& msg);
void callback_robot1pose(const unicyclesim::Pose::ConstPtr& msg);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "hochzeit_subscribe");
    
    

    // --------------------> Leader-Koordinaten festlegen zum Spawnen

    float x_leader = 1;
    float y_leader = 3;
    float theta_leader = degreeToRad(90); //in Grad angeben!

    // -------------------->  Ende der Koordinatenplanung


    starts_init(x_leader, y_leader, theta_leader); //Spawnen des Leader-Roboters mit den oben angegebenen Koordinaten


    ros::NodeHandle node;

    //subscriben der Soll-Position für den Leader-Roboter
    ros::Subscriber subSoll = node.subscribe("soll_publish", 100, updateString);

    //subscriben des Trajektorien-Endes
    ros::Subscriber endeSoll = node.subscribe("soll_ende", 1, updateEnde);

    //subscribe der Ist-Position des Leader-Roboters
    ros::Subscriber subPose1 = node.subscribe("/robot1/pose", 1, callback_robot1pose);

    //publishen der errechneten linearen und angularen Geschwindigkeit des Leader-Roboters
    ros::Publisher pubTwistLeader = node.advertise<geometry_msgs::Twist>("/robot1/cmd_vel", 1);

    geometry_msgs::Twist twistLeader;
    ros::Rate loop_rate(50);

    int count = 1;
    float test = 0.1;
  

    while (ros::ok() && ende) // while roscore is running
    {

        //LinDinOut-Regler
        //twistLeader.linear.x = leader(targetValLeader.theta, targetValLeader.x, targetValLeader.y, currentValLeader.x, currentValLeader.y, currentValLeader.theta, test, test, test, test, 1)[0];
        //twistLeader.angular.z = leader(targetValLeader.theta, targetValLeader.x, targetValLeader.y, currentValLeader.x, currentValLeader.y, currentValLeader.theta, test, test, test, test, 1)[1];
        
        
        //P2P-Regler
        twistLeader.linear.x = (v(10, currentValLeader.x, currentValLeader.y, targetValLeader.x, targetValLeader.y)) * ((float)(count*count)/1'000'000);
        twistLeader.angular.z = w(10, currentValLeader.x, currentValLeader.y, currentValLeader.theta, targetValLeader.x, targetValLeader.y) * ((float)(count*count)/1'000'000);

        pubTwistLeader.publish(twistLeader);


        if(count < 1000)
        {
            count++;  //Um die Ausgabe zu verringern in if-Bedingung oben && für sinnvolle Beschleunigung
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
  
    std::cout << "Ist  --> x, y, theta: " << currentValLeader.x << ", " << currentValLeader.y << ", " << radToDegree(currentValLeader.theta) << std::endl;

    return 0;
}


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