#include "utility_hochzeit.h"



geometry_msgs::Pose2D targetValLeader; //--> wird zum Sollwert von x, y und theta
std::vector<std::string> vecCurrent;
unicyclesim::Pose currentValLeader; //--> wird zum aktuellen Wert von x, y und theta
int ende = 1; //dient zum Beenden des Reglers nach Trajektorienende


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