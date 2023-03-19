#pragma once

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
#include "init_leader.h"
#include "P2P_controller.h"
#include "linDinOut.h"


std::vector<std::string>& seperateString(std::string toSeperate, const char *seperator);


void updateString(const std_msgs::String::ConstPtr& msg);


void updateEnde(const std_msgs::String::ConstPtr& msg);


void callback_robot1pose(const unicyclesim::Pose::ConstPtr& msg);