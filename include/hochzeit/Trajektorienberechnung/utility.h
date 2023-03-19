#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "math.h"
#include "turtlesim/Pose.h"

#include "unicyclesim/Spawn.h"
#include "unicyclesim/Kill.h"
#include "unicyclesim/SetPen.h"
#include "unicyclesim/Pose.h"
#include <sstream>
#include <iostream>
#include <cmath>



/**
 * @brief Funktion um von Grad in Rad umzuwandeln
 * 
 * @param[in] angle in Grad
 * @return Winkel in Rad
*/
double degreeToRad(double angle);


/**
 * @brief Funktion um von Rad in Grad umzuwandeln
 * 
 * @param[in] angle in Rad
 * @return Winkel in Grad
*/
double radToDegree(double angle);


/**
 * @brief Funktion um aus einer vorgegebenen Geschwindigkeit und einer Strecke die Dauer zu berechnen, die benötigt wird, um die Strecke zu fahren
 * 
 * @param[in] _v in m/s
 * @param[in] _l in m
 * @return Dauer in Sekunden
*/
double neededTime(double _v, double _l);