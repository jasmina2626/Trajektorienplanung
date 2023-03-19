#pragma once


#include <iostream>
#include <cmath>



#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "math.h"

#include "unicyclesim/Spawn.h"
#include "unicyclesim/Kill.h"
#include "unicyclesim/SetPen.h"
#include "unicyclesim/Pose.h"
#include <sstream>



/**
 * @brief Funktion um von Grad in Rad umzuwandeln
 * 
 * @param[in] angle in Grad
 * @return Winkel in Rad
 * @note Zusatz
*/
double degreeToRad(double angle);

double radToDegree(double angle);

double integral(double(*f)(double x, double y), double(*g)(double x), double(*h)(double x), double a, double b, int n, double a_ellipse, double b_ellipse);

double segmentEllipse(const double& a, const double& b, const int& angle, const double& steps);

double neededTime(double _v, double _l);

int ellipse(float _velocity, float _a, float _b, float _angle, bool _cw, float &x_start, float &y_start, float &theta_start, ros::Publisher &chatter_pub);