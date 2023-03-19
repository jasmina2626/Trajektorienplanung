#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"

/*
v: Lineargeschwindigkeit
w: Winkelgeschwindigkeit
kp_v: Verstärkungsfaktor v
kp_w: Verstärkungsfaktor w
xist: x-Ist-Position 
yist: y-Ist-Position
thetaist: theta-Ist-Position
trajecx: x-Soll-Position aus Trajektorie
trajecy: y-Soll-Position aus Trajektorie

*/



 float v(float kp_v, float xist, float yist, float trajecx, float trajecy);

 float w(float kp_w, float xist, float yist, float thetaist, float trajecx, float trajecy);