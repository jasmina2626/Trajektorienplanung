#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"

#include <iomanip>

#include <vector>
using namespace std;


/*
theta => Ausrichtungswinkel
dtheta => 1.Ableitung des Ausrichtungswinkel
ddtheta => 2.Ableitung des Ausrichtungswinkel
x => soll X-Werte der Trajektorie
y => soll Y-Werte der Trajektorie
dx => 1.Ableitung der soll X-Werte der Trajektorie
dy => 1.Ableitung der soll Y-Werte der Trajektorie
ddx => 2.Ableitung der soll X-Werte der Trajektorie
ddy => 2.Ableitung der soll Y-Werte der Trajektorie
kp1 => Einstellparameter für x-Werte
kp2 => Einstellparameter für y-Werte
kd1=> Einstellparameter für x-Werte
kd2 => Einstellparameter für y-Werte
T => Periodendauer
i_x => ist X-Werte der Trajektorie
i_y => ist Y-Werte der Trajektorie
i_dx => 1.Ableitung der ist X-Werte der Trajektorie
i_dy => 1.Ableitung der ist Y-Werte der Trajektorie
*/


float ableitung(float x, float x1);

vector<float> leader(float theta, float x, float y, float i_x, float i_y, float i_theta, float kp1, float kp2, float kd1, float kd2, float T);
