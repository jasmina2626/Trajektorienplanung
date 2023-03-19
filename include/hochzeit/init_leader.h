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

/**
 * @brief Funktion, um den Leader-Roboter an der gewünschten Anfangsposition zu platzieren
 * 
 * @param[out] x_start Gibt die Start-x-Koordinate des Leader-Roboters vor --> an dieser x-Koordinate wird der Roboter platziert
 * @param[out] y_start Gibt die Start-y-Koordinate des Leader-Roboters vor --> an dieser y-Koordinate wird der Roboter platziert
 * @param[out] theta_start Gibt die Start-theta-Koordinate des Leader-Roboters vor --> mit diesem Ausrichtungswinkel wird der Roboter platziert
 * 
 * @note In dieser Funktion sollte *NICHTS VERÄNDERT* werden
*/
int starts_init(float &x_start, float &y_start, float &theta_start);