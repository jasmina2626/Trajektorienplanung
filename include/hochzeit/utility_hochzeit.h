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


/**
 * @brief Funktion, um einen String mit Hilfe von einem Seperator, in einzelne Teile zu zerlegen
 * 
 * @param[in] toSeperate Gibt die Start-x-Koordinate des Leader-Roboters vor --> an dieser x-Koordinate wird der Roboter platziert
 * @param[in] seperator Gibt die Start-y-Koordinate des Leader-Roboters vor --> an dieser y-Koordinate wird der Roboter platziert
 * 
 * @note In dieser Funktion sollte *NICHTS VERÄNDERT* werden
 */
std::vector<std::string>& seperateString(std::string toSeperate, const char *seperator);


/**
 * @brief Funktion, um die aktuellen Soll-Werte in einer stringmessage zu speichern
 * 
 * @param[in] msg Gibt die Start-x-Koordinate des Leader-Roboters vor --> an dieser x-Koordinate wird der Roboter platziert
 * 
 * @note In dieser Funktion sollte *NICHTS VERÄNDERT* werden, die Funktion dient dazu, um die gepublished-en Sollwerte aus der 
 * Trajektorienplanung in die einzelnen double-Werte zu zerlegen
*/
void updateString(const std_msgs::String::ConstPtr& msg);


/**
 * @brief Funktion, um den Leader-Roboter an der gewünschten Anfangsposition zu platzieren
 * 
 * @param[out] x_start Gibt die Start-x-Koordinate des Leader-Roboters vor --> an dieser x-Koordinate wird der Roboter platziert
 * @param[out] y_start Gibt die Start-y-Koordinate des Leader-Roboters vor --> an dieser y-Koordinate wird der Roboter platziert
 * @param[out] theta_start Gibt die Start-theta-Koordinate des Leader-Roboters vor --> mit diesem Ausrichtungswinkel wird der Roboter platziert
 * 
 * @note In dieser Funktion sollte *NICHTS VERÄNDERT* werden
*/
void updateEnde(const std_msgs::String::ConstPtr& msg);


/**
 * @brief Funktion, um den Leader-Roboter an der gewünschten Anfangsposition zu platzieren
 * 
 * @param[out] x_start Gibt die Start-x-Koordinate des Leader-Roboters vor --> an dieser x-Koordinate wird der Roboter platziert
 * @param[out] y_start Gibt die Start-y-Koordinate des Leader-Roboters vor --> an dieser y-Koordinate wird der Roboter platziert
 * @param[out] theta_start Gibt die Start-theta-Koordinate des Leader-Roboters vor --> mit diesem Ausrichtungswinkel wird der Roboter platziert
 * 
 * @note In dieser Funktion sollte *NICHTS VERÄNDERT* werden
*/
void callback_robot1pose(const unicyclesim::Pose::ConstPtr& msg);