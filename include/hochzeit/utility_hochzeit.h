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
 * @param[in] toSeperate Der String, der nach 'seperator' aufgeteilt werden soll
 * @param[in] seperator Trennzeichen
 * 
 * @note In dieser Funktion sollte *NICHTS VERÄNDERT* werden
 */
std::vector<std::string>& seperateString(std::string toSeperate, const char *seperator);


/**
 * @brief Funktion, um die aktuellen Soll-Werte aus einer Stringmessage als double-Werte zu speichern
 * 
 * @param[in] msg Beinhaltet die x, y und theta-Werte als String
 * 
 * @note In dieser Funktion sollte *NICHTS VERÄNDERT* werden. Die Funktion dient dazu, um die gepublished-en Sollwerte aus der 
 * Trajektorienplanung in die einzelnen double-Werte zu zerlegen
*/
void updateString(const std_msgs::String::ConstPtr& msg);


/**
 * @brief Funktion, um die Variable Ende zu aktualisieren und in einer Stringmessage zu speichern. 
 * Dies wird benötigt, um den Regler zu stoppen nachdem die Trajketorie zu ende ist.
 * 
 * @param[in] msg String, in dem die Variable 'ende' steht
 * 
 * @note In dieser Funktion sollte *NICHTS VERÄNDERT* werden. Die Funktion dient dazu, um das gepublished-te 'ende' aus der 
 * Trajektorienplanung zu aktualisieren
*/
void updateEnde(const std_msgs::String::ConstPtr& msg);


/**
 * @brief Funktion, um die aktuelle Ist-Position des Leader-Roboters aus einer Pose-message zu speichern und zu aktualisieren
 * 
 * @param[in] msg Pose-Geometrymessage, in dem die Ist-Werte des Leader-Roboters stehen (x, y und theta)
 * 
 * @note In dieser Funktion sollte *NICHTS VERÄNDERT* werden. Die Funktion dient dazu, um die Ist-Werte vom
 * Leader-Roboter selbst als Ist-Wert-Grundlage für den Regler zu nutzen
*/
void callback_robot1pose(const unicyclesim::Pose::ConstPtr& msg);