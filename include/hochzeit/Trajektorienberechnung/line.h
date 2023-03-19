#pragma once

#include "utility.h"


/**
 * @brief Funktion um Soll-Koordinaten von einer Linie zu berechnen
 * 
 * @param[in] _velocity [m/s] gibt die Geschwindigkeit in m/s an, mit der der Kreis gefahren werden soll
 * @param[in] _length [m] gibt die Länge der zu fahrenden Strecke an
 * @param[out] x_start übergibt die x-Anfangskoordinate der Trajektorie
 * @param[out] y_start übergibt die y-Anfangskoordinate der Trajektorie
 * @param[out] theta_start übergibt die theta-Anfangskoordinate der Trajektorie
 * @param[out] chatter_pub mit diesem Parameter werden die Koordinaten gepublished
 * 
 * @return Am Ende werden die aktuellen Positionsdaten (x, y und theta) als Referenz zurückgegeben
 * @note Der Publisher muss der selbe sein, wie der aus der main von 'leader_move_node.cpp'
*/
int line(float _velocity, float _length, float &x_start, float &y_start, float &theta_start, ros::Publisher &pub);