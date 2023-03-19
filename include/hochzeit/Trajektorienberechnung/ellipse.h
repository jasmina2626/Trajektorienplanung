#pragma once

#include "utility.h"


/**
 * @brief Integralfunktion (nummerisch) abgestimmt auf Segmentbogenberechnung für Ellipsen
 * 
 * @param[out] f Zeiger auf Wurzelfunktion
 * @param[out] g Zeiger auf cos-Funktion
 * @param[out] h Zeiger auf sin-Funktion
 * @param[in] a untere Integralgrenze --> i.d.R immer gleich 0
 * @param[in] b obere Integralgrenze
 * @param[in] n Schrittanzahl --> i.d.R aus segmentEllipse() übernommen
 * @param[in] a_ellipse [m] Länge vom Mittelpunkt aus zur Hauptachse (x-Achse --> Achse kann auch rotiert sein) --> i.d.R aus ellipse() übernommen
 * @param[in] b_ellipse [m] Länge vom Mittelpunkt aus zur Hauptachse (y-Achse --> Achse kann auch rotiert sein) --> i.d.R aus ellipse() übernommen

 * 
 * @return Gibt die Fläche unter Graphen zurück
 * @note In dieser Funktion sollte *NICHTS VERÄNDERT* werden
*/
double integral(double(*f)(double x, double y), double(*g)(double x), double(*h)(double x), double a, double b, int n, double a_ellipse, double b_ellipse);


/**
 * @brief Funktion um Segmentbogen einer Ellipse zu berechnen
 * 
 * @param[out] a [m] Länge vom Mittelpunkt aus zur Hauptachse (x-Achse --> Achse kann auch rotiert sein) 
 * --> i.d.R aus ellipse() übernommen
 * @param[out] b [m] Länge vom Mittelpunkt aus zur Nebenachse (y-Achse --> Achse kann auch rotiert sein) --> muss kleiner als a sein 
 * --> i.d.R aus ellipse() übernommen
 * @param[out] angle [°] gibt den zu fahrenden Winkel in Grad an --> 180 entspricht einem Halbkreis
 * @param[out] steps gibt die Schrittanzahl für Integralfunktion an (je größer, desto genauer das Ergebnis)

 * 
 * @return Gibt die Segmentbogenlänge der Ellipse in m zurück
 * @note In dieser Funktion sollte *NICHTS VERÄNDERT* werden, steps wird nur für Integral-Funktion benötigt 
 * --> damit wird die Genauigkeit der numerischen Integration bestimmt
*/
double segmentEllipse(const double& a, const double& b, const int& angle, const double& steps);


/**
 * @brief Funktion um Soll-Koordinaten von einer Ellipse zu berechnen
 * 
 * @param[in] _velocity [m/s] gibt die Geschwindigkeit in m/s an, mit der der Kreis gefahren werden soll
 * @param[in] _a [m] Länge vom Mittelpunkt aus zur Hauptachse (x)
 * @param[in] _b [m] Länge vom Mittelpunkt aus zur Nebenachse (y) --> muss kleiner als _a sein
 * @param[in] _angle [°] gibt den zu fahrenden Winkel in Grad an --> 180 entspricht einem Halbkreis
 * @param[in] _cw gibt an, ob im oder gegen den Uhrzeigersinn gefahren werden soll [im Uhrzeigersinn = true, Gegen den Uhrzeigersinn = false]
 * @param[out] x_start übergibt die x-Anfangskoordinate der Trajektorie
 * @param[out] y_start übergibt die y-Anfangskoordinate der Trajektorie
 * @param[out] theta_start übergibt die theta-Anfangskoordinate der Trajektorie
 * @param[out] chatter_pub mit diesem Parameter werden die Koordinaten gepublished
 * 
 * @return Am Ende werden die aktuellen Positionsdaten (x, y und theta) als Referenz zurückgegeben
 * @note Der Publisher muss der selbe sein, wie der aus der main von 'leader_move_node.cpp'
*/
int ellipse(float _velocity, float _a, float _b, float _angle, bool _cw, float &x_start, float &y_start, float &theta_start, ros::Publisher &chatter_pub);