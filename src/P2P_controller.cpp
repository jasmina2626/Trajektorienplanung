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



 float v(float kp_v, float xist, float yist, float trajecx, float trajecy)
 {
    //Aktuelle Entfernung zum Sollwert
    float dist = sqrt(pow(trajecx-xist,2)+pow(trajecy-yist,2));
    //P-Regler
    float v = kp_v*dist;

    return v;
    
 }

 float w(float kp_w, float xist, float yist, float thetaist, float trajecx, float trajecy)
 {
    //Sollwert für theta
    float w_theta = atan2(trajecy-yist, trajecx-xist);
    //Istwert für theta
    float ist_theta = thetaist;
    //Regelabweichung
    float e_theta = w_theta-ist_theta;
    //Macht dass der Winkel zwischen -pi und pi ist
    e_theta = atan2(sin(e_theta),cos(e_theta));
    //P-Regler
    float w = kp_w * (e_theta);

    return w;
 }