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


#define CW 1                 //clockwise
#define CCW 0                //counterclockwise
#define QUARTER_CIRCLE 90
#define SEMICIRCLE 180
#define THREE_QUARTER_CIRCLE 270
#define FULL_CIRCLE 360

#define MAX_X 4.5
#define MAX_Y 4.5 

geometry_msgs::Pose2D current; // Current x, y, and theta 


double radToDegree(double _angle) {double angle = (_angle/(M_PI*2))*360; return angle;}
double degreeToRad(double _angle) {double angle = (_angle/360)*2*M_PI; return angle;}



int circleT(float _velocity, float _radius, float _angle, bool _cw, float &x_start, float &y_start, float &theta_start, ros::Publisher &chatter_pub)
{
  /*
  _angle [°] indicates the angle in degrees --> 180 corresponds to a semicircle 
  _radius [m] specifies the radius of the driven circle in meters
  _cw specifies whether to drive clockwise or counterclockwise
      clockwise = 1 CW
      counterclockwise = 0 CCW
  _velocity [m/sec] specifies the speed in m/sec at which the circle is to be run
  */

  std::cout << "Circle" << std::endl;
  
  ros::Rate loop_rate(50);            //10 corresponds 10Hz
  geometry_msgs::Pose2D msg;
  std_msgs::String msgString;
  
  double theta_start_rad = degreeToRad(theta_start); //Theta in Rad umrechnen
  
  
  //x_soll, y_soll sind die Sollwerte um den Kreis zu fahren!
  //x_m und y_m sind die Koordinaten des Kreismittelpunkts
  float x_soll = x_start, y_soll = y_start, theta_soll = theta_start_rad;
  float x_m = 0, y_m = 0;





  //NEUES VON WAGNER
  float x_ende = 0, y_ende = 0, theta_ende = 0, x1 = 0, x2 = 0; //Ausgleichsparameter --> wirkliches Ende wird an nächste FKT übergeben, Variablen für Ableitung




  double time = (M_PI * _radius * _angle) / (_velocity * 180);   //Calculate circulation time
  float circumference = (_angle/180) * M_PI * _radius;           //Calculate circumference in m as a function of radius
  
  //_angle = wie weit der Kreis gefahren werden soll
  float speed = _velocity * (_angle/circumference);            //Speed in °/sec to make circle  
  float angular_speed = speed * 2 * M_PI / 360;                  //convert in radian/sec
  float relative_angle = _angle * 2 * M_PI / 360;                //convert in radian/sec

  
  
  if (_cw) //Im Uhrzeigersinn
    {
      x_m = x_start + _radius*cos(theta_start_rad-(M_PI/2));
      y_m = y_start + _radius*sin(theta_start_rad-(M_PI/2));
    }
  else
    { 
      x_m = x_start + _radius*cos(theta_start_rad+(M_PI/2));
      y_m = y_start + _radius*sin(theta_start_rad+(M_PI/2));
    }
   
    

  float current_angle = 0, angle_preview = 0;

  double t0, t1;
  t0 = t1 = ros::Time::now().toSec();

  
    while(((time - (t1-t0)) >= 0) && ros::ok()) // >= (0.013-_velocity*0.0012))) -->  vorher: (current_angle < relative_angle) || 1) && 
    {
        t1 = ros::Time::now().toSec();
        current_angle = angular_speed * (t1 - t0);

        if (_cw) //Im Uhrzeigersinn
          {
            theta_soll = theta_soll - current_angle + angle_preview;
            x_soll = _radius*cos(-current_angle + theta_start_rad + (M_PI/2)) + x_m;
            y_soll = _radius*sin(-current_angle + theta_start_rad + (M_PI/2)) + y_m;
          }
        else //Gegen den Uhrzeigersinn
          { 
            theta_soll = theta_soll + current_angle - angle_preview;
            x_soll = _radius*cos(current_angle + theta_start_rad - (M_PI/2)) + x_m;
            y_soll = _radius*sin(current_angle + theta_start_rad - (M_PI/2)) + y_m;

          }

        
        //Soll-Positionen publishen
        std::stringstream ss;
        ss << x_soll << "," << y_soll << "," << theta_soll;
        msgString.data = ss.str();
        chatter_pub.publish(msgString);


        //std::cout << "x_soll: " << x_soll << "    y_soll: " << y_soll << "    theta_soll: " << radToDegree(theta_soll) << std::endl;
        //std::cout << "time: " << (t1-t0) << std::endl;

        
        
        ros::sM_PInOnce();
        loop_rate.sleep();

        //Ausgleichsbedingungen, falls mehr als 360° gefahren wurden
        //if(theta_soll < 0)
        //    theta_soll += (2*M_PI);
        //if(theta_soll > 2*M_PI)
        //    theta_soll -= 2*M_PI;

        if(theta_soll < (-M_PI))
            theta_soll += (2*M_PI);
        if(theta_soll > M_PI)
            theta_soll -= 2*M_PI;


        angle_preview = current_angle;
        

        //Abbruchbedingung für "Fahren außerhalb der gegebenen Fläche" ?
                
    }

    relative_angle = 0;

    //Benötigt, damit die nächste Funktion die aktuelle Soll-Position bekommt
    x_start = x_soll;
    y_start = y_soll;
    theta_start = radToDegree(theta_soll);
    
    
    //chatter_pub.publish(msgString);

    //Hier war Ausgleichbedingung für mehr als 360°    

    ros::sM_PInOnce();
    loop_rate.sleep();

    
  
  return 0;
}


int lineT(float _velocity, float _length, float &x_start, float &y_start, float &theta_start, ros::Publisher &pub)
{
  std::cout << "Line" << std::endl;
  std_msgs::String msgString;
  float time = _length/_velocity;

  double theta_start_rad = degreeToRad(theta_start); //Theta in Rad umrechnen
  float x_soll = x_start, y_soll = y_start;

  double t0, t1;
  t0 = t1 = ros::Time::now().toSec();

  while(((time - (t1-t0)) >= 0) && ros::ok())
    {
      x_soll = x_start + cos(theta_start_rad) * (t1-t0)*_velocity;
      y_soll = y_start + sin(theta_start_rad) * (t1-t0)*_velocity;      
      
      //Soll-Positionen publishen
      std::stringstream ss;
      ss << x_soll << "," << y_soll << "," << theta_start;
      msgString.data = ss.str();
      
      pub.publish(msgString);


      //std::cout << "time: " << (t1-t0) << std::endl;
      t1 = ros::Time::now().toSec();
    }

  //Benötigt, damit die nächste Funktion die aktuelle Soll-Position bekommt
  x_start = x_soll;
  y_start = y_soll;
  theta_start = radToDegree(theta_start_rad);

  return 0;  
}