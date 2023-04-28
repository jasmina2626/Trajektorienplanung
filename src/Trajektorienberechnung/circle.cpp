#include "circle.h"


int circle(float _velocity, float _radius, float _angle, bool _cw, float &x_start, float &y_start, float &theta_start, ros::Publisher &pub)
{
  std::cout << "Circle" << std::endl;
  
  ros::Rate loop_rate(50);            //50 entrspricht 50Hz
  geometry_msgs::Pose2D msg;
  std_msgs::String msgString;
  
  double theta_start_rad = degreeToRad(theta_start); //Theta in Rad umrechnen
  
  
  //x_soll, y_soll und theta_soll sind die Sollwerte um den Kreis zu fahren!
  //x_m und y_m sind die Koordinaten des Kreismittelpunkts
  float x_soll = x_start, y_soll = y_start, theta_soll = theta_start_rad;
  float x_m = 0, y_m = 0;

  float current_angle = 0, previous_angle = 0;  //Benötigt, um die Winkeldifferenz in der while-Schleife zu berechnen
  double t0, t1; //Zeitvariablen --> t0 ist Startzeit


  double time = (M_PI * _radius * _angle) / (_velocity * 180);   //Berechnung der Umlaufzeit
  float circumference = (_angle/180) * M_PI * _radius;           //Berechnung des Umfangs in m in Abhängigkeit vom Radius
  
  //_angle = wie weit der Kreis gefahren werden soll
  float speed = _velocity * (_angle/circumference);            //Geschwindigkeit in °/s um Kreis zu fahren
  float angular_speed = speed * 2 * M_PI / 360;                //konvertieren in rad/s
  
  
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
   
    

  t0 = t1 = ros::Time::now().toSec();

  
    while(((time - (t1-t0)) >= 0) && ros::ok()) //Schleife, um Soll-Koordinaten zu berechnen --> läuft solange, bis errechnete Umlaufzeit vorbei ist
    {
        t1 = ros::Time::now().toSec();
        current_angle = angular_speed * (t1 - t0);

        if (_cw) //Im Uhrzeigersinn
          {
            theta_soll = theta_soll - current_angle + previous_angle;
            x_soll = _radius*cos(-current_angle + theta_start_rad + (M_PI/2)) + x_m;
            y_soll = _radius*sin(-current_angle + theta_start_rad + (M_PI/2)) + y_m;
          }
        else //Gegen den Uhrzeigersinn
          { 
            theta_soll = theta_soll + current_angle - previous_angle;
            x_soll = _radius*cos(current_angle + theta_start_rad - (M_PI/2)) + x_m;
            y_soll = _radius*sin(current_angle + theta_start_rad - (M_PI/2)) + y_m;

          }



        //Ausgleichsbedingungen, falls mehr als 360° gefahren wurden --> für schönere Ausgabe
        if(theta_soll < (-M_PI))
            theta_soll += (2*M_PI);
        if(theta_soll > M_PI)
            theta_soll -= 2*M_PI;


        //Soll-Positionen publishen
        std::stringstream ss;
        ss << x_soll << "," << y_soll << "," << theta_soll;
        msgString.data = ss.str();
        pub.publish(msgString);


//Ausgabemöglichkeiten
        //std::cout << "x_soll: " << x_soll << "    y_soll: " << y_soll << "    theta_soll: " << radToDegree(theta_soll) << std::endl;
        //std::cout << "time: " << (t1-t0) << std::endl;

        
        ros::spinOnce();
        loop_rate.sleep();

        previous_angle = current_angle;                
    }

    //Benötigt, damit die nächste Funktion die aktuelle Soll-Position bekommt
    x_start = x_soll;
    y_start = y_soll;
    theta_start = radToDegree(theta_soll);


    ros::spinOnce();
    loop_rate.sleep();

  
  return 0;
}