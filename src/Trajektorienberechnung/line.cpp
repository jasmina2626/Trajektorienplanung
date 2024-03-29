#include "line.h"


int line(float _velocity, float _length, float &x_start, float &y_start, float &theta_start, ros::Publisher &pub)
{
  std::cout << "Line" << std::endl;
  std_msgs::String msgString;
  float time = _length/_velocity; //Streckenzeit berechnen

  double theta_start_rad = degreeToRad(theta_start); //Theta in Rad umrechnen
  float x_soll = x_start, y_soll = y_start; //x_soll und y_soll sind die Sollwerte um die Linie zu fahren
  double t0, t1; //Zeitvariablen --> t0 ist Startzeit

  t0 = t1 = ros::Time::now().toSec();

  while(((time - (t1-t0)) >= 0) && ros::ok()) //Schleife, um Soll-Koordinaten zu berechnen --> läuft solange, bis errechnete Umlaufzeit vorbei ist
    {
      x_soll = x_start + cos(theta_start_rad) * (t1-t0)*_velocity;
      y_soll = y_start + sin(theta_start_rad) * (t1-t0)*_velocity;      
      
      //Soll-Positionen publishen
      std::stringstream ss;
      ss << x_soll << "," << y_soll << "," << theta_start;
      msgString.data = ss.str();
      
      pub.publish(msgString);

//Zeitausgabe
      //std::cout << "time: " << (t1-t0) << std::endl;
      
      t1 = ros::Time::now().toSec();
    }

  //Benötigt, damit die nächste Funktion die aktuelle Soll-Position bekommt
  x_start = x_soll;
  y_start = y_soll;
  theta_start = radToDegree(theta_start_rad);

  return 0;  
}