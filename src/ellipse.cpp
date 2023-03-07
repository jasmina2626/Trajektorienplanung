#pragma once


#include <iostream>
#include <cmath>



#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "math.h"

#include "unicyclesim/Spawn.h"
#include "unicyclesim/Kill.h"
#include "unicyclesim/SetPen.h"
#include "unicyclesim/Pose.h"
#include <sstream>




double degreeToRad(double angle){
    double _angle = (angle/360) * 2*M_PI;
    return _angle;
}
double radToDegree(double angle){
    double _angle = (angle*360)/(M_PI*2);
    return _angle;
}
double integral(double(*f)(double x, double y), double(*g)(double x), double(*h)(double x), double a, double b, int n, double a_ellipse, double b_ellipse)
{
    double step = (b - a)/n; // width of rectangle
    double area = 0.0;
    double y = 0;  // height of rectangle

    for(int i = 0; i < n; ++i)
    {
        double x = a + (i + 0.5) * step;  //Integrationsvariable
        y = f(((f(g(x),2) * b_ellipse * b_ellipse) + (a_ellipse * a_ellipse * f(h(x),2))), 0.5);

        area += y * step; // find the area of the rectangle and add it to the previous area. Effectively summing up the area under the curve.
    }

    return area;
}

double segmentEllipse(const double& a, const double& b, const int& angle, const double& steps) {

    if(a <= b) {std::cout << "ERROR! a <= b" << std::endl; return 1;}

    double segment = 0, umfangGesamt = 2 * a * M_PI, epsilonQuadrat = 1 - ((b*b)/(a*a)), i = 1, j = 1;
    double ProduktzeichenAlt = 0.5, Produktzeichen = 1, Summenzeichen = 0, umfangGesamtAlt = 0, winkelDiff = 0;

    //angle = wie viel Grad vom Bogen gefahren werden sollen --> MUSS IN FUNKTION IN GRAD ÜBERGEBEN WERDEN
    double angleRad = degreeToRad(angle);


    //Gesamtumfangsberechnung der Ellipse
    do{
        umfangGesamtAlt = umfangGesamt;
        ProduktzeichenAlt =  pow(((2*j - 1)/(2*j)), 2);
        Produktzeichen = Produktzeichen * ProduktzeichenAlt;
        Summenzeichen = 1/(2*i-1);

        umfangGesamt -= 2 * a * M_PI * Produktzeichen * Summenzeichen * pow(epsilonQuadrat,i);
        i++, j++;

    }while((umfangGesamtAlt-umfangGesamt) > 0.00001);



    //obere Grenze für Integral bestimmen
    double t = atan((a/b)*tan(angleRad));

    //Errechnen des Bogenmaßes rückwärts, für den II. und IV. Quadranten --> spiegelverkehrtes Ellipsensegment --> "von hinten drauf addieren"
    if(angle > 90 && angle < 180) {winkelDiff = angle - 90; winkelDiff = degreeToRad(90 - winkelDiff);}
    if(angle > 270 && angle < 360) {winkelDiff = angle - 270; winkelDiff = degreeToRad(90 - winkelDiff);}

    double tBW = atan((a/b)*tan(winkelDiff)); //obere Grenze für Integral bestimmen --> Backwards
    double segmentBW = integral(std::pow, std::cos, std::sin, 0, tBW, steps, a, b);


    //Definitionslücken von tan() ausschliessen, also 90° und 270° --> Backwards nicht berechnen bei 90° und 270°
    //Integral bestimmen für "normalen" Segmentbogen
    if(angle != 90 && angle != 270) {
        segment = integral(std::pow, std::cos, std::sin, 0, t, steps, a, b);
        segmentBW =  (umfangGesamt/4) - segmentBW;
    }

    //Ausgleichsbedingungen für Definitionslücken von tan() und "Segmentbogenverlängerung"
    if(angleRad == M_PI/2 || (angleRad < M_PI && angleRad > M_PI/2)) segment = umfangGesamt/4 + segmentBW;// + segmentBackwards; //90° <= Winkel < 180°
    if(angleRad == M_PI || (angleRad < (3*M_PI)/2 && angleRad > M_PI)) segment += umfangGesamt/2; //180° <= Winkel < 270°
    if(angleRad == (3*M_PI)/2 || (angleRad < 2*M_PI && angleRad > (3*M_PI)/2)) segment = (3*umfangGesamt)/4 + segmentBW; //270° <= Winkel < 360°
    if(angleRad == 2*M_PI) segment = umfangGesamt;  //360° = Winkel

    //std::cout << "segmentEllipse: " << segment << std::endl;

    return segment;
}

double neededTime(double _v, double _l)
{
    double time = _l/_v;
    return time;
}



int ellipse(float _velocity, float _a, float _b, float _angle, bool _cw, float &x_start, float &y_start, float &theta_start, ros::Publisher &chatter_pub)
{
    double x = 0, y = 0, theta = 0, alpha = 0, l_UK = 0, l_IK = 0;
    float x_m = 0, y_m = 0;
    

    float thetaRad = degreeToRad(theta_start);
    
    
    double bogenlaenge = segmentEllipse(_a, _b, _angle, 1000);
    double time = neededTime(_velocity, bogenlaenge);
    double v_UK = 0, v_IK = 0;
    std_msgs::String msgString;

    x = _a * cos(thetaRad);
    y = _b * sin(thetaRad);
    
    std::cout << "x: " << x_start << "   y: " << y_start << "  phi: " << theta_start << std::endl;

    
    if (_cw)
    {
        alpha = radToDegree(atan((_a/_b)*tan(degreeToRad(theta_start + 90))));
    }
    else
    {
        alpha = radToDegree(atan((_a/_b)*tan(degreeToRad(theta_start - 90))));
    }


    theta_start = degreeToRad(theta_start);


    //Mittelpunktbestimmung
    if (_cw)
    {
        x_m = x_start + _a*cos(thetaRad-(M_PI/2));
        y_m = y_start + _a*sin(thetaRad-(M_PI/2));
    }
    else
    {
        x_m = x_start + _a*cos(thetaRad+(M_PI/2));
        y_m = y_start + _a*sin(thetaRad+(M_PI/2));
    }


    float current_angle = 0, angle_previous = 0;

    //_angle = wie weit die Ellipse gefahren werden soll
    float speed = _velocity * (_angle/bogenlaenge);            //Speed in °/sec to make circle  
    float angular_speed = speed * 2 * M_PI / 360;              //convert in radian/sec

    l_UK = M_PI * _a * (_angle/180);    //Bogenlänge des Umkreises für _angle
    l_IK = M_PI * _b * (_angle/180);    //Bogenlänge des Inkreises für _angle
    v_UK = l_UK / time;                 //Geschwindigkeit m/s vom Umkreis
    v_IK = l_IK / time;                 //Geschwindigkeit m/s vom Inkreis


    float speed_UK = (v_UK * (_angle/l_UK)) * 2 * M_PI / 360;    //Speed in rad/sec to make circle 
    float speed_IK = (v_IK * (_angle/l_IK)) * 2 * M_PI / 360;    //Speed in rad/sec to make circle 


    double x_start_calc = 0, y_start_calc = 0, x_start_calc_rotated = 0, y_start_calc_rotated = 0, x_start_calc_ohne_offset = 0, y_start_calc_ohne_offset = 0;
    double x_calc_ohne_offset = 0, y_calc_ohne_offset = 0;


    double t0, t1, m;  //m = Steigung an Tangente durch Punkt auf Ellipse
    t0 = t1 = ros::Time::now().toSec();


    int count = 0;
    float omega = 0;
    omega = thetaRad - M_PI/2;


    //While-Schleife, um Soll x, y und theta-Werte zu berechnen
    while(((time - (t1-t0)) >= 0) && ros::ok())
    {
        t1 = ros::Time::now().toSec();
        current_angle = speed_UK * (t1 - t0);  //entspricht aktuellem alpha --> für Um- und Inkreis identisch
        alpha = alpha + current_angle - angle_previous;

        if(y_calc_ohne_offset != 0) //y = 0 beachten
                m = - (((x_calc_ohne_offset) * _b * _b) / ((y_calc_ohne_offset) * _a * _a));


        //M_PI: genereller "Richtungswechsel" von Tangente (Winkelausgleichen)
        if((y_calc_ohne_offset) > 0) //1.Quadrant + 2. Quadrant
            theta = atan(m) + M_PI + omega;
        
        else if((x_calc_ohne_offset) < 0 && (y_calc_ohne_offset) < 0) //3.Quadrant --> +PI
            theta = atan(m) + 2*M_PI + omega;
        
        else if((x_calc_ohne_offset) > 0 && (y_calc_ohne_offset) < 0) //4.Quadrant --> passt wie es ist :) <3 (eigentlich -PI)
            theta = atan(m) + omega;



        /*
            1. x_start_calc_rotated ist die Start-x-Koordinate bei einer horizontalen ellipse ohne Offset (d.h. schon rotiert)
            2. von dort aus die Ellipse theoretisch starten und fahren lassen --> d.h. es braucht keinen Drehwinkel bzw. kein thetaRad --> x"soll" berechnet ohne Offset
            3. das Ergebnis wieder rotieren
            4. auf das Ergebnis dann den Offset (MP) addieren
        */

        if (_cw) //Im Uhrzeigersinn
          {            
            x_start_calc_ohne_offset = _a * cos(thetaRad + M_PI/2);
            y_start_calc_ohne_offset = _a * sin(thetaRad + M_PI/2);
                   
            //1.
            x_start_calc_rotated = x_start_calc_ohne_offset * sin(thetaRad + M_PI/2) + y_start_calc_ohne_offset * cos(thetaRad + M_PI/2);
            y_start_calc_rotated = x_start_calc_ohne_offset * sin(thetaRad + M_PI/2) - y_start_calc_ohne_offset * cos(thetaRad + M_PI/2);

            //2.
            x_calc_ohne_offset = _a * cos(-current_angle);
            y_calc_ohne_offset = _b * sin(-current_angle);
            
            //3.
            double x_rotated = x_calc_ohne_offset * cos(thetaRad + M_PI/2) - y_calc_ohne_offset * sin(thetaRad + M_PI/2);
            double y_rotated = x_calc_ohne_offset * sin(thetaRad + M_PI/2) + y_calc_ohne_offset * cos(thetaRad + M_PI/2);
            
            //4.
            x = x_rotated + x_m;
            y = y_rotated + y_m;
          }


        else //Gegen den Uhrzeigersinn
          { 
            x_start_calc_ohne_offset = _a * cos(thetaRad - M_PI/2);
            y_start_calc_ohne_offset = _a * sin(thetaRad - M_PI/2);
                
            //1.
            x_start_calc_rotated = x_start_calc_ohne_offset * sin(thetaRad - M_PI/2) + y_start_calc_ohne_offset * cos(thetaRad - M_PI/2);
            y_start_calc_rotated = x_start_calc_ohne_offset * sin(thetaRad - M_PI/2) - y_start_calc_ohne_offset * cos(thetaRad - M_PI/2);
                
            //2.
            x_calc_ohne_offset = _a * cos(current_angle);
            y_calc_ohne_offset = _b * sin(current_angle);
                
            //3.
            double x_rotated = x_calc_ohne_offset * cos(thetaRad - M_PI/2) - y_calc_ohne_offset * sin(thetaRad - M_PI/2);
            double y_rotated = x_calc_ohne_offset * sin(thetaRad - M_PI/2) + y_calc_ohne_offset * cos(thetaRad - M_PI/2);
                
            //4.
            x = x_rotated + x_m;
            y = y_rotated + y_m;
          }



        angle_previous = current_angle;


        //Soll-Positionen publishen
        std::stringstream ss;
        ss << x << "," << y << "," << theta;
        msgString.data = ss.str();
        chatter_pub.publish(msgString);






//Ausgabe der berechneten Position

        while((theta < 0) || (theta > 2*M_PI))
        {
            if(theta < 0)
                theta += (2*M_PI);
            if(theta > 2*M_PI)
                theta -= 2*M_PI;
        }
        

        if(count%10000 == 0)
            //std::cout << "x: " << x << "     y: " << y << "     theta: " << radToDegree(theta) << "     omega: " << radToDegree(omega) << std::endl;
        

        count++;


    }


    x_start_calc = x_m + _a * cos(thetaRad - M_PI/2);
    y_start_calc = y_m + _a * sin(thetaRad - M_PI/2);



    std::cout << "x: " << x << "     y: " << y << "     theta: " << radToDegree(theta) << std::endl;
    std::cout << "xM: " << x_m << "     yM: " << y_m  << std::endl;
    //std::cout << "x_start_calc: " << x_start_calc << "     y_start_calc: " << y_start_calc << std::endl;
    //std::cout << "x_start_calc_o_o: " << x_start_calc_ohne_offset << "     y_start_calc_o_o: " << y_start_calc_ohne_offset << std::endl;
    //std::cout << "x_start_calc_ro: " << x_start_calc_rotated << "     y_start_calc_ro: " << y_start_calc_rotated << std::endl;


    x_start = x;
    y_start = y;
    theta_start = radToDegree(theta);


    return 0;
}