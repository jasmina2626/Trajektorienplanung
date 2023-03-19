#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <stdio.h>

#include "trajec.h"


#include <string>
#include <sstream>


//Hier ist die main() um die Trajektorie des Leaders zu planen
//In der while-Schleife kann man verschiedene Funktionen aufrufen
//Die Funktionen die man aufrufen kann findet man in "trajec.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "soll_publisher");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::String>("soll_publish", 1000);
    ros::Publisher ende = n.advertise<std_msgs::String>("soll_ende", 10, true);
    
    ros::Rate loop_rate(50);
    
    geometry_msgs::Pose2D msg;
    std_msgs::String msg2, msgEnde;
    
    int end = 1;
    double t0, t1;

    msgEnde.data = "1";
    ende.publish(msgEnde);
    std::cout << "Trajec" << std::endl;
    


    //init Trajektorien-Start
    float x = 0.884;
    float y = 0.884; 
    float theta = 120; //In GRAD angeben!!

    while (ros::ok() && end)
    {
        
        //In Funktionen MUSS der selbe ros::Publisher übergeben werden, damit die berechneten Werte subscribed werden!




        // ------------------> Hier die Trajektorie planen ---> msg Daten als Referenz übergeben!!

        //circle(0.1, 0.5, 180, false, x, y, theta, pub);
        //line(0.1, 1, x, y, theta, pub);
        //circle(0.1, 0.5, 180, false, x, y, theta, pub);
        //line(0.1, 1, x, y, theta, pub);

        //ellipse(0.1, 1.25, 0.5, 360, false, x, y, theta, pub);
        ellipse(0.1, 1.25, 0.5, 360, true, x, y, theta, pub);



        // -----------------> Trajektorienplanung ENDE



        msgEnde.data = "0";
        std::cout << "\nsende Ende";
        t0 = t1 = ros::Time::now().toSec();


        //While-Schleife, um Trajektorien-Ende zu senden
        while(((1.5 - (t1-t0)) >= 0) && ros::ok())
        {  
            t1 = ros::Time::now().toSec();
            ende.publish(msgEnde);
        }
        
        std::cout << " ---> Ende gesendet" << std::endl;


        ros::spinOnce();
        loop_rate.sleep();


        end = 0;
    }

    return 0;
}
