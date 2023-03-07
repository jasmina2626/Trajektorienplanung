#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"


float vF(float k1, float k2, float XF, float XL, float YF, float YL, float phid, float ld, float thetaL, float thetaF, float d, float wL, float vL)
{
    float lx = -(XL - XF - d * cos(thetaF)) * cos(thetaL) - (YL - YF - d * sin(thetaF)) * sin(thetaL);
    float ly = (XL - XF - d * cos(thetaF)) * sin(thetaL) - (YL - YF - d * sin(thetaF)) * cos(thetaL);

    float lxd = ld * cos(phid);
    float lyd = ld * sin(phid);

    float ex = lxd - lx;
    float ey = lyd - ly;
    float etheta = thetaF - thetaL;

    float f1 = -wL * ld * sin(phid) + vL;
    float f2 = wL * ld * cos(phid);

    float v_F = (k1 * ex + wL * ey + f1) * cos(etheta) - (-k2 * ey + wL* ex - f2) * sin(etheta);

    //std::cout << "x_soll: " << XL << "  x_ist: " << XF << std::endl;

    return v_F;
}

float wF(float k1, float k2, float XF, float XL, float YF, float YL, float phid, float ld, float thetaL, float thetaF, float d, float wL, float vL)
{
    float lx = -(XL - XF - d * cos(thetaF)) * cos(thetaL) - (YL - YF - d * sin(thetaF)) * sin(thetaL);
    float ly = (XL - XF - d * cos(thetaF)) * sin(thetaL) - (YL - YF - d * sin(thetaF)) * cos(thetaL);

    float lxd = ld * cos(phid);
    float lyd = ld * sin(phid);

    float ex = lxd - lx;
    float ey = lyd - ly;
    float etheta = thetaF - thetaL;

    float f1 = -wL * ld * sin(phid) + vL;
    float f2 = wL * ld * cos(phid);

    float w_F = (1/d) * ((-k1 * ex - wL * ey - f1) * sin(etheta) - (-k2 * ey + wL * ex - f2) * cos(etheta));

    return w_F;
}