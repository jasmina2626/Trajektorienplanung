#include "utility.h"


double degreeToRad(double angle){
    double _angle = (angle/360) * 2*M_PI;
    return _angle;
}

double radToDegree(double angle){
    double _angle = (angle*360)/(M_PI*2);
    return _angle;
}

double neededTime(double _v, double _l)
{
    double time = _l/_v;
    return time;
}