#pragma once

#include "utility.h"



double integral(double(*f)(double x, double y), double(*g)(double x), double(*h)(double x), double a, double b, int n, double a_ellipse, double b_ellipse);

double segmentEllipse(const double& a, const double& b, const int& angle, const double& steps);

int ellipse(float _velocity, float _a, float _b, float _angle, bool _cw, float &x_start, float &y_start, float &theta_start, ros::Publisher &chatter_pub);