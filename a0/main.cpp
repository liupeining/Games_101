#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

#define _USE_MATH_DEFINES   //这里是加入了一些常见数学常量，见：https://blog.csdn.net/timidsmile/article/details/7322181

int main(){
    Eigen::Vector3d p(2.0f,1.0f,1.0f);
    Eigen::Matrix3d rotate;
    Eigen::Matrix3d trans;
    double theta = 45.0/180.0*M_PI;
    rotate << cos(theta), -1.0*sin(theta), 0,
            sin(theta), cos(theta),      0, 
            0,          0,               1;
    trans << 1, 0, 1, 
            0, 1, 2, 
            0, 0, 1;
    p = trans * rotate * p;
    std::cout << p << std::endl;
    return 0;
}