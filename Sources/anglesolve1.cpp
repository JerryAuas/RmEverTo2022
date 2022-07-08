#include "anglesolve1.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

float* Angle_Solve::calAngle(Mat cam, Mat dis, int x, int y)
{
    //大符用
    float* angle_and_distance = new float[3];
    double fx = cam.at<double>(0, 0);
    double fy = cam.at<double>(1, 1);
    double cx = cam.at<double>(0, 2);
    double cy = cam.at<double>(1, 2);
    Point2f pnt;
    vector<cv::Point2f> in;
    vector<cv::Point2f> out;
    in.push_back(Point2f(x, y));
    //去除畸变
    undistortPoints(in, out, cam, dis, noArray(), cam);
    pnt = out.front();
    double rxNew = (pnt.x - cx) / fx;
    double ryNew = (pnt.y - cy) / fy;
    float yaw = atan(rxNew) / CV_PI * 180;
    float pitch = atan(ryNew) / CV_PI * 180;
    #ifdef ANGLE_DISPLAY
    cout << "yaw :" << yaw << endl;
    cout << "pitch :" << pitch << endl;
    #endif //ANGLE_DISPLAY
    angle_and_distance[0] = yaw;
    angle_and_distance[1] = pitch;
    angle_and_distance[2] = 666.7;
    return angle_and_distance;
}

float* Angle_Solve::calPnP(vector<Point3f> POINT_3D, vector<Point2f> point_2d_of_armor, Mat cam, Mat DISTORTION_COEFF)
{
    float* angle_and_distance = new float[3];
    Mat _rVec = Mat::zeros(3, 1, CV_64FC1);
    Mat _tVec = Mat::zeros(3, 1, CV_64FC1);
    float yaw2, pitch2, _euclideanDistance;
    solvePnP(POINT_3D, point_2d_of_armor, cam, DISTORTION_COEFF, _rVec, _tVec, false, CV_ITERATIVE);
    //_rVec: xuan zhuan
    //_tVec: ping yi
    yaw2 = atan(_tVec.at<double>(0, 0) / _tVec.at<double>(2, 0)) / 2 / CV_PI * 360;
    pitch2 = atan(_tVec.at<double>(1, 0) / _tVec.at<double>(2, 0)) / 2 / CV_PI * 360;
    _euclideanDistance = sqrt(_tVec.at<double>(0, 0)*_tVec.at<double>(0, 0) + _tVec.at<double>(1, 0)*_tVec.at<double>(1, 0) + _tVec.at<double>(2, 0)* _tVec.at<double>(2, 0));
    //角度下移右移正
    //cout << "yaw2 :" << yaw2 << endl;
    //cout << "pitch2 :" << pitch2 << endl;
    //cout << "Distance :" << _euclideanDistance << endl;
    //cout << "_tVec:" << _tVec << endl;
    angle_and_distance[0] = yaw2 +1;
    angle_and_distance[1] = pitch2 - 2;
    angle_and_distance[2] = _euclideanDistance;
    return angle_and_distance;
}

//shoot style of shaobing
float* Angle_Solve::addStyle(float* angle_and_distance, int style){
    float* angle_and_distance_and_style = new float[4];
    angle_and_distance_and_style[0] = angle_and_distance[0];
    angle_and_distance_and_style[1] = angle_and_distance[1];
    angle_and_distance_and_style[2] = angle_and_distance[2];
    angle_and_distance_and_style[3] = style;
    return angle_and_distance_and_style;
}

float* Angle_Solve::compensateOffset(float* angle_and_distance, float offset_z){
    float* O_angle_and_distance = new float[3];
    float theta_y = angle_and_distance[0] / 180 * CV_PI;
    float theta_p = angle_and_distance[1] / 180 * CV_PI;
    float theta_y_prime = atan(angle_and_distance[2] * sin(theta_y) / (angle_and_distance[2] * cos(theta_y) + offset_z));
    float theta_p_prime = atan(angle_and_distance[2] * sin(theta_p) / (angle_and_distance[2] * cos(theta_p) + offset_z));
    float d_prime = sqrt(pow(offset_z + angle_and_distance[2] * cos(theta_y), 2) + pow(angle_and_distance[2] * sin(theta_y), 2));
    float O_yaw = theta_y_prime / CV_PI * 180;
    float O_pitch = theta_p_prime / CV_PI * 180;
    O_angle_and_distance[0] = O_yaw;
    O_angle_and_distance[1] = O_pitch;
    O_angle_and_distance[2] = d_prime;
    return O_angle_and_distance;
}

float* Angle_Solve::compensateGravity(float* angle_and_distance, float v){
    float* G_angle_and_distance = new float[3];
    float theta_p = angle_and_distance[1] / 180 * CV_PI;
    float theta_p_prime = atan((sin(theta_p) - 0.5*9.8*angle_and_distance[2] / pow(v,2)) / cos(theta_p));
    float G_pitch = theta_p_prime / CV_PI * 180;
    G_angle_and_distance[0] = angle_and_distance[0];
    G_angle_and_distance[1] = G_pitch;
    G_angle_and_distance[2] = angle_and_distance[2];
    return G_angle_and_distance;
}
