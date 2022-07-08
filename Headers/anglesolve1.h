#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>


using namespace std;
using namespace cv;

class Angle_Solve
{
public:
    ~Angle_Solve(){}
    float* calAngle(Mat cam, Mat dis, int x, int y);
    float* calPnP(vector<Point3f> POINT_3D, vector<Point2f> point_2d_of_armor, Mat cam, Mat DISTORTION_COEFF);
    float* addStyle(float* angle_and_distance, int style);
    float* compensateOffset(float* angle_and_distance, float offset_z);
    float* compensateGravity(float* angle_and_distance, float v);
    vector<Point3f>POINT_3D_OF_ARMOR_SMALL = vector<Point3f>{
            //yi zhuang jia ban zhong xin wei yuan dian,zhuang jia ban si ge dian de zuo biao
            Point3f(-69,-30,0), //tl
            Point3f(69,-30,0),  //tr
            Point3f(69,30,0), //br
            Point3f(-69,30,0), //bl
    };
    vector<Point3f>POINT_3D_OF_ARMOR_BIG = vector<Point3f>{
            Point3f(-115,-30,0),
            Point3f(115,-30,0),
            Point3f(115,30,0),
            Point3f(-115,30,0),
    };
};

class CameraParam
{
private:
    Mat cameraMatrix;
    Mat distCoeffs;
public:
    CameraParam()
    {

//        //1号相机内参
//        cameraMatrix = Mat::eye(3, 3, CV_64F);
//        cameraMatrix.at<double>(0, 0) = 6.413591403442769e+02;
//        cameraMatrix.at<double>(0, 1) = -0.048682192679541;
//        cameraMatrix.at<double>(0, 2) = 3.267949571767471e+02;
//        cameraMatrix.at<double>(1, 0) = 0;
//        cameraMatrix.at<double>(1, 1) = 6.412273039484726e+02;
//        cameraMatrix.at<double>(1, 2) = 2.614315206641344e+02;
//        cameraMatrix.at<double>(2, 0) = 0;
//        cameraMatrix.at<double>(2, 1) = 0;
//        cameraMatrix.at<double>(2, 2) = 1;
//        //畸变参数
//        distCoeffs = Mat::zeros(5, 1, CV_64F);
//        distCoeffs.at<double>(0, 0) = -0.220134132391775;
//        distCoeffs.at<double>(1, 0) = 0.127717435745730;
//        distCoeffs.at<double>(2, 0) = -1.777718033791424e-04;
//        distCoeffs.at<double>(3, 0) = -0.001381219886486;
//        distCoeffs.at<double>(4, 0) = 0;


//        //2号相机内参
//        cameraMatrix = Mat::eye(3, 3, CV_64F);
//        cameraMatrix.at<double>(0, 0) = 6.374821166587172e+02;
//        cameraMatrix.at<double>(0, 1) = -0.144493405604357;
//        cameraMatrix.at<double>(0, 2) = 3.204027859741846e+02;
//        cameraMatrix.at<double>(1, 0) = 0;
//        cameraMatrix.at<double>(1, 1) = 6.371617721475170e+02;
//        cameraMatrix.at<double>(1, 2) = 2.522511246476694e+02;
//        cameraMatrix.at<double>(2, 0) = 0;
//        cameraMatrix.at<double>(2, 1) = 0;
//        cameraMatrix.at<double>(2, 2) = 1;
//        //畸变参数
//        distCoeffs = Mat::zeros(5, 1, CV_64F);
//        distCoeffs.at<double>(0, 0) = -0.220991024509454;
//        distCoeffs.at<double>(1, 0) = 0.174193498637808;
//        distCoeffs.at<double>(2, 0) = -8.445344864365196e-05;
//        distCoeffs.at<double>(3, 0) = -3.183642231601496e-04;
//        distCoeffs.at<double>(4, 0) = 0;


        //3号相机内参
        cameraMatrix = Mat::eye(3, 3, CV_64F);
        cameraMatrix.at<double>(0, 0) = 6.384643063828709e+02;
        cameraMatrix.at<double>(0, 1) = 0.184910695025243;
        cameraMatrix.at<double>(0, 2) = 3.148062465149410e+02;
        cameraMatrix.at<double>(1, 0) = 0;
        cameraMatrix.at<double>(1, 1) = 6.381573046914292e+02;
        cameraMatrix.at<double>(1, 2) = 2.695320974529934e+02;
        cameraMatrix.at<double>(2, 0) = 0;
        cameraMatrix.at<double>(2, 1) = 0;
        cameraMatrix.at<double>(2, 2) = 1;
        //畸变参数
        distCoeffs = Mat::zeros(5, 1, CV_64F);
        distCoeffs.at<double>(0, 0) = -0.223715388073907;
        distCoeffs.at<double>(1, 0) = 0.185379938015519;
        distCoeffs.at<double>(2, 0) = -7.194200516210393e-04;
        distCoeffs.at<double>(3, 0) = -5.862458087782471e-04;
        distCoeffs.at<double>(4, 0) = 0;


        //4号相机内参
        /*
        cameraMatrix = Mat::eye(3, 3, CV_64F);
        cameraMatrix.at<double>(0, 0) = 6.410394006850360e+02;
        cameraMatrix.at<double>(0, 1) = 0.038619665792993;
        cameraMatrix.at<double>(0, 2) = 3.288227884869464e+02;
        cameraMatrix.at<double>(1, 0) = 0;
        cameraMatrix.at<double>(1, 1) = 6.407833558147597e+02;
        cameraMatrix.at<double>(1, 2) = 2.608982753430882e+02;
        cameraMatrix.at<double>(2, 0) = 0;
        cameraMatrix.at<double>(2, 1) = 0;
        cameraMatrix.at<double>(2, 2) = 1;
        //畸变参数
        distCoeffs = Mat::zeros(5, 1, CV_64F);
        distCoeffs.at<double>(0, 0) = -0.213323895713259;
        distCoeffs.at<double>(1, 0) = 0.141325986713193;
        distCoeffs.at<double>(2, 0) = -6.736524348235457e-04;
        distCoeffs.at<double>(3, 0) = 0.001738858083073;
        distCoeffs.at<double>(4, 0) = 0;
        */
    }

    Mat getCameraMatrix()
    {
        return this->cameraMatrix;
    }

    Mat getDistCoeffs()
    {
        return this->distCoeffs;
    }
};

