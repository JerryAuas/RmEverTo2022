#pragma once

#include <math.h>
#include <sys/time.h>
#include <cmath>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>
#include <memory>
#include "anglesolve1.h"
#include "serial1.h"

#define DETECT_DEBUG
#define TIME_COUNT
#define ANGLE_DISPLAY
#define DETECT_DEBUG

using namespace cv;
using namespace std;
using namespace rm;

struct Frame
{
    Mat img;
    // size_t seq;
    double timeStamp;
};

class FrameBuffer
{
    public:
        ~FrameBuffer() = default;
        bool push(const Frame& aframe);
        bool getLatest(Mat& aframe);
        void init(int size);
    private:
        vector <Frame> _frames;
        std::timed_mutex _mutexes[6];

        int _tailIdx;
        int _headIdx;

        double _lastGetTimeStamp;
};;

struct ArmorParam
{
    int red_brightness_threshold;
    int blue_brightness_threshold;
    float light_min_area;
    float light_min_size;
    float light_extend_ratio;
    float light_width_height_ratio;
    float light_contour_rect_ratio;
    float light_max_angle_diff;
    float light_max_height_diff_ratio;
    float light_max_y_diff_ratio;
    float light_min_x_diff_ratio;
    float small_armor_min_aspect_ratio;
    float small_armor_max_aspect_ratio;
    float big_armor_max_aspect_ratio;
    float big_armor_min_aspect_ratio;
    string enemy_color;

    //灯条筛选参数
    ArmorParam()
    {
        red_brightness_threshold = 10;
        blue_brightness_threshold = 10;

        light_min_size = 4.0;
        light_min_area = 30.0;

        light_width_height_ratio = 0.9;//0.7
        light_contour_rect_ratio = 0.65;

        light_extend_ratio = 1.1;

        //light_max_angle_diff = 7.0;
        light_max_height_diff_ratio = 0.2;

        light_max_y_diff_ratio = 0.18;
        light_min_x_diff_ratio = 3.6;
        //小装甲板
        small_armor_max_aspect_ratio = 2.1;
        small_armor_min_aspect_ratio = 1.86;
        //大装甲板
        big_armor_min_aspect_ratio = 3.8;
        big_armor_max_aspect_ratio = 4.2;
        enemy_color = "RED";
    }
};

class LightDescriptor {
public:
    LightDescriptor(){}
    LightDescriptor(const cv::RotatedRect& light) {
        width = light.size.width;
        length = light.size.height;
        center = light.center;
        angle = light.angle;
        area = light.size.area();
        light.points(pts);
    }
    const LightDescriptor& operator=(const LightDescriptor& ld){
        this->width = ld.width;
        this->length = ld.length;
        this->center = ld.center;
        this->angle = ld.angle;
        this->area = ld.area;
        //this->pts = ld.pts;
        return *this;
    }

    cv::RotatedRect rec() const {
        return cv::RotatedRect(center, cv::Size2f(width, length), angle);
    }

public:
    float width;
    float length;
    cv::Point2f center;
    float angle;
    float area;
    cv::Point2f pts[4];
};

class Target {
public:
    Target(){}
    const Target& operator=(const Target& pt)
    {
        this->center = pt.center;
        this->armor4dot = pt.armor4dot;
        return *this;
    }

public:
    Point center;
    vector<Point2f> armor4dot;
};

class ArmorDetect{
public:
    ~ArmorDetect(){}
    int armordetect();
    bool findarmor(Mat frame);
    void produce();
    void consume();
    void streamoff();
    //大符
    //void buffinit();
    bool findbuff(Mat image, KalmanFilter KF, Mat measurement);

private:
    CameraParam cameraParam = CameraParam();
    Mat cameraMatrix = cameraParam.getCameraMatrix();
    Mat distCoeffs = cameraParam.getDistCoeffs();
    ArmorParam armorParam = ArmorParam();
    #ifdef DETECT_DEBUG
    int c;
    #endif //DETECT_DEBUG

    #ifdef TIME_COUNT
    int clocker = 0;
    long long previous_time = 0.0, average_time = 0.0;
    #endif //TIME_COUNT
    int threshold_param;
    vector<Mat> channels;
    Mat binBrightImage_light, grayImage;
    FrameBuffer _buffer;

    //大符参数
    bool initflag = true;
    Mat binary, hsving;
    int stateNum = 4;
    int measureNum = 2;

    //初始化 判断是否正在由自瞄进行控制  的数组状态。

    // 0: 播弹盘不动
    // 1: 转动播弹盘
    // 2: 寻找目标模式

    int shoot_style = 0;
};
