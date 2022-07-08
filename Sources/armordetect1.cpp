#include "armordetect1.h"
//************大恒相机驱动库*************
#include "GxIAPI.h"
#include "DxImageProc.h"
#include "anglesolve1.h"
#include <fstream>
#include <chrono>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
static std::mutex mutex1;
#define DETECT_DEBUG

// 计算当前时刻距离 1970年1月1号 的毫秒数
#ifdef TIME_COUNT
long long getCurrentTime(){
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}
#endif //TIME_COUNT

void FrameBuffer::init(int size)
{
    _frames.resize(size);
    _tailIdx = 0;
    _headIdx = 0;
    _lastGetTimeStamp = 0.0;
}
bool FrameBuffer::push(const Frame& aframe)
{
    // cout<<"xie"<<endl;
    const int newHeadIdx = (_headIdx + 1) % _frames.size();

    unique_lock<timed_mutex> lock(_mutexes[newHeadIdx],chrono::milliseconds(2));

    if(!lock.owns_lock())
        return false;

    _frames[newHeadIdx] = aframe;
    if(newHeadIdx == _tailIdx)
        _tailIdx = (_tailIdx + 1) % _frames.size();
    _headIdx = newHeadIdx;
    return true;
}
bool FrameBuffer::getLatest(Mat& aframe)
{
    volatile const int headIdx = _headIdx;

    unique_lock<timed_mutex> lock(_mutexes[headIdx],chrono::milliseconds(5));
    if(!_frames.size())
    {
        return false;
    }



    if(!lock.owns_lock() ||
       _frames[headIdx].img.empty() ||
       _frames[headIdx].timeStamp == _lastGetTimeStamp)
    {
        return false;
    }
    aframe = _frames[headIdx].img;
    _lastGetTimeStamp = _frames[headIdx].timeStamp;

    return true;

}

cv::RotatedRect& adjustRec(cv::RotatedRect& rec, const int mode) {
    using std::swap;
    float& width = rec.size.width;
    float& height = rec.size.height;
    float& angle = rec.angle;
    if (mode == 1) {
        if (width < height) {
            swap(width, height);
            angle += 90.0;
        }
    }

    while (angle >= 90.0)
        angle -= 180.0;
    while (angle < -90.0)
        angle += 180.0;

    if (mode == 0) {
        if (angle >= 45.0) {
            swap(width, height);
            angle -= 90.0;
        }
        else if (angle < -45.0) {
            swap(width, height);
            angle += 90.0;
        }
    }
    return rec;
}




float predict(float*w,float b,float*X)
{
    int num = 0;
    for (int i =0; i<5;i++)
    {
        num += w[i]*X[i];
    }
    num += b;
    float A = 1/(1+exp(-num));
    return A;
}


void ArmorDetect::produce()
{
    auto startTime = chrono::high_resolution_clock::now();
    Angle_Solve Angle_Solve;
    KalmanFilter KF(4, 2, 0);
    //Mat processNoise(stateNum, 1, CV_32F);
    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);
    KF.transitionMatrix = (Mat_<float>(stateNum, stateNum) << 1, 0, 1, 0,//A 状态转移矩阵
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);
    //这里没有控制矩阵B，默认为0
    setIdentity(KF.measurementMatrix);//H=[1,0,0,0;0,1,0,0]测量矩阵
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));//Q高斯白噪音，单位阵
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));//R高斯白噪音，单位阵
    setIdentity(KF.errorCovPost, Scalar::all(1));//P后验误差协方差矩阵，初始化为单位阵
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));//初始化状态为随机值
    char *pRGB24Buf = NULL;
    ArmorDetect ArmorDetect;
    Mat frame;
    GX_STATUS status = GX_STATUS_SUCCESS;
    GX_DEV_HANDLE hDevice = NULL;
    uint32_t nDeviceNum = 0;
    //初始化库
    status = GXInitLib();
    if (status != GX_STATUS_SUCCESS)
    {
        return;
    }
    //枚举设备列表
    status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if ((status != GX_STATUS_SUCCESS) || (nDeviceNum <= 0))
    {
        return;
    }
    //cout << "Open success!"<<endl;

    //打开第一个设备
    status = GXOpenDeviceByIndex(1, &hDevice);
    if (status == GX_STATUS_SUCCESS)
    {
        int64_t nDecimationH= 1;
        int64_t nDecimationV= 1;
        //设 置 水 平 和 垂 直 Decimation 模 式 为 Sum 模 式
        status = GXSetEnum(hDevice,GX_ENUM_BINNING_HORIZONTAL_MODE,GX_BINNING_HORIZONTAL_MODE_SUM);
        status = GXSetEnum(hDevice,GX_ENUM_BINNING_VERTICAL_MODE,GX_BINNING_VERTICAL_MODE_SUM);
        status = GXSetInt(hDevice, GX_INT_DECIMATION_HORIZONTAL, nDecimationH);
        status = GXSetInt(hDevice, GX_INT_DECIMATION_VERTICAL, nDecimationV);
        //设 置 一 个 offset 偏 移 为 (320,272) ,640x480 尺 寸 的 区 域
        GX_STATUS status = GX_STATUS_SUCCESS;
        int64_t nWidth= 640;
        int64_t nHeight= 512;
        int64_t nOffsetX = 320;
        int64_t nOffsetY = 272;
        status = GXSetInt(hDevice, GX_INT_WIDTH, nWidth);
        status = GXSetInt(hDevice, GX_INT_HEIGHT, nHeight);
        status = GXSetInt(hDevice, GX_INT_OFFSET_X, nOffsetX);
        status = GXSetInt(hDevice, GX_INT_OFFSET_Y, nOffsetY);
        // 使能采集帧率调节模式
        status = GXSetEnum(hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON);
        // 设置采集帧率,假设设置为 210.0（每秒采集 210 张图像）, 用户按照实际需求设此值
        status = GXSetFloat(hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, 210.0);
        //设置曝光时间
        double exposure_time = 5000.0000;
        status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, exposure_time);
        //设 置 曝 光 延 迟 为 2us
        double dExposureValue = 2.0;
        status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_DELAY, dExposureValue);
        //自 动 白 平 衡 设 置
        status = GXSetEnum(hDevice,GX_ENUM_BALANCE_WHITE_AUTO,GX_BALANCE_WHITE_AUTO_CONTINUOUS);
        //获取图像采集帧率
        double current_frame;
        status = GXGetFloat(hDevice,GX_FLOAT_CURRENT_ACQUISITION_FRAME_RATE,&current_frame);
        //cout << current_frame << endl;
        //定义 GXDQAllBuf 的传入参数
        PGX_FRAME_BUFFER pFrameBuffer[5];
        //定 义 实 际 填 充 图 像 个 数
        uint32_t nFrameCount = 0;
        //开始循环采集图像
        Mat frame;
        for(;;)
        {
            status = GXStreamOn(hDevice);
            if (status == GX_STATUS_SUCCESS)
            {
                //调 用 GXDQAllBufs 获 取 队 列 中 所 有 图 像
                status = GXDQAllBufs(hDevice, pFrameBuffer, 5, &nFrameCount, 1000);
                if (status == GX_STATUS_SUCCESS)
                {
                    for(uint32_t i = 0; i < nFrameCount; i++)
                    {
                      if(pFrameBuffer[i] != NULL && pFrameBuffer[i]->nStatus == GX_FRAME_STATUS_SUCCESS)
                      {
                        //图像获取成功
                        //对图像进行处理...

                        #ifdef TIME_COUNT
                        ArmorDetect.clocker += 1;
                        #endif //TIME_COUNT

                        frame.create(pFrameBuffer[i]->nHeight, pFrameBuffer[i]->nWidth, CV_8UC3);
                        pRGB24Buf = new char[(size_t)(pFrameBuffer[i]->nWidth*pFrameBuffer[i]->nHeight*3)];
                        //false图像为不反转
                        VxInt32 DxStatus = DxRaw8toRGB24(pFrameBuffer[i]->pImgBuf,pRGB24Buf,pFrameBuffer[i]->nWidth,pFrameBuffer[i]->nHeight,RAW2RGB_NEIGHBOUR,DX_PIXEL_COLOR_FILTER(BAYERBG),false);
                        if (DxStatus != DX_OK)
                        {
                            //cout << "DxStatus != DX_OK" << endl;
                            if (pRGB24Buf != NULL)
                            {
                            delete []pRGB24Buf;
                            pRGB24Buf = NULL;
                            }
                            return;
                        }

                        memcpy(frame.data, pRGB24Buf, (pFrameBuffer[i]->nHeight * pFrameBuffer[i]->nWidth * 3));
                        if (pRGB24Buf != NULL)
                        {
                            delete []pRGB24Buf;
                            pRGB24Buf = NULL;
                        }
                        if(ArmorDetect.findarmor(frame) == false) continue;
                }
             }
             //调 用 GXQAllBufs 将 获 取 到 的 所 有 图 像 buf 放 回 库 中 继 续 采 图
             status = GXQAllBufs(hDevice);
           }
           }
            #ifdef DETECT_DEBUG
            if(ArmorDetect.c == 27){
                break;
            }
            #endif //DETECT_DEBUG
        }
        status = GXStreamOff(hDevice);
    }
    status = GXCloseDevice(hDevice);
    status = GXCloseLib();
    return;
}

void ArmorDetect::consume()
{
    bool last_con = 0;
    Mat frame;
    while(true)
    {
        if(!_buffer.getLatest(frame))
            continue;
        if(findarmor(frame) == false)
        {
            last_con = 0;
            continue;
        }
        last_con = 1;
    }
}
int ArmorDetect::armordetect(){
    return 0;
}
bool ArmorDetect::findarmor(Mat frame){
    imshow("start", frame);
    Angle_Solve Angle_Solve;
    Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    //此处我改了两分，如果赛场上白光影响比较大的话在调回去
    Scalar lower(0, 80, 150);
    //Scalar lower(0, 20, 120);
    //Scalar upper(200, 255, 255);
    Scalar upper(30, 255, 255);
    Mat mask,copy;
    waitKey(1);
    frame.copyTo(copy);
    cv::cvtColor(frame, mask, cv::COLOR_BGR2HSV);
    
    inRange(mask, lower, upper, mask);
    //imshow("mask", mask);  //**
    waitKey(1);
    split(frame, channels);
    //cannot show
    if (armorParam.enemy_color == "RED") {
        grayImage = channels.at(2) - channels.at(0);  // Get red-blue image;
        threshold_param = armorParam.red_brightness_threshold;
        //imshow("grayImageRED",grayImage);
    } else {
        grayImage = channels.at(0) - channels.at(2);  // Get blue-red image;
        threshold_param = armorParam.blue_brightness_threshold;
        imshow("grayImageBLUE",grayImage);
    }
    threshold(grayImage, binBrightImage_light, threshold_param, 255, cv::THRESH_BINARY);
    //imshow("threshold",binBrightImage_light);
    //c = waitKey(1);
    //对图像进行膨胀
    //erode(grayImage, binBrightImage_light, element);
    bitwise_and(binBrightImage_light, mask, binBrightImage_light);
    dilate(binBrightImage_light, binBrightImage_light, element);
    imshow("and",binBrightImage_light);
    //waitKey(1);
    vector<vector<Point>> lightContours;
    vector<LightDescriptor> lightInfos;
    findContours(binBrightImage_light.clone(), lightContours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);


    for(int i = 0; i < lightContours.size(); i++){
        drawContours(copy, lightContours, i, (0, 0, 255), 2);
    }

    //imshow("copy", copy);
    //waitKey(1);

    for (const auto& contour : lightContours){
        float lightContourArea = contourArea(contour);
        //cout << "Area" << lightContourArea << endl;
        //cout << "size" << contour.size() << endl;
        if (contour.size() < armorParam.light_min_size || lightContourArea < armorParam.light_min_area)continue;
        //RotatedRect lightRec = fitEllipse(contour);
        RotatedRect lightRec = minAreaRect(contour);

        adjustRec(lightRec, 0);



        //cout << "light_width_height_ratio:" <<lightRec.size.width / lightRec.size.height << endl;
        //cout << "light_contour_rect_ratio:" << lightContourArea / lightRec.size.area() << endl;
        if (lightRec.size.width / lightRec.size.height > armorParam.light_width_height_ratio || lightContourArea / lightRec.size.area() < armorParam.light_contour_rect_ratio)
            continue;
        lightRec.size.width *= armorParam.light_extend_ratio;
        lightRec.size.height *= armorParam.light_extend_ratio;
        circle(frame, lightRec.center, 2, Scalar(255, 0, 0), 2);//绘制灯条中心点
        //imshow("asd",frame);
        lightInfos.push_back(LightDescriptor(lightRec));
    }

    //cout<<lightInfos.size()<<endl; //**

    if (!lightInfos.empty()){
    //按照中心的 x 坐标进行排序
    sort(lightInfos.begin(), lightInfos.end(), [](const LightDescriptor& ld1, const LightDescriptor& ld2)
        {
            return ld1.center.x < ld2.center.x;
        });
    vector<Target> targets;
    float meanLen;
    vector<Point3f> POINT_3D;
    //最终识别筛选后的灯条（前期筛选的好的话，灯条数组的长度 5～6 个）
    //cout << lightInfos.size() << endl;

    //imshow("frame",frame);
//    c = waitKey(1);
    for (size_t i = 0; i < lightInfos.size(); i++){
        for (size_t j = i + 1; (j < lightInfos.size()); j++){
            //矩形的四个顶点（Pointf）
            Point tl, tr, bl, br;
            Target target;
            //识别出的左右灯条
            const LightDescriptor& leftLight = lightInfos[i];
            const LightDescriptor& rightLight = lightInfos[j];


            //float angleDiff_ = abs(leftLight.angle - rightLight.angle);
            //cout << "angleDiff_" << angleDiff_ << endl;
            float lenDiff_ratio = abs(leftLight.length - rightLight.length) / max(leftLight.length, rightLight.length);
            //cout << "lenDiff_ratio" << lenDiff_ratio << endl;
            if (lenDiff_ratio > armorParam.light_max_height_diff_ratio){
                //cout<<"lenDiff"<<endl;
                continue;
            }
		   //左右灯条相距距离
            float dis = sqrt(powf((leftLight.center.x - rightLight.center.x), 2) + powf((leftLight.center.y - rightLight.center.y), 2));
		//左右灯条长度的平均值
            meanLen = (leftLight.length + rightLight.length) / 2;
		//左右灯条中心点y的差值
            float yDiff = abs(leftLight.center.y - rightLight.center.y);
		//y差比率
            float yDiff_ratio = yDiff / meanLen;
            //cout << "yDiff_ratio" << yDiff_ratio << endl;
		//左右灯条中心点x的差值
            float xDiff = abs(leftLight.center.x - rightLight.center.x);
		//x差比率
            float xDiff_ratio = xDiff / meanLen;
            //cout << "xDiff_ratio" << xDiff_ratio << endl;
		//相距距离与灯条长度比值
            float ratio = dis / meanLen;
            //cout<<lenDiff_ratio<<"    "<<dis<<"   "<<yDiff_ratio<<"   "<<xDiff_ratio<<"   "<<ratio<<endl;

//            if (yDiff_ratio > armorParam.light_max_y_diff_ratio || xDiff_ratio < armorParam.light_min_x_diff_ratio || ratio > armorParam.big_armor_max_aspect_ratio || (armorParam.small_armor_max_aspect_ratio < ratio && ratio < armorParam.big_armor_min_aspect_ratio) || ratio < armorParam.small_armor_min_aspect_ratio)
//           {
//                continue;
//            }
//            if(ratio > armorParam.small_armor_max_aspect_ratio){
//                //cout << "BIG" << endl;
//                POINT_3D = Angle_Solve.POINT_3D_OF_ARMOR_BIG;
//            }
//            else{
//                //cout << "SMALL" << endl;
//                POINT_3D = Angle_Solve.POINT_3D_OF_ARMOR_SMALL;
//            }


//first_virsion
//            float X[5] = {lenDiff_ratio,dis,yDiff_ratio,xDiff_ratio,ratio};
//            float w[5] = {0.00073398,0.34063113,0.00319593,0.02722487,0.02908563};
//            float A = predict(w,0.013575395699640891,X);
//            cout<<A<<endl;
//            if (A <= 0.8){
//                continue;
//            }

//second_virsion
//            float X[5] = {lenDiff_ratio,dis,yDiff_ratio,xDiff_ratio,ratio};
//            float w[5] = {-0.02259218,-0.02275177,-3.15388574,-0.81726829,-1.18132289};
//            float A = predict(w,13.097098273639993,X);
//            cout<<A<<endl;
//            if (A <= 0.6){
//                continue;
//            }

//third_virsion
              float X[5] = {lenDiff_ratio,dis,yDiff_ratio,xDiff_ratio,ratio};
              float w[5] = {-1.96231213,-0.02850556,-6.53832968,0.04956654,-5.15525653};
              float A = predict(w,19.701902069923097,X);
              //cout<<"A="<<A<<endl;  //**
              if (A <= 0.6){
                  //cout<<"A<0.6"<<endl;
                  continue;
              }

                          if(ratio > armorParam.small_armor_max_aspect_ratio){
//                              cout << "BIG" << endl;
                              POINT_3D = Angle_Solve.POINT_3D_OF_ARMOR_BIG;
                          }
                          else{
//                              cout << "SMALL" << endl;
                              POINT_3D = Angle_Solve.POINT_3D_OF_ARMOR_SMALL;
                          }

            //cout << "ratio:" <<ratio << endl;
            Point pt = Point((leftLight.center.x + rightLight.center.x) / 2, (leftLight.center.y + rightLight.center.y) / 2);

//            Point pt1 = Point(leftLight.center.x, leftLight.center.y);
            circle(frame, pt, 2, Scalar(0, 0, 255), 2);


            if(leftLight.pts[1].x > rightLight.pts[1].x){
               tl = Point(rightLight.pts[1].x,rightLight.pts[1].y);
               tr = Point(leftLight.pts[2].x,leftLight.pts[2].y);
               bl = Point(rightLight.pts[0].x,rightLight.pts[0].y);
               br = Point(leftLight.pts[3].x,leftLight.pts[3].y);
            } else {
               tl = Point(leftLight.pts[1].x,leftLight.pts[1].y);
               tr = Point(rightLight.pts[2].x,rightLight.pts[2].y);
               bl = Point(leftLight.pts[0].x,leftLight.pts[0].y);
               br = Point(rightLight.pts[3].x,rightLight.pts[3].y);
            }
            target.armor4dot.push_back(tl);
            target.armor4dot.push_back(tr);
            target.armor4dot.push_back(br);
            target.armor4dot.push_back(bl);
            target.center = pt;
            targets.push_back(target);
            //calAngle(cameraMatrix, distCoeffs, pt.x, pt.y);
        }
    }

    if (targets.empty()) {
        // {yaw,  pitch,  dis,  shootstyle}
        float moveGimbal[4] = {0, 0, 0, 2};
        Serial moveSentry;
        moveSentry.try1(moveGimbal);
        //delete[] moveGimbal;
        return true;
    }
    sort(targets.begin(), targets.end(), [](const Target& pt1, const Target& pt2)
    {
		//最接近中心点 offsetX点
          return abs(pt1.center.x - 320) < abs(pt2.center.x - 320);
    });
    const Target& true_target = targets[0];

    #ifdef DETECT_DEBUG
    Mat result1 = frame.clone();
    string text_x = to_string(true_target.center.x);
    string text_y = to_string(true_target.center.y);
    string text = "(" + text_x + "," + text_y + ")";
    putText(result1, text, Point(true_target.center.x, true_target.center.y), FONT_HERSHEY_PLAIN, 1.5, Scalar::all(255), 1, 8, 0);
    circle(result1, true_target.center, 2, Scalar(0, 0, 255), 2);
    imshow("result",result1);
    #endif //DETECT_DEBUG

    float* angle_and_distance = Angle_Solve.calPnP(POINT_3D,true_target.armor4dot,cameraMatrix,distCoeffs);
    angle_and_distance = Angle_Solve.compensateOffset(angle_and_distance, 0);//测量枪管和镜头前后距离(mm)
    //angle_and_distance = Angle_Solve.compensateGravity(angle_and_distance, 13.5);//重力补偿(mm)
    float* angle_and_distance_and_style = Angle_Solve.addStyle(angle_and_distance, shoot_style);
    Serial test;
    test.try1(angle_and_distance_and_style);
    delete[] angle_and_distance_and_style;
    delete[] angle_and_distance;
    return true;
}else{
	float moveGimbal[4] = {0, 0, 0, 2};
    Serial moveSentry;
    moveSentry.try1(moveGimbal);
    //delete[] moveGimbal;
    return true;
} 
}

/*
void ArmorDetect::buffinit(){
    KF.init(stateNum, measureNum, 0);
    measurement = Mat::zeros(measureNum, 1, CV_32F);
    KF.transitionMatrix = (Mat_<float>(stateNum, stateNum) << 1, 0, 1, 0,//A 状态转移矩阵
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);
        //这里没有控制矩阵B，默认为0
    setIdentity(KF.measurementMatrix);//H=[1,0,0,0;0,1,0,0]测量矩阵
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));//Q高斯白噪音，单位阵
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));//R高斯白噪音，单位阵
    setIdentity(KF.errorCovPost, Scalar::all(1));//P后验误差协方差矩阵，初始化为单位阵
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));//初始化状态为随机值
}*/

bool ArmorDetect::findbuff(Mat image, KalmanFilter KF, Mat measurement){
    image.copyTo(binary);
    imshow("raw", image);
    cvtColor(image, image, COLOR_BGR2HSV);
    Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    inRange(image, Scalar(85, 150, 79), Scalar(124, 255, 255), image);
    //imshow("inrange", image);
    dilate(image, image, element);
    //imshow("dilate", image);
    //erode(image, image, element);
    //imshow("erode", image);
    //dilate(image, image, Mat());
    //dilate(image, image, Mat());
    //GaussianBlur(image, image, Size(1, 1), 0, 0);
    //imshow("GaussianBlur", image);
    floodFill(image, Point(5, 60), Scalar(255), 0, FLOODFILL_FIXED_RANGE);
    //imshow("floodFill", image);
    threshold(image, image, 90, 255, THRESH_BINARY_INV);
    //imshow("threshold_inv", image);
    vector<vector<Point>> contours;
    findContours(image, contours, RETR_LIST, CHAIN_APPROX_NONE);
    cout << "size" << contours.size() << endl;
    //cout << "******" << endl;
    for (size_t i = 0; i < contours.size(); i++) {
        vector<Point> points;
        double area = contourArea(contours[i]);
        //cout << "area:" << area << endl;
        //cout << "******" << endl;
        if (area < 50 || 1e4 < area) continue;
        drawContours(image, contours, static_cast<int>(i), Scalar(0), 2);
        //imshow("contours", image);
        points = contours[i];
        RotatedRect rrect = fitEllipse(points);
        cv::Point2f* vertices = new cv::Point2f[4];
        rrect.points(vertices);

        float aim = rrect.size.height / rrect.size.width;
        //cout << "aim:" << aim << endl;
        if (aim > 1.7 && aim < 2.6) {
            for (int j = 0; j < 4; j++)
            {
                cv::line(binary, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0), 4);
            }
            float middle = 100000;

            for (size_t j = 1; j < contours.size(); j++) {
                //cout << "size" << contours.size() << endl;
                //cout << "******" << endl;
                vector<Point> pointsA;
                double area = contourArea(contours[j]);
                //cout << "area" << area << endl;
                //cout << "******" << endl;
                if (area < 50 || 1e4 < area) continue;

                pointsA = contours[j];

                RotatedRect rrectA = fitEllipse(pointsA);

                float aimA = rrectA.size.height / rrectA.size.width;
                //cout << "aimA:" << aimA << endl;

                if (aimA > 3.0) {
                    float distance = sqrt((rrect.center.x - rrectA.center.x)*(rrect.center.x - rrectA.center.x) +
                        (rrect.center.y - rrectA.center.y)*(rrect.center.y - rrectA.center.y));
                    //cout << "distance" << distance << endl;
                    if (middle > distance)
                        middle = distance;

                }
            }
            //调参31-33范围最优
            //cout << "middle" << middle << endl;
            if (middle > 32) {                               //距离根据实际情况调节，和图像尺寸和物体远近有关
                cv::circle(binary, Point(rrect.center.x, rrect.center.y), 4, cv::Scalar(0, 0, 255), 1);
                Mat prediction = KF.predict();
                Point predict_pt = Point((int)prediction.at<float>(0), (int)prediction.at<float>(1));

                measurement.at<float>(0) = (float)rrect.center.x;
                measurement.at<float>(1) = (float)rrect.center.y;
                KF.correct(measurement);

                circle(binary, predict_pt, 3, Scalar(255, 0, 0), -1);

                rrect.center.x = (int)prediction.at<float>(0);
                rrect.center.y = (int)prediction.at<float>(1);
            }
        }
    }
    imshow("result", binary);
    //imshow("binary", image);
    //waitKey(1);
    return false;
}
