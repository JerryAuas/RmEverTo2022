/**************************************************************
MIT License
Copyright (c) 2018 SEU-SuperNova-CVRA
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
Authors:    Binyan Hu
**************************************************************/
#include "serial1.h"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

#include <iostream>
#include <fstream>
#include <string>
#include <string.h>
#include <stdexcept>
#include <exception>

using namespace std;

//初始化 判断是否为小陀螺模式  的数组状态。
int initShootMode1 = 0;
//初始化数组
float detectArr[3];
int initMode1 = 0;

//初始化 自瞄/目标寻找
int shootMode = 0;

float shootModeArr[3];



namespace rm
{
    Serial::Serial():
            _serialFd(-1),
            _errorCode(OJBK),
            _en_debug(false),
            _lastRecordSeq(0)
    {
        static_assert(sizeof(ControlFrame) == 12, "Size of backdata is not 16");
        //static_assert(sizeof(FeedBackFrame) == 20, "Size of backdata is not be 20");

        // 初始化通讯帧结构体通用项
        //_controlFrame.SOF = JetsonCommSOF;
        //_controlFrame.frame_seq = 0;
        //_controlFrame.EOF = JetsonCommEOF;
    }

    Serial::~Serial()
    {
        tcflush(_serialFd, TCIOFLUSH); //清空终端未完成的输入/输出请求和数据
//        if (-1 == close(_serialFd))	//close函数关闭文件出错会返回-1
//        {
//            _errorCode = SYSTEM_ERROR;	//SYSTEM_ERROR值为1
//            cout << "Serial closing  failed." << endl;
//        }
//        else
//        {
//            _errorCode = OJBK;	//OJBK值为0
//        }
    }

    int Serial::openPort()
    {
        _serialFd = open("/dev/ttyTHS2", O_RDWR | O_NOCTTY | O_NONBLOCK);
        //ttyTHS2对应妙算上的UART1，O_RDWR为读、写打开，
        //O_NOCTTY为O_NOCTTY 如果pathname指的是终端设备，则不将此设备分配作为此进程的控制终端。
        //O_NONBLOCK 如果pathname指的是一个FIFO、一个块特殊文件或一个字符特殊文件，则此选择项为此文件的本次打开操作和后续的I/O操作设置非阻塞方式。
        if (_serialFd == -1)
        {
            cout << "Open serial port failed." << endl;
            return _errorCode = SYSTEM_ERROR;;
        }

        //linux里面的东西，不太懂，照抄就行吧~
        termios tOption;                                // 串口配置结构体
        tcgetattr(_serialFd, &tOption);                 //获取当前设置
        cfmakeraw(&tOption);
        cfsetispeed(&tOption, B115200);                 // 接收波特率
        cfsetospeed(&tOption, B115200);                 // 发送波特率
        tcsetattr(_serialFd, TCSANOW, &tOption);
        tOption.c_cflag &= ~PARENB;
        tOption.c_cflag &= ~CSTOPB;
        tOption.c_cflag &= ~CSIZE;
        tOption.c_cflag |= CS8;
        tOption.c_cflag &= ~INPCK;
        tOption.c_cflag |= (B460800 | CLOCAL | CREAD);  // 设置波特率，本地连接，接收使能
        tOption.c_cflag &= ~(INLCR | ICRNL);
        tOption.c_cflag &= ~(IXON);
        tOption.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tOption.c_oflag &= ~OPOST;
        tOption.c_oflag &= ~(ONLCR | OCRNL);
        tOption.c_iflag &= ~(ICRNL | INLCR);
        tOption.c_iflag &= ~(IXON | IXOFF | IXANY);
        tOption.c_cc[VTIME] = 1;                        //只有设置为阻塞时这两个参数才有效
        tOption.c_cc[VMIN] = 1;
        tcflush(_serialFd, TCIOFLUSH);                  //TCIOFLUSH刷新输入、输出队列。

//        cout << "Serial preparation complete." << endl;
        return _errorCode = OJBK;
    }

    int Serial::closePort()
    {
        tcflush(_serialFd, TCIOFLUSH);
        if (-1 == close(_serialFd))
        {
            _errorCode = SYSTEM_ERROR;
            cout << "Serial closing failed." << endl;
        }
        else
        {
            _errorCode = OJBK;
        }
        return _errorCode;
    }

    bool Serial::isOpened() const
    {
        return (_serialFd != -1);
    }

//debug模式，会详细记录数据到控制台
    void Serial::setDebug(bool en_debug)
    {
        _en_debug = en_debug;
    }

    int Serial::setup(int& self_color)
    {
        if(_en_debug)
        {
            cout << "[setup]\n";
        }

        u_int16_t tmp_self_team;

//        _controlFrame.shoot_mode = SET_UP;	//0xCCCC
        if(send() == OJBK)
        {
            if(receive() == OJBK)
            {
                tmp_self_team = ((_feedBackFrame.task_mode << 8) | _feedBackFrame.bullet_speed);
                if(tmp_self_team != BLUE_TEAM && tmp_self_team != RED_TEAM)
                {
                    return _errorCode = CORRUPTED_FRAME;
                }

//                _controlFrame.shoot_mode = tmp_self_team;
                if(send() == OJBK)
                {
                    if(tmp_self_team == BLUE_TEAM)
                    {
                        self_color = rm::BLUE;
                    }
                    else if(tmp_self_team == RED_TEAM)
                    {
                        self_color = rm::RED;
                    }
                }
            }
        }
        return _errorCode;
    }

    int Serial::record(u_int8_t& frame_seq)
    {
        if(_en_debug)
        {
            cout << "[record]\n";
        }

        _lastRecordSeq++;	//sequence 帧序列位置
        //_controlFrame.frame_seq = _lastRecordSeq;
//        _controlFrame.shoot_mode = RECORD_ANGLE;	//0xFFFF
        //shoot_mode = 1;
//        _controlFrame.shoot_mode = 1;

        frame_seq = _lastRecordSeq;
        return send();

    }

    int Serial::control(const ControlData& controlData)
    {
        if(_en_debug)
        {
            cout << "[control]\n";
        }
        //pack包装字符串并传入
//        controlData = controlData + shoot_mode;
        _controlFrame = pack(controlData);
        print(_controlFrame);
        return send();
    }

    int Serial::getErrorCode() const
    {
        return _errorCode;
    }


    void Serial::print(const ControlFrame &ct)
    {
        //cout<<hex<<(unsigned int)ct.SOF<<endl;
        //cout<<dec<<(unsigned int)ct.frame_seq<<endl;
//        cout<<hex<<(unsigned int)ct.shoot_mode<<endl;
        //cout<<dec<<              ct.pitch_dev<<endl;
        //cout<<dec<<              ct.yaw_dev<<endl;
        //cout<<dec<<              shoot_mode<<endl;
        //cout<<dec<<         (int)ct.rail_speed<<endl;
//        cout<<hex<<(unsigned int)ct.gimbal_mode<<endl;
        //cout<<hex<<(unsigned int)ct.EOF<<endl;
    }

    /*
    void Serial::print(const FeedBackFrame &fb)
    {
        cout<<hex<<(unsigned int)fb.SOF<<endl;
        cout<<dec<<(unsigned int)fb.frame_seq<<endl;
        cout<<hex<<(unsigned int)fb.task_mode<<endl;
        cout<<dec<<(unsigned int)fb.bullet_speed<<endl;
        cout<<dec<<(unsigned int)fb.rail_pos<<endl;
        cout<<dec<<(unsigned int)fb.shot_armor<<endl;
        cout<<dec<<(unsigned int)fb.remain_HP<<endl;
        for(int i = 0;i<11;i++)
        {
            cout<<dec<<(unsigned int)(fb.reserved[i])<<", ";
        }
        cout<<endl;
        cout<<hex<<(unsigned int)fb.EOF<<endl;
    }
     */

    Serial::ControlFrame Serial::pack(const ControlData& ctrl)
    {
        return ControlFrame
                {
                        //JetsonCommSOF,
                        //ctrl.frame_seq,
//                        ctrl.shoot_mode,
                        //shoot_mode,
                        ctrl.pitch_dev,
                        ctrl.yaw_dev,
                        ctrl.shoot_style
//                        ctrl.moveMode
                        //ctrl.rail_speed,
                        //ctrl.gimbal_mode,
                        //JetsonCommEOF
                };
    }

    FeedBackData Serial::unpack(const Serial::FeedBackFrame& fb)
    {
        return FeedBackData
                {
                        fb.task_mode,
                        fb.bullet_speed,
                        fb.rail_pos,
                        fb.shot_armor,
                        fb.remain_HP
                };
    }

    int Serial::send()
    {
        tcflush(_serialFd, TCOFLUSH);

        int sendCount;
        try
        {
            sendCount  = write(_serialFd, &_controlFrame, sizeof(ControlFrame));
        }
        catch(exception e)
        {
            cout << e.what() << endl;
            return _errorCode = SYSTEM_ERROR;
        }

        if (sendCount == -1)
        {
            if (_en_debug)
            {
                //cout << "\tSerial sending failed. Frame sequence: " << (int)_controlFrame.frame_seq << endl;
            }
            _errorCode = READ_WRITE_ERROR;
        }
        else if (sendCount < static_cast<int>(sizeof(ControlFrame)))
        {
            if (_en_debug)
            {
                //cout << "\tSerial sending failed. "<< sizeof(ControlFrame) - sendCount <<
                    // " bytes unsent. Frame sequence: " << (int)_controlFrame.frame_seq << endl;
            }
            _errorCode = READ_WRITE_ERROR;
        }
        else
        {
            if (_en_debug)
            {
                //cout << "\tSerial sending succeeded. " << "Frame sequence: " << (int)_controlFrame.frame_seq << endl;
            }
            _errorCode = OJBK;
        }

        return _errorCode;
    }
    int Serial::receive()
    {
        memset(&_feedBackFrame,0,sizeof(_feedBackFrame));

        int readCount = 0;
        const auto t1 = std::chrono::high_resolution_clock::now();
        while (readCount < int(sizeof(FeedBackFrame)))
        {
            auto t2 = std::chrono::high_resolution_clock::now();
            if ((chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)).count() > 10) // Time limit is 10ms
            {
                if(_en_debug)
                {
                    cout << "\tReceiving time out. " << sizeof(FeedBackFrame) - readCount
                         << " bytes not received. Frame sequence: "<< (int)_feedBackFrame.frame_seq << endl;
                }
                return _errorCode = TIME_OUT;
            }

            int onceReadCount;
            try
            {
                //
                onceReadCount = read(_serialFd, ((unsigned char *)(&_feedBackFrame)) + readCount, sizeof(FeedBackFrame) - readCount);
            }
            catch(exception e)
            {
                cout << e.what() << endl;
                return _errorCode = SYSTEM_ERROR;
            }

            if (onceReadCount == -1)
            {
                if (errno == EAGAIN)
                {
                    continue;
                }

                if(_en_debug)
                {
                    cout << "\tRead data from serial failed. Frame sequence: " << (int)_feedBackFrame.frame_seq << endl;
                }
                return _errorCode = READ_WRITE_ERROR;
            }

            readCount += onceReadCount;
        }

        tcflush(_serialFd, TCIFLUSH);

        if (_feedBackFrame.SOF != JetsonCommSOF || _feedBackFrame.EOF != JetsonCommEOF)
        {
            if (_en_debug)
            {
                cout << "\tFeed back frame SOF or EOF is not correct. SOF: " << (int)_feedBackFrame.SOF << " ,EOF: " << (int)_feedBackFrame.EOF << endl;
            }
            return _errorCode = CORRUPTED_FRAME;
        }
        else
        {
            if (_en_debug)
            {
                cout << "\tSerial receiving succeeded. " << "Frame sequence: " << (int)_feedBackFrame.frame_seq << endl;
            }
            return _errorCode = OJBK;
        }
    }
    /*
     用于队列初始化
     初步设定队列大小为3
     */
    void initArr(){
        if(initMode1 == 0){
            for(int i=0; i<3; i++){
                detectArr[i] = 0;
            }
            initMode1 = 1;
        }

    }

    void initShoot(){
        if(shootMode == 0){
            for(int i=0; i<3; i++){
                shootModeArr[i] = 0;
            }
            shootMode = 1;
        }
    }

    //用于动态更新数组
    float addArr(float *arr, float num){
        //从后向前增加
        arr[2] = arr[1];
        arr[1] = arr[0];
        arr[0] = num;
        return *arr;
    }

    float addShoot(float *shoot, float num){
        shoot[2] = shoot[1];
        shoot[1] = shoot[0];
        shoot[0] = num;
        return *shoot;
    }


    // 小陀螺反模式判断
    bool detect(float* arr){
        if(arr[2] > 0 && arr[1] > 0 && arr[0] > 0){
            return true;
        }else if(arr[2] <= 0 && arr[1] <= 0 && arr[0] <= 0){
                return true;
        }else{
            return false;
        }
    }

    //应对突然出现shoot_style=2 的情况，对shoot_style进行矫正，去除概率因素
    bool detectShoot(float* shoot){
        if(shoot[2] == shoot[1] && shoot[1] == shoot[0]){
            if(shoot[0] = 2){
                return true;
            }
            return false;
        }
        return false;
    }


    void sendMess(Serial sentry, ControlData data_){
        /*
         应对场景：
         敌方步兵进入哨兵瞄准区域，多数情况为小陀螺旋转模式,
         对已瞄准目标进行标定，比较之前2帧包括本次共3帧视频帧,

         小陀螺右旋模式
         镜头会随目标的旋转而出现 右-左-右 循环情况
         */



        
        
    }
    void Serial::try1(float* angle){
        ControlData data_;
	    std::chrono::duration<double, std::ratio<1, 30>> hz30(3.5);
        //0是yaw,1是pitch
        if(angle[1] > 1.5 || angle[1] < -1.5){
            data_.pitch_dev = angle[1];
        }
        else{
            data_.pitch_dev = 0;
        }
        if(fabs(angle[0]) > 1){
	    if(angle[0]>0){
		data_.yaw_dev = -log(exp(fabs(angle[0]))) /3;
		    if(data_.yaw_dev < 0.8){
			data_.yaw_dev *= 0.3;
			if(data_.yaw_dev > -0.4){
			    data_.yaw_dev = 0;			
			}
		    }
		}else{
	        data_.yaw_dev = log(exp(fabs(angle[0]))) / 3;
		if(data_.yaw_dev < 0.8){
		    data_.yaw_dev *= 0.3;
		    if(data_.yaw_dev < 0.4){
			    data_.yaw_dev = 0;			
			}
		    }
		}
            //云台移动方向相反
        }
        else{
            data_.yaw_dev = 0;
        }
        //pitch 正值向下
        //yaw 正值向左
	
	//data_.yaw_dev = 0.5;
	//data_.pitch_dev = 0;
        Serial test;

        //初始化模式数组
        initShoot();

        *shootModeArr = addShoot(shootModeArr, data_.shoot_style);

        if(detectShoot(shootModeArr)){
            data_.shoot_style = 2;
        }else{
            //此处赋值 0、1 都一样， 下面仍然会判断
            data_.shoot_style = 0;
        }

	    if(data_.shoot_style != 2) {
            if (fabs(data_.yaw_dev) < 1 && data_.pitch_dev == 0) {
                data_.shoot_style = 1;
            } else { data_.shoot_style = 0; }
            //        data_.shoot_style = angle[3];
            cout << "yaw: " << data_.yaw_dev << "  pitch: " << data_.pitch_dev << "  shoot_style: " << data_.shoot_style
                 << endl;


            // 初始化判断数组
            initArr();

            //
            *detectArr = addArr(detectArr, data_.yaw_dev);
            int length = sizeof(detectArr) / sizeof(detectArr[0]);

            if (detect(detectArr)) {
                test.openPort();
                test.isOpened();
                test.tryControl(data_, hz30);
                test.closePort();
            } else {
                data_.yaw_dev += 1;
                test.openPort();
                test.isOpened();
                test.tryControl(data_, hz30);
                test.closePort();
            }
        }else{
            test.openPort();
            test.isOpened();
            test.tryControl(data_, hz30);
            test.closePort();
	    }
    }
}
