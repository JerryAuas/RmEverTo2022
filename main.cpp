#include <iostream>
#include "armordetect1.h"
#include <thread>
using namespace std;

int main()
{
    ArmorDetect ArmorDetect;

     std::thread th1(&ArmorDetect::produce,&ArmorDetect);
     std::thread th2(&ArmorDetect::consume,&ArmorDetect);
     th1.join();
     th2.join();
    return 0;
}


