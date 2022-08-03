#include <iostream>

#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

int main()
{
    Mat img_video0;
    Mat img_video2;
    Mat img_video4;

    VideoCapture cap_video0("/dev/video0");
    VideoCapture cap_video2("/dev/video2");
    VideoCapture cap_video4("/dev/video4");
    
    for (int i; i<100; i++){
        cap_video0.read(img_video0);
        cap_video2.read(img_video2);
        cap_video4.read(img_video4);
        waitKey(1000/30);
    }

    imwrite("img_video0.png", img_video0);
    imwrite("img_video2.png", img_video2);
    imwrite("img_video4.png", img_video4);
    
    return 0;
}