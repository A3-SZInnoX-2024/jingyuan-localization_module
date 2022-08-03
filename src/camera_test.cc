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
    cap_video0.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap_video0.set(CAP_PROP_FRAME_HEIGHT, 800);

    VideoCapture cap_video2("/dev/video2");
    cap_video2.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap_video2.set(CAP_PROP_FRAME_HEIGHT, 800);
    
    VideoCapture cap_video4("/dev/video4");
    cout << cap_video4.isOpened() << endl;
    cap_video4.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap_video4.set(CAP_PROP_FRAME_HEIGHT, 720);

    for (int i = 0; i<100; i++){
	//cap_video0.read(img_video0);
	//cap_video2.read(img_video2);
        cap_video4.read(img_video4);
	/*imshow("img0", img_video0);
	imshow("img2", img_video2);
	//imshow("img4", img_video4);
	//cap_video2.release();
	//cap_video4.open("/dev/video4");
	//cap_video4.set(CAP_PROP_FRAME_WIDTH, 1280);
	cap_video4.set(CAP_PROP_FRAME_HEIGHT,720);
	cap_video4.read(img_video4);
	imshow("img4", img_video4);
	cap_video4.release();
	cap_video2.open("/dev/video2");
    	cap_video2.set(CAP_PROP_FRAME_WIDTH, 1280);
    	cap_video2.set(CAP_PROP_FRAME_HEIGHT, 800);*/
        waitKey(30);
    }
    /*
    cap_video0.release();
    cap_video2.release();
    VideoCapture cap_video4("/dev/video4");
    cap_video4.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap_video4.set(CAP_PROP_FRAME_HEIGHT, 720);
    
    for (int i = 0; i < 100; i++){
    	cap_video4.read(img_video4);
	imshow("img4", img_video4);
	waitKey(1000/30);
    }*/

    //imwrite("imgvideo2.png", img_video2);
    imwrite("imgvideo4.png", img_video4);
    //cap_video4.release();
//    cout << img_video2.rows << endl;
    //cout << img_video4.rows << endl;
    return 0;
}
