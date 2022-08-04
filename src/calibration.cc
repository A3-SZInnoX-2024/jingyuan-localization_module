#include <iostream>

#include "opencv2/opencv.hpp"

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/common/getopt.h"
#include "apriltag/common/image_u8.h"
#include "apriltag/common/pjpeg.h"
#include "apriltag/common/zarray.h"

//TODO
#define VIDEO "/dev/video2"

using namespace std;
using namespace cv;

void creat_xml(Mat matrix)
{
        FileStorage fs("matrix/matrix.xml", FileStorage::WRITE);
        fs << "matrix" << matrix;
        fs.release();
}

void draw(Mat frame, apriltag_detection_t *det)
{
	line(frame, Point(det->p[0][0], det->p[0][1]),
	Point(det->p[1][0], det->p[1][1]),
	Scalar(0xff, 0, 0), 2);
	line(frame, Point(det->p[0][0], det->p[0][1]),
	Point(det->p[3][0], det->p[3][1]),
	Scalar(0xff, 0xff, 0), 2);
	line(frame, Point(det->p[1][0], det->p[1][1]),
	Point(det->p[2][0], det->p[2][1]),
	Scalar(0, 0xff, 0), 2);
	line(frame, Point(det->p[2][0], det->p[2][1]),
	Point(det->p[3][0], det->p[3][1]),
	Scalar(0, 0, 0xff), 2);
	stringstream ss;
	ss << det->id;
	String text = ss.str();
	int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
	double fontscale = 1.0;
	int baseline;
	Size textsize = getTextSize(text, fontface, fontscale, 2,
                                &baseline);
	putText(frame, text, Point(det->c[0]-textsize.width/2,
	det->c[1]+textsize.height/2),
	fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
}

//TODO
Mat cameraMatrix = (Mat_<double>(3,3) << 865.920425978195, 0, 654.873058068272,
0, 865.658142181186, 350.919789060440,
0, 0, 1);
/*1147.14807662718, 0, 631.287953202974,
0, 1146.30770069275, 412.277737454872,
0, 0, 1); *///left

		/*1145.91055981631, 0, 691.158173381245,
0, 1145.67928958460, 412.102171045947,
0, 0, 1);*///right

		/*1.298868409966510e+03, 0, 6.377400159724644e+02,
		0, 1.299075503646548e+03, 3.875515416693556e+02,
		 0, 0, 1);
*/
Mat distCoeffs = (Mat_<double>(1,5) << 0.00104731823467106, 0.00232463818599928, 0, 0, 0);
		//0.00100803187859399, 0.00426627267853791, 0, 0, 0);//left
		//0.00137372073339136, 0.00469950907335194, 0, 0, 0);//right
	       //	0.001087898463482, 0.006889907983617, 0.157299920720531, 0.192507866331635, 0);
//

Mat rotate_world_vehicle = (Mat_<double>(3,3) << 1, 0, 0,
						 0, 1, 0,
						 0, 0, 1);
//TODO
Mat world_vehicle_matrix = (Mat_<double>(4, 4) << 1, 0, 0, - 6900,
						  0, 1, 0, - 3900,
						  0, 0, 1, - 0,
						  0, 0, 0, 1);

void localization(apriltag_detection_t *det)
{
	vector<Point3d> objectPoints;
	vector<Point2d> imagePoints;
	imagePoints.push_back(Point2d(det->p[0][0], det->p[0][1]));
	imagePoints.push_back(Point2d(det->p[1][0], det->p[1][1]));
	imagePoints.push_back(Point2d(det->p[2][0], det->p[2][1]));
	imagePoints.push_back(Point2d(det->p[3][0], det->p[3][1]));

	int id = det->id;

	if (id > 100)
	{
		id = id - 100;
		int center_x = ((id - 1) % 13) * 600 + 300;
		int center_y = ((id - 1) / 13) * 600 + 300;
		objectPoints.push_back(Point3d(center_x - 100, center_y - 100, 0));
		objectPoints.push_back(Point3d(center_x + 100, center_y - 100, 0));
		objectPoints.push_back(Point3d(center_x + 100, center_y + 100, 0));
		objectPoints.push_back(Point3d(center_x - 100, center_y + 100, 0));
		Mat rvec;
		Mat rmat;
		Mat tvec;
		cout << "obj:" << objectPoints << endl;
		cout << "img" << imagePoints << endl;
		cout << "cameraMatrix" << cameraMatrix << endl;
		cout << "distCoeffs" << distCoeffs << endl;
		solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
		Rodrigues(rvec, rmat);
		cout << "rmat" << rmat << endl;
		cout << "tvec" << tvec << endl;
		Mat supplement = (Mat_<double>(1, 4) << 0, 0, 0, 1);
		Mat zero_coordinate = (Mat_<double>(4, 1) << 0, 0, 0, 1);
		Mat world_cam_matrix;
		hconcat(rmat, tvec, world_cam_matrix);
		vconcat(world_cam_matrix, supplement, world_cam_matrix);
		cout << "world_cam_matrix" << world_cam_matrix << endl;
		Mat world_cam_matrix_invert;
		invert(world_cam_matrix, world_cam_matrix_invert);
		cout << "world_cam_matrix_invert" << world_cam_matrix_invert << endl;
		Mat cam_coordinate = world_cam_matrix_invert * zero_coordinate;
		cout << "cam_coordinate" << cam_coordinate << endl;
		Mat cam_vehicle_matrix = world_vehicle_matrix * world_cam_matrix_invert;
		cout << "cam_vehicle_matrix" << cam_vehicle_matrix << endl;
		//creat_xml(cam_vehicle_matrix);
	}
}

int main(int argc, char **argv)
{
	VideoCapture cap_front;
	cap_front.open(VIDEO);
	cap_front.set(CAP_PROP_FRAME_WIDTH, 1280);
	cap_front.set(CAP_PROP_FRAME_HEIGHT, 800);//TODO
	if (!cap_front.isOpened())return -1;

	apriltag_detector_t *td = apriltag_detector_create();
	apriltag_family_t *tf = tag36h11_create();
	apriltag_detector_add_family(td, tf);

	Mat frame, gray;

	while (1)
	{
		cap_front.read(frame);
		cvtColor(frame, gray, COLOR_BGR2GRAY);

		image_u8_t img_tag = {
			.width = gray.cols,
			.height = gray.rows,
			.stride = gray.cols,
			.buf = gray.data
		};
		zarray_t *detections = apriltag_detector_detect(td, &img_tag);
		apriltag_detection_t *det;
		for (int i = 0; i < zarray_size(detections); i++)
		{
           	 	zarray_get(detections, i, &det);
			//draw(frame, det);
			//cout << "id:" << det->id << endl;
			localization(det);
		}
		//imshow("Tag Det", frame);
		waitKey(1000/30);
	}
	return 0;
}
