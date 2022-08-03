#include "ros/ros.h"
#include "localization_moudle/Position.h"

#include <iostream>
#include <cmath>

#include "opencv2/opencv.hpp"

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/common/getopt.h"
#include "apriltag/common/image_u8.h"
#include "apriltag/common/pjpeg.h"
#include "apriltag/common/zarray.h"

using namespace std;
using namespace cv;

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

Mat cam_vehicle_matrix = (Mat_<double>(4,4) << 
	0.999836565761177, -0.01783691104797391, 0.002947265030837263, -57.13496524915718,
	-0.006616468791816809, -0.2093116508007128, 0.9778265977052409, -17.39096735866906,
	-0.0168245091346596, -0.977686287846642, -0.2093954594756788, 255.9823877947722,
	0, 0, 0, 1);

Mat cameraMatrix_front = (Mat_<double>(3,3) << 1.298868409966510e+03, 0, 6.377400159724644e+02,
					 0, 1.299075503646548e+03, 3.875515416693556e+02,
					 0, 0, 1);

Mat distCoeffs_front = (Mat_<double>(1,5) << 0.001087898463482, 0.006889907983617, /*0.157299920720531, 0.192507866331635*/0,0, 0);

struct localization_type{
	int x;
	int y;
	double phi;
}localization_msg;

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
		int center_x = (id % 13 - 1) * 600 + 300;
		int center_y = (id / 13) * 600 + 300;
		objectPoints.push_back(Point3d(center_x - 100, center_y - 100, 0));
		objectPoints.push_back(Point3d(center_x + 100, center_y - 100, 0));
		objectPoints.push_back(Point3d(center_x + 100, center_y + 100, 0));
		objectPoints.push_back(Point3d(center_x - 100, center_y + 100, 0));
		Mat rvec;
		Mat rmat;
		Mat tvec;
		cout << "obj:" << objectPoints << endl;
		cout << "img" << imagePoints << endl;
		cout << "cameraMatrix_front" << cameraMatrix_front << endl;
		cout << "distCoeffs" << distCoeffs_front << endl;
		solvePnP(objectPoints, imagePoints, cameraMatrix_front, distCoeffs_front, rvec, tvec);
		Rodrigues(rvec, rmat);
		cout << "rmat" <<rmat << endl;
		cout << "tvec" << tvec << endl;
		Mat supplement = (Mat_<double>(1, 4) << 0, 0, 0, 1);
		Mat zero_coordinate = (Mat_<double>(4, 1) << 0, 0, 0, 1);
		Mat world_cam_matrix;
		hconcat(rmat, tvec, world_cam_matrix);
		vconcat(world_cam_matrix, supplement, world_cam_matrix);
		cout << "world_cam_matrix" << world_cam_matrix << endl;
		Mat world_vehicle_matrix = cam_vehicle_matrix * world_cam_matrix;
		cout << "world_vehicle_matrix" << world_vehicle_matrix << endl;
		Mat world_vehicle_matrix_invert;
		invert(world_vehicle_matrix, world_vehicle_matrix_invert);
		Mat vehicle_coordinate = world_vehicle_matrix_invert * zero_coordinate;
		cout << "vehicle_coordinate" << vehicle_coordinate << endl;
		localization_msg.x = vehicle_coordinate.at<double>(0, 0);
		localization_msg.y = vehicle_coordinate.at<double>(0, 1);
		cout << "x:" << localization_msg.x << "y:" << localization_msg.y << endl;
		double phi_cos = world_vehicle_matrix.at<double>(1, 0)/1;
		double phi_sin = world_vehicle_matrix.at<double>(1, 1)/1;
		double phi_bycos = acos(phi_cos);
		double phi_bysin = asin(phi_sin);
		double phi = 0;
		if (world_vehicle_matrix.at<double>(1, 1) > 0){
			phi = acos(phi_cos);
			cout << "phi" << phi * 360 / (2 * M_PI) << endl;
		}
		else
		{
			phi = 2 * M_PI - acos(phi_cos);
			cout << "phi" << phi * 360 / (2 * M_PI)  << endl;
		}
		localization_msg.phi = phi;
	}
}

int main(int argc, char **argv)
{
	localization_moudle::Position position_msg;
	ros::init(argc, argv, "localization");
	ros::NodeHandle node;
	ros::Publisher localization_publisher = node.advertise<localization_moudle::Position>("position",1);
	ros::Rate loop_rate(1000/30);

	VideoCapture cap_front;
	cap_front.open("/dev/video2");
	cap_front.set(CAP_PROP_FRAME_WIDTH, 1280);
	cap_front.set(CAP_PROP_FRAME_HEIGHT, 800);
	if (!cap_front.isOpened())return -1;

	apriltag_detector_t *td = apriltag_detector_create();
	apriltag_family_t *tf = tag36h11_create();
	apriltag_detector_add_family(td, tf);

	Mat frame, gray;

	ROS_INFO("init finished");

	while (ros::ok())
	{
		ros::spinOnce();

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
		int fraction = 0;
		int max_fraction = 0;
		int winner_num = 0;
		bool flag = 0;
		for (int i = 0; i < zarray_size(detections); i++)
		{
           	zarray_get(detections, i, &det);
			fraction = pow(det->p[0][0] - det->p[2][0], 2) + pow(det->p[0][1] - det->p[0][1], 2) + pow(det->p[1][0] - det->p[3][0], 2) + pow(det->p[1][1] - det->p[3][1], 2);
			//cout << "id:" << det->id  << "fraction" << fraction << endl;
			if (fraction > max_fraction && det->id > 100)
			{
				flag = 1;
				winner_num = i;
				max_fraction = fraction;
			}
			//localization(det);
		}
		cout << zarray_size(detections) << endl;
		if (flag && zarray_size(detections) > 0)
		{
			cout << "winner_num:" << winner_num << endl;
			zarray_get(detections, winner_num, &det);
			draw(frame, det);
			localization(det);
			position_msg.x = localization_msg.x;
			position_msg.y = localization_msg.y;
			position_msg.phi = localization_msg.phi;
			localization_publisher.publish(position_msg);
			ROS_INFO("located");
		}
		flag = 0;
		max_fraction = 0;
		winner_num = 0;
		fraction = 0;

		imshow("Tag Det", frame);
		waitKey(1000/30);
		loop_rate.sleep();
	}
	return 0;
}
