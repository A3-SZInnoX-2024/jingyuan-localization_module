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

Mat cam_vehicle_matrix_left = (Mat_<double>(4,4) << 
		 0.02490513271721116, 0.4166923841261823, -0.9087063284557757, -58.6491130272924,
		 0.9996022253648473, 0.00165273536643637, 0.02815420948017134, -87.94911274958258,
		 0.01323349575822407, -0.909046052451165, -0.4164854728714682, 272.3884351922972,
		 0, 0, 0, 1);

Mat cameraMatrix_left = (Mat_<double>(3,3) << 

1145.91055981631, 0, 691.158173381245,
		0, 1145.67928958460, 412.102171045947,
		0, 0, 1);
/*
		1.298868409966510e+03, 0, 6.377400159724644e+02,
		0, 1.299075503646548e+03, 3.875515416693556e+02,
		0, 0, 1);*/
Mat distCoeffs_left = (Mat_<double>(1,5) << 0.001087898463482, 0.006889907983617, 0, 0, 0);

struct localization_type{
	int x;
	int y;
	double phi;
}localization_msg;

struct localization_mineral_type{
	int x;
	int y;
	double phi;
}localization_mineral_msg;

void localization_mineral(apriltag_detection_t *det)
{
	vector<Point3d> objectPoints;
	objectPoints.push_back(Point3d(0, -40, -40));
	objectPoints.push_back(Point3d(0, 40, -40));
	objectPoints.push_back(Point3d(0, 40, 40));
	objectPoints.push_back(Point3d(0, -40, 40));
	vector<Point2d> imagePoints;
	imagePoints.push_back(Point2d(det->p[0][0], det->p[0][1]));
	imagePoints.push_back(Point2d(det->p[1][0], det->p[1][1]));
	imagePoints.push_back(Point2d(det->p[2][0], det->p[2][1]));
	imagePoints.push_back(Point2d(det->p[3][0], det->p[3][1]));
	Mat rvec;
	Mat rmat;
	Mat tvec;
	solvePnP(objectPoints, imagePoints, cameraMatrix_left, distCoeffs_left, rvec, tvec);//TODO
	Mat supplement = (Mat_<double>(1, 4) << 0, 0, 0, 1);
	Mat zero_coordinate = (Mat_<double>(4, 1) << 0, 0, 0, 1);
	Rodrigues(rvec, rmat);
	Mat mineral_cam_matrix;
	hconcat(rmat, tvec, mineral_cam_matrix);
	vconcat(mineral_cam_matrix, supplement, mineral_cam_matrix);
	Mat mineral_vehicle_matrix = cam_vehicle_matrix_left * mineral_cam_matrix;
	Mat mineral_coordinate_in_vehicle = mineral_vehicle_matrix * zero_coordinate;
	localization_mineral_msg.x = mineral_coordinate_in_vehicle.at<double>(0, 0);
	localization_mineral_msg.y = mineral_coordinate_in_vehicle.at<double>(0, 1);
	double mineral_jhat_x = mineral_vehicle_matrix.at<double>(1, 0);
	double mineral_jhat_y = mineral_vehicle_matrix.at<double>(1, 1);
	double phi_cos = mineral_jhat_y / sqrt(pow(mineral_jhat_x, 2) + pow(mineral_jhat_y, 2));
	cout << "mjx:" << mineral_jhat_x << endl;
	cout << "mjy:" << mineral_jhat_y << endl;
	if (mineral_jhat_x > 0)
	{
		localization_mineral_msg.phi = acos(phi_cos);
	}
	else
	{
		localization_mineral_msg.phi = 2 * M_PI - acos(phi_cos);
	}
	cout << localization_mineral_msg.phi * 360 / (2 * M_PI) << endl; 
}

void localization(apriltag_detection_t *det)
{
	vector<Point3d> objectPoints;
	vector<Point2d> imagePoints;
	imagePoints.push_back(Point2d(det->p[0][0], det->p[0][1]));
	imagePoints.push_back(Point2d(det->p[1][0], det->p[1][1]));
	imagePoints.push_back(Point2d(det->p[2][0], det->p[2][1]));
	imagePoints.push_back(Point2d(det->p[3][0], det->p[3][1]));

	int id = det->id;
	Mat cameraMatrix, distCoeffs;
	cameraMatrix = cameraMatrix_left;
	distCoeffs = distCoeffs_left;
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
		//cout << "obj:" << objectPoints << endl;
		//cout << "img" << imagePoints << endl;
		//cout << "cameraMatrix" << cameraMatrix << endl;
		//cout << "distCoeffs" << distCoeffs << endl;
		solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
		Rodrigues(rvec, rmat);
		//cout << "rmat" <<rmat << endl;
		//cout << "tvec" << tvec << endl;
		Mat supplement = (Mat_<double>(1, 4) << 0, 0, 0, 1);
		Mat zero_coordinate = (Mat_<double>(4, 1) << 0, 0, 0, 1);
		Mat world_cam_matrix;
		hconcat(rmat, tvec, world_cam_matrix);
		vconcat(world_cam_matrix, supplement, world_cam_matrix);
		//cout << "world_cam_matrix" << world_cam_matrix << endl;
		Mat world_vehicle_matrix = cam_vehicle_matrix_left * world_cam_matrix;
		//cout << "world_vehicle_matrix" << world_vehicle_matrix << endl;
		Mat world_vehicle_matrix_invert;
		invert(world_vehicle_matrix, world_vehicle_matrix_invert);
		Mat vehicle_coordinate = world_vehicle_matrix_invert * zero_coordinate;
		//cout << "vehicle_coordinate" << vehicle_coordinate << endl;
		localization_msg.x = vehicle_coordinate.at<double>(0, 0);
		localization_msg.y = vehicle_coordinate.at<double>(0, 1);
		//cout << "x:" << localization_msg.x << "y:" << localization_msg.y << endl;
		double addition = sqrt(pow(world_vehicle_matrix.at<double>(1, 0), 2) + pow(world_vehicle_matrix.at<double>(1, 1), 2));
		double phi_cos = world_vehicle_matrix.at<double>(1, 0)/addition;
		double phi_sin = world_vehicle_matrix.at<double>(1, 1)/addition;
		double phi_bycos = acos(phi_cos);
		double phi_bysin = asin(phi_sin);
		double phi = 0;
		if (world_vehicle_matrix.at<double>(1, 1) > 0){
			phi = acos(phi_cos);
			//cout << "phi" << phi * 360 / (2 * M_PI) << endl;
		}
		else
		{
			phi = 2 * M_PI - acos(phi_cos);
			//cout << "phi" << phi * 360 / (2 * M_PI)  << endl;
		}
		localization_msg.phi = phi;
	}
}

int main(int argc, char **argv)
{
	localization_moudle::Position position_msg;
	localization_moudle::Position position_mineral_msg;
	ros::init(argc, argv, "localization_left");
	ros::NodeHandle node;
	ros::Publisher localization_publisher = node.advertise<localization_moudle::Position>("position_left",1);
	ros::Publisher localization_mineral_publisher = node.advertise<localization_moudle::Position>("position_mineral",1);
	ros::Rate loop_rate(1000/30);

	VideoCapture cap_left;
	cap_left.open("/dev/video2");
	cap_left.set(CAP_PROP_FRAME_WIDTH, 1280);
	cap_left.set(CAP_PROP_FRAME_HEIGHT, 800);
	if (!cap_left.isOpened())return -1;

	apriltag_detector_t *td = apriltag_detector_create();
	apriltag_family_t *tf = tag36h11_create();
	apriltag_detector_add_family(td, tf);

	Mat frame_left, gray_left, frame_right, gray_right;

	ROS_INFO("init finished");

	int fraction = 0;
	int max_fraction = 0;
	int winner_num = 0;
	int fraction_mineral = 0;
	int max_fraction_mineral = 0;
	int winner_num_mineral = 0;

	bool flag = 0;
	bool flag_mineral = 0;
	bool left_right = 0;

	while (ros::ok())
	{
		ros::spinOnce();

		cap_left.read(frame_left);
		cvtColor(frame_left, gray_left, COLOR_BGR2GRAY);

		image_u8_t img_tag_left = {
			.width = gray_left.cols,
			.height = gray_left.rows,
			.stride = gray_left.cols,
			.buf = gray_left.data
		};

		zarray_t *detections = apriltag_detector_detect(td, &img_tag_left);
		apriltag_detection_t *det;
		
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
			if (fraction > max_fraction_mineral && det->id < 101 && (det->id % 4 == 1)){
				flag_mineral = 1;
				winner_num_mineral = i;
				max_fraction_mineral = fraction;
			}
			//localization(det);
		}
		cout << zarray_size(detections) << endl;
		
		if (flag && zarray_size(detections) > 0)
		{
			//cout << "winner_num:" << winner_num << endl;
			zarray_get(detections, winner_num, &det);
			draw(frame_left, det);
			localization(det);
			position_msg.x = localization_msg.x;
			position_msg.y = localization_msg.y;
			position_msg.phi = localization_msg.phi;
			localization_publisher.publish(position_msg);
			//ROS_INFO("located");
		}
		if (flag_mineral && zarray_size(detections) > 0)
		{
			//cout << "winner_num:" << winner_num << endl;
			zarray_get(detections, winner_num_mineral, &det);
			draw(frame_left, det);
			localization_mineral(det);
			position_mineral_msg.x = localization_mineral_msg.x;
			position_mineral_msg.y = localization_mineral_msg.y;
			position_mineral_msg.phi = localization_mineral_msg.phi;
			localization_mineral_publisher.publish(position_mineral_msg);
			//ROS_INFO("located");
		}

		flag = 0;
		flag_mineral = 0;
		max_fraction = 0;
		max_fraction_mineral = 0;
		winner_num = 0;
		winner_num_mineral = 0;
		fraction = 0;
		fraction_mineral = 0;

		//imshow("Tag Det", frame_left);
		//waitKey(1000/30);
		loop_rate.sleep();
	}
	return 0;
}
