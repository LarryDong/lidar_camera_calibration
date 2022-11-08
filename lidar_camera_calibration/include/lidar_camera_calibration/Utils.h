#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <map>
#include <fstream>
#include <cmath>

#include "opencv2/opencv.hpp"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/common/intersections.h>

using namespace std;
using namespace cv;

cv::Point project(const pcl::PointXYZ &pt, const cv::Mat &projection_matrix){
	//cv::Point2f xy = projectf(pt, projection_matrix);
	cv::Mat pt_3D(4, 1, CV_32FC1);

	pt_3D.at<float>(0) = pt.x;
	pt_3D.at<float>(1) = pt.y;
	pt_3D.at<float>(2) = pt.z;
	pt_3D.at<float>(3) = 1.0f;

	cv::Mat pt_2D = projection_matrix * pt_3D;
	float w = pt_2D.at<float>(2);
	float x = pt_2D.at<float>(0) / w;
	float y = pt_2D.at<float>(1) / w;
	return cv::Point(x, y);
}


cv::Mat project(cv::Mat projection_matrix, cv::Rect frame, pcl::PointCloud<pcl::PointXYZ> point_cloud, pcl::PointCloud<pcl::PointXYZ> *visible_points){
	cv::Mat plane = cv::Mat::zeros(frame.size(), CV_8UC1);
	ROS_INFO_STREAM("Points before project into image: " << point_cloud.points.size());
	for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++){
		// cout << "pt: " << pt->x << ", " << pt->y << ", " << pt->z << endl;
		if (pt->z < 0)		//~ only consider front point
			continue;
		cv::Point xy = project(*pt, projection_matrix);		//~ projects points in approx. camera pose to camera image

		// if(xy.x >=0 && xy.x < frame.br().x && xy.y >=0 && xy.x < frame.br().y){
		if (xy.inside(frame)){
		// 	if (visible_points != NULL){					//~ not used.
		// 		visible_points->push_back(*pt);
		// 	}
			plane.at<uchar>(xy.y, xy.x)=255;
		}
	}
	cv::Mat plane_gray;
	cv::dilate(plane, plane_gray, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
	return plane_gray;
}

void onMouse(int event, int x, int y, int f, void *g){

	cv::Point *P = static_cast<cv::Point *>(g);
	switch (event){
	case CV_EVENT_LBUTTONDOWN:
		P->x = x;
		P->y = y;
		break;

	case CV_EVENT_LBUTTONUP:
		P->x = x;
		P->y = y;
		// std::cout << P->x << " " << P->y << "\n";
		break;

	default:
		break;
	}
}