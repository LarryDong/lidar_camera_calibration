#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <algorithm>
#include <map>

#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include "lidar_camera_calibration/Corners.h"
#include "lidar_camera_calibration/PreprocessUtils.h"
#include "lidar_camera_calibration/Find_RT.h"

#include "lidar_camera_calibration/marker_6dof.h"

////// Dongyan's include
#include <pcl/io/ply_io.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/visualization/pcl_visualizer.h>


using namespace cv;
using namespace std;
using namespace ros;
using namespace message_filters;
using namespace pcl;

string CAMERA_INFO_TOPIC;
string VELODYNE_TOPIC;
Mat projection_matrix;

pcl::PointCloud<myPointXYZRID> point_cloud;
Hesai::PointCloud point_cloud_hesai;
pcl::PointCloud<lslidarPointXYZI> point_cloud_ls;

Eigen::Quaterniond qlidarToCamera; 
Eigen::Matrix3d lidarToCamera;

ros::Publisher pubPointCloudInCam;


void callback_noCam(const sensor_msgs::PointCloud2ConstPtr& msg_pc, const lidar_camera_calibration::marker_6dof::ConstPtr& msg_rt){

	ROS_INFO_STREAM("Velodyne scan received at " << msg_pc->header.stamp.toSec());
	ROS_INFO_STREAM("marker_6dof received at " << msg_rt->header.stamp.toSec());

	// Loading Velodyne point cloud_sub
    if (config.lidar_type == 0) 				// velodyne lidar
		fromROSMsg(*msg_pc, point_cloud);
    else if (config.lidar_type == 1){ 			// hesai lidar
        fromROSMsg(*msg_pc, point_cloud_hesai);
        point_cloud = *(toMyPointXYZRID(point_cloud_hesai));
    }
	else if (config.lidar_type == 2){			// lslidar
        fromROSMsg(*msg_pc, point_cloud_ls);
        point_cloud = *(lslidar2MyPointXYZRID(point_cloud_ls));
	}


	//Rotation matrix to transform lidar point cloud to camera's frame
	point_cloud = transform(point_cloud,  config.initialTra[0], config.initialTra[1], config.initialTra[2], config.initialRot[0], config.initialRot[1], config.initialRot[2]);
	
	//--> test rotated message in camera's.
	sensor_msgs::PointCloud2 pcMsg;
    pcl::toROSMsg(point_cloud, pcMsg);
    pcMsg.header.stamp = ros::Time::now();
    pcMsg.header.frame_id = "/laser_link";
    pubPointCloudInCam.publish(pcMsg);
	//<-- test rotated message in camera's.


	point_cloud = intensityByRangeDiff(point_cloud, config);
	pcl::io::savePLYFile("/home/larrydong/Desktop/testPCD.ply", point_cloud);

	cv::Mat temp_mat(config.s, CV_8UC3);						// config.s: camera's size.
	pcl::PointCloud<pcl::PointXYZ> retval = *(toPointsXYZ(point_cloud));

	ROS_WARN("--> Print aruco information: id, txyz, rxyz.");


	bool no_error = getCorners(temp_mat, retval, config.P, config.num_of_markers, config.MAX_ITERS);	// TODO:
	if(no_error){
		// extract marker_info. markder info: only marker's pose (position & rotation. The corners are calculated by find_transformation)
		std::vector<float> marker_info;
		for(std::vector<float>::const_iterator it = msg_rt->dof.data.begin(); it != msg_rt->dof.data.end(); ++it){
			marker_info.push_back(*it);
			std::cout << *it << " ";		// id, tx,ty,tz,rx,ry,rz
		}
		std::cout << std::endl;

		// HERE transformed the point_cloud from LiDAR to camera's pose.
		qlidarToCamera = Eigen::AngleAxisd(config.initialRot[2], Eigen::Vector3d::UnitZ())
			*Eigen::AngleAxisd(config.initialRot[1], Eigen::Vector3d::UnitY())
			*Eigen::AngleAxisd(config.initialRot[0], Eigen::Vector3d::UnitX());
		//~ ZYX eular angle. rotate from Z, Y, and then X.
		std::cout << "Rotate XYZ: " << config.initialRot[0] << ", " << config.initialRot[1] << ", " << config.initialRot[2] << endl;
		lidarToCamera = qlidarToCamera.matrix();
		std:: cout << "\n Initial Rotation \n " << lidarToCamera << "\n";

		find_transformation(marker_info, config.num_of_markers, config.MAX_ITERS, lidarToCamera);
	}
	//ros::shutdown();
}


int main(int argc, char** argv){
	readConfig();						//~ Load global parameter `config` here. from header.
	ROS_WARN("Load configuration done...");
	ros::init(argc, argv, "find_transform");
	ros::NodeHandle n;

	ROS_INFO_STREAM("Reading CameraInfo from configuration file");
	n.getParam("/lidar_camera_calibration/velodyne_topic", VELODYNE_TOPIC);
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, VELODYNE_TOPIC, 1);
	message_filters::Subscriber<lidar_camera_calibration::marker_6dof> rt_sub(n, "lidar_camera_calibration_rt", 1);		//~ provided by `aruco_mapping`.
	pubPointCloudInCam = n.advertise<sensor_msgs::PointCloud2>("/pc_in_cam", 10);

	typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, lidar_camera_calibration::marker_6dof> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, rt_sub);
	sync.registerCallback(boost::bind(&callback_noCam, _1, _2));
	ros::spin();

	return EXIT_SUCCESS;
}
