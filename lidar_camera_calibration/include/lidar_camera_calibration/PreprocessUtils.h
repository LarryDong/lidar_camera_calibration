#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <map>
#include <fstream>
#include <cmath>

#include "opencv2/opencv.hpp"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <ros/package.h>

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

//------------- PointT declaration ---------------//

struct myPointXYZRID{			//~ Ring, Intensity, Distance(range)
	PCL_ADD_POINT4D; // quad-word XYZ
	float intensity; ///< laser intensity reading
	uint16_t ring; ///< laser ring number
	float range;EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};
POINT_CLOUD_REGISTER_POINT_STRUCT(
		myPointXYZRID, (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity) (uint16_t, ring, ring))


namespace Hesai{
struct PointXYZIT {
    PCL_ADD_POINT4D;
    float intensity;
    double timestamp;
    uint16_t ring;                      ///< laser ring number
    float range;EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
};
typedef pcl::PointCloud<::Hesai::PointXYZIT> PointCloud;
}; // namespace Hesai
POINT_CLOUD_REGISTER_POINT_STRUCT(Hesai::PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring))


//~ lslidar's point type. intensity is not "float".
struct lslidarPointXYZI{
	PCL_ADD_POINT4D;
	double timestamp;
	uint16_t ring;
	uint8_t intensity;				// lslidar's intensity is a `uint8_type` (type=2)
	float range;EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(lslidarPointXYZI,
								(float, x, x)(float, y, y)(float, z, z)
								(uint8_t, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring))
typedef pcl::PointCloud<lslidarPointXYZI> lslidarPointCloud;


//------------- config utils ---------------//
struct config_settings{
	cv::Size s;			                        //image width and height
	std::vector<std::pair<float,float>> xyz_;   //filter point cloud to region of interest
	float intensity_thresh;                     //intensity thresh
	int num_of_markers;
	bool useCameraInfo;
	cv::Mat P;									//~ camera's projection matrix
	int MAX_ITERS;
	std::vector<float> initialRot;
	std::vector<float> initialTra;
    int lidar_type;                             //0: velodyne, 1: hesai, 2: lslidar

	void print(){
		std::cout <<  "Size of the frame: " << s.width << " x " << s.height << "\n";
		for(int i = 0; i < 3; i++){
			std::cout << "Limits: " << xyz_[i].first << " to " << xyz_[i].second << std::endl;
		}
		std::cout << "Number of markers: " << num_of_markers << std::endl;
		std::cout << "Intensity threshold (between 0.0 and 1.0): " << intensity_thresh <<std::endl; 
		std::cout << "\nuseCameraInfo: " << useCameraInfo << "\n";
		std::cout << "Projection matrix: \n" << P << "\n";
		std::cout << "MAX_ITERS: " << MAX_ITERS << "\n";
        std::cout << "Lidar_type: " << ((lidar_type == 2) ? "Lslidar\n" : "others\n");
		std::cout << "initial rotation: "
							<< initialRot[0] << " " 
							<< initialRot[1] << " "
							<< initialRot[2] << "\n";
		std::cout << "initial translation: "
							<<	initialTra[0] << " "  
							<<	initialTra[1] << " "  
							<<	initialTra[2] << "\n";
	}
}config;



void readConfig(){
	std::string pkg_loc = ros::package::getPath("lidar_camera_calibration");
	std::ifstream infile(pkg_loc + "/conf/config_file.txt");
	float left_limit=0.0, right_limit=0.0;
	float angle;
	float dist;

	// line 1: camera's width/height
	infile >> config.s.width >> config.s.height;

	// line 2-4: x,y,z's range (in lidar's coordinate)
	for(int i = 0; i<3; i++){
		infile >> left_limit >> right_limit;
		config.xyz_.push_back(std::pair<float,float>(left_limit, right_limit));
	}

	// line 5: lidar intensity threshold
	// line 6: aruco markers number
	// line 7: use camera info topic or from a config file
	infile >> config.intensity_thresh >> config.num_of_markers >> config.useCameraInfo;

	// line 8-10: read camera's projection matrix.
	float p[12];
	for(int i=0; i<12; i++){
		infile >> p[i];
	}
	cv::Mat(3, 4, CV_32FC1, &p).copyTo(config.P);

	// line 11: maximum iteration
	infile >> config.MAX_ITERS;

	// line 12: initial rotation. Convert points coordinates in lidar to camera's -> camera coordinate to lidar. ZYX eular angle.
	for(int i = 0; i < 3; i++){
		infile >> angle;
		config.initialRot.push_back(angle);
		// std::cout << "Read angle i=" << i << ", angle: " << angle << std::endl;
	}
	
	// line 13: initial translation
	for(int i = 0; i < 3; i++){
		infile >> dist;
		config.initialTra.push_back(dist);
	}

	// line 14: lidar type. 2: lslidar.
	infile >> config.lidar_type;

	infile.close();
	config.print();
}

//------------- point cloud utils ---------------//
pcl::PointCloud<myPointXYZRID>::Ptr toMyPointXYZRID(const Hesai::PointCloud& point_cloud_ptr){
	pcl::PointCloud<myPointXYZRID>::Ptr new_cloud(new pcl::PointCloud<myPointXYZRID>);
	for (auto pt = point_cloud_ptr.points.begin(); pt < point_cloud_ptr.points.end(); pt++){	
        myPointXYZRID myPt;
        myPt.x = pt->x;
        myPt.y = pt->y;
        myPt.z = pt->z;
        myPt.intensity = pt->intensity;
        myPt.ring = pt->ring;
        myPt.range = pt->range;
	}
	return new_cloud;
}


pcl::PointCloud<myPointXYZRID>::Ptr lslidar2MyPointXYZRID(const lslidarPointCloud& point_cloud_ptr){
	pcl::PointCloud<myPointXYZRID>::Ptr new_cloud(new pcl::PointCloud<myPointXYZRID>);
	for (auto pt = point_cloud_ptr.points.begin(); pt < point_cloud_ptr.points.end(); pt++)	{
		myPointXYZRID myPt;
        myPt.x = pt->x;
        myPt.y = pt->y;
        myPt.z = pt->z;
		myPt.intensity = (float(pt->intensity))/255.0;		// TODO: changed to float type.

		float angle = atan(myPt.z / sqrt(myPt.x * myPt.x + myPt.y * myPt.y)) * 180 / M_PI;
        int scanID = int(angle + 17);
		// if (scanID > 16)		// reduce to line 0~16. 
		// 	continue;
		myPt.ring = scanID;
		myPt.range = (myPt.x * myPt.x + myPt.y * myPt.y + myPt.z * myPt.z);		//~ range is not used.
		if (myPt.x > 0 && myPt.x < 100 || myPt.x > -100 && myPt.x < 0)			// only save valid data. Avoid NaN.
			new_cloud->push_back(myPt);
		// cout << counter++ << ". Pos: (" << myPt.x << ", " << myPt.y << ", " << myPt.z << "), intensity: " << myPt.intensity << ", ring: " << myPt.ring << endl;
	}
	ROS_INFO_STREAM("Converted "<<new_cloud->points.size() <<" points from lslidarPoint to myPointXYZRID");
	return new_cloud;
}


pcl::PointCloud<pcl::PointXYZ>* toPointsXYZ(pcl::PointCloud<myPointXYZRID> point_cloud){
	pcl::PointCloud<pcl::PointXYZ> *new_cloud = new pcl::PointCloud<pcl::PointXYZ>();
	for (pcl::PointCloud<myPointXYZRID>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++){
		new_cloud->push_back(pcl::PointXYZ(pt->x, pt->y, pt->z));
	}
	return new_cloud;
}


pcl::PointCloud<myPointXYZRID> transform(pcl::PointCloud<myPointXYZRID> pc, float x, float y, float z, float rot_x, float rot_y, float rot_z){
	Eigen::Affine3f transf = pcl::getTransformation(x, y, z, rot_x, rot_y, rot_z);
	pcl::PointCloud<myPointXYZRID> new_cloud;
	pcl::transformPointCloud(pc, new_cloud, transf);
	return new_cloud;
}

//~ normalize between max(min, 0.0)~min(max, 1.0). Attention: avoid NaN data.
pcl::PointCloud<myPointXYZRID> normalizeIntensity(pcl::PointCloud<myPointXYZRID> point_cloud, float min, float max){
	float min_found = 10e6;
	float max_found = -10e6;
	for (pcl::PointCloud<myPointXYZRID>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++){
		max_found = MAX(max_found, pt->intensity);
		min_found = MIN(min_found, pt->intensity);
	}
	// ROS_INFO_STREAM("Max found: " << max_found << ", min found: " << min_found);
	for (pcl::PointCloud<myPointXYZRID>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++){
		pt->intensity = (pt->intensity - min_found) / (max_found - min_found) * (max - min) + min;
	}
	return point_cloud;
}


pcl::PointCloud<myPointXYZRID> intensityByRangeDiff(pcl::PointCloud<myPointXYZRID> point_cloud, config_settings config){
	std::vector<std::vector<myPointXYZRID*>> rings(32);			//~ 32 line lslidar
	for(pcl::PointCloud<myPointXYZRID>::iterator pt = point_cloud.points.begin() ; pt < point_cloud.points.end(); pt++){
		pt->range = (pt->x * pt->x + pt->y * pt->y + pt->z * pt->z);
		rings[pt->ring].push_back(&(*pt));
	}

	// assign a new intensity based on distance
	for(std::vector<std::vector<myPointXYZRID*>>::iterator ring = rings.begin(); ring < rings.end(); ring++){
		myPointXYZRID* prev, *succ;
		if (ring->empty())
			continue;
		//~ not used
		// float last_intensity = (*ring->begin())->intensity;
		// float new_intensity;									
		// (*ring->begin())->intensity = 0;
		// (*(ring->end() - 1))->intensity = 0;
		for (std::vector<myPointXYZRID*>::iterator pt = ring->begin() + 1; pt < ring->end() - 1; pt++){
			prev = *(pt - 1);
			succ = *(pt + 1);
			(*pt)->intensity = MAX( MAX( prev->range-(*pt)->range, succ->range-(*pt)->range), 0) * 10;
		}
	}
	point_cloud = normalizeIntensity(point_cloud, 0.0, 1.0);

	// ROS_WARN_STREAM("PointCloud size (before intensity filter): " << point_cloud.points.size());
	pcl::PointCloud<myPointXYZRID> filtered;
	//~ filter point cloud based on intensity range and xyz range.
	for (pcl::PointCloud<myPointXYZRID>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++){
		// cout << "No. " << counter++ << ", " << pt->intensity << endl;
		if (pt->intensity > config.intensity_thresh){
			if(pt->x >= config.xyz_[0].first && pt->x <= config.xyz_[0].second 
				&& pt->y >= config.xyz_[1].first && pt->y <= config.xyz_[1].second 
				&& pt->z >= config.xyz_[2].first && pt->z <= config.xyz_[2].second){
				filtered.push_back(*pt);			//~ save points in the ranges.
			}
		}
	}

	// ROS_INFO_STREAM("Find points in range: "
	// 				<< "   x:(" << config.xyz_[0].first << ", " << config.xyz_[0].second
	// 				<< "), y:(" << config.xyz_[1].first << ", " << config.xyz_[2].second
	// 				<< "), z:(" << config.xyz_[2].first << ", " << config.xyz_[2].second << "). ");
	// ROS_INFO_STREAM("Threshold Intensity: " << config.intensity_thresh);

	ROS_INFO_STREAM("Point Cloud size after intensityByDiff: "<<filtered.points.size());
	return filtered;
}
