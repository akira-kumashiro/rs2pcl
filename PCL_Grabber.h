#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//#include "example.hpp" // Include short list of convenience functions for rendering

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/filters/passthrough.h>

//#include <boost/format.hpp>
//#include <boost/shared_ptr.hpp>
//#include <boost/thread/thread.hpp>
//#include <pcl/common/common_headers.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/console/parse.h>

#include <opencv2\opencv.hpp>
#include <opencv2/core/core_c.h>
#include <opencv/highgui.h>
#include <opencv\cv.h>
#include <opencv2\highgui\highgui.hpp>

#ifdef _DEBUG
//Debugモードの場合
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_core2411d.lib")
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_imgproc2411d.lib")
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_highgui2411d.lib")
#else
//Releaseモードの場合
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_core2411.lib")
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_imgproc2411.lib")
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_highgui2411.lib")
#endif

class PCL_Grabber
{
public:
	PCL_Grabber();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud(rs2::pipeline &pipe);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr calcPointCloud(const rs2::points& points);


	~PCL_Grabber();
private:
	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;

	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;
	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline _pipe;

	cv::Mat color_mat, depth_mat;
	cv::Size colorSize, depthSize;

	cv::Size getFrameSize(const rs2::depth_frame frame);
	cv::Size getFrameSize(const rs2::video_frame frame);
};

