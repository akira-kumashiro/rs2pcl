#ifndef __REALSENSE__
#define __REALSENSE__

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <string>

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

//#include <opencv2\opencv.hpp>
//#include <opencv2/core/core_c.h>
//#include <opencv/highgui.h>
//#include <opencv\cv.h>
//#include <opencv2\highgui\highgui.hpp>
//
//#ifdef _DEBUG
////Debugモードの場合
//#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_core2411d.lib")
//#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_imgproc2411d.lib")
//#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_highgui2411d.lib")
//#else
////Releaseモードの場合
//#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_core2411.lib")
//#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_imgproc2411.lib")
//#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_highgui2411.lib")
//#endif

class RealSense
{
private:
	// RealSense
	rs2::pipeline pipeline;
	rs2::pipeline_profile pipeline_profile;
	rs2::frameset frameset;
	std::string serial_number;
	std::string friendly_name;

	// Color Buffer
	rs2::frame color_frame;
	cv::Mat color_mat;
	uint32_t color_width = 1280;
	uint32_t color_height = 720;
	uint32_t color_fps = 30;
	cv::Size colorSize = cv::Size(color_width, color_height);

	// Depth Buffer
	rs2::frame depth_frame;
	cv::Mat depth_mat;
	uint32_t depth_width = 640;
	uint32_t depth_height = 480;
	uint32_t depth_fps = 30;
	cv::Size depthSize = cv::Size(depth_width, depth_height);

	//pcとpointsはlibrealsense独自規格の点群データ関連
	//openGLでは使いやすいらしい
	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;

	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;
	   
public:
	// Constructor
	RealSense(const std::string serial_number, const std::string friendly_name = "");

	// Destructor
	~RealSense();

	// Update Data
	void update();

	// Draw Data
	void draw();

	// Show Data
	void show();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr calcPointCloud(const rs2::points& points);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	std::string cloudName;
private:
	// Initialize
	void initialize();

	// Initialize Sensor
	inline void initializeSensor();

	// Finalize
	void finalize();

	// Update Frame
	inline void updateFrame();

	// Update Color
	inline void updateColor();

	// Update Depth
	inline void updateDepth();

	// Draw Color
	inline void drawColor();

	// Draw Depth
	inline void drawDepth();

	// Show Color
	inline void showColor();

	// Show Depth
	inline void showDepth();
};

#endif // __REALSENSE__