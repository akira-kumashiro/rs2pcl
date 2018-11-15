// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

//#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
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

//#include <opencv2\opencv.hpp>
//#include <opencv2/core/core_c.h>
//#include <opencv/highgui.h>
//#include <opencv\cv.h>
//#include <opencv2\highgui\highgui.hpp>

#include "PCL_Grabber.h";

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

//class PCL_Grabber_1
//{
//public:
//	//PCL_Grabber();
//	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr run(void);
//	//cv::Size getFrameSize(const rs2::depth_frame frame);
//	//cv::Size getFrameSize(const rs2::video_frame frame);
//private:
//	// Declare pointcloud object, for calculating pointclouds and texture mappings
//	rs2::pointcloud pc;
//
//	// We want the points object to be persistent so we can display the last cloud when a frame drops
//	rs2::points points;
//	// Declare RealSense pipeline, encapsulating the actual device and sensors
//	rs2::pipeline pipe;
//
//	cv::Mat color_mat, depth_mat;
//	cv::Size colorSize, depthSize;
//
//	using pcl_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;
//
//
//public:
//	PCL_Grabber_1()
//	{
//		// Create a simple OpenGL window for rendering:
//		//window app(1280, 720, "RealSense PCL Pointcloud Example");
//		// Construct an object to manage view state
//		//glfw_state app_state;
//		// register callbacks to allow manipulation of the pointcloud
//		//register_glfw_callbacks(app, app_state);
//
//
//		// Start streaming with default recommended configuration
//		pipe.start();
//
//		wprintf_s(L"start\n");
//	}
//
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr run(void)
//	{
//		// Wait for the next set of frames from the camera
//		auto frames = pipe.wait_for_frames();
//
//		auto depth = frames.get_depth_frame();
//		depthSize = getFrameSize(depth);
//
//		depth_mat = cv::Mat(depthSize, CV_16SC1, const_cast<void*>(depth.get_data()));
//
//		wprintf_s(L"depth:%dx%d\n", depthSize.width, depthSize.height);
//
//		// Generate the pointcloud and texture mappings
//		points = pc.calculate(depth);
//
//		auto color = frames.get_color_frame();
//		colorSize = getFrameSize(color);
//
//		color_mat = cv::Mat(colorSize, CV_8UC3, const_cast<void*>(color.get_data()));
//
//		wprintf_s(L"color:%dx%d\n", colorSize.width, colorSize.height);
//
//		pc.map_to(color);
//
//		//app_state.tex.upload(color);
//
//		auto pcl_points = points_to_pcl(points);
//
//		return pcl_points;
//		//pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//		//pcl::PassThrough<pcl::PointXYZ> pass;
//		//pass.setInputCloud(pcl_points);
//		//pass.setFilterFieldName("z");
//		//pass.setFilterLimits(0.0, 1.0);
//		//pass.filter(*cloud_filtered);
//
//		//std::vector<pcl_ptr> layers;
//		//layers.push_back(pcl_points);
//		//layers.push_back(cloud_filtered);
//
//
//		//draw_pointcloud(app, app_state, layers);
//	}
//
//	// Struct for managing rotation of pointcloud view
//	//struct state {
//	//	state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
//	//		ml(false), offset_x(0.0f), offset_y(0.0f) {}
//	//	double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y;
//	//};
//
//
//	// Helper functions
//	//void register_glfw_callbacks(window& app, state& app_state);
//	//void draw_pointcloud(window& app, glfw_state& app_state, const std::vector<pcl_ptr>& points);
//
//	// Draw Color
//	void drawColor()
//	{
//		// Create cv::Mat form Color Frame
//		
//	}
//
//	// Draw Depth
//	void drawDepth()
//	{
//		// Create cv::Mat form Depth Frame
//		
//	}
//
//	pcl_ptr points_to_pcl(const rs2::points& points)
//	{
//		pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//
//		auto sp = points.get_profile().as<rs2::video_stream_profile>();
//		cloud->width = sp.width();
//		cloud->height = sp.height();
//		cloud->is_dense = false;
//		cloud->points.resize(points.size());
//		//auto ptr = points.get_vertices();
//		//auto tex_coords = points.get_texture_coordinates();
//
//
//			// Retrieve Vetrices
//		const rs2::vertex* vertices = points.get_vertices();
//
//		// Retrieve Coordinated Texture
//		const rs2::texture_coordinate* texture_coordinates = points.get_texture_coordinates();
//
//
//		// Create cv::Mat from Vertices and Texture
//		//cv::Mat vertices_mat = cv::Mat(depth_height, depth_width, CV_32FC3, cv::Vec3f::all(std::numeric_limits<float>::quiet_NaN()));
//		//cv::Mat texture_mat = cv::Mat(depth_height, depth_width, CV_8UC3, cv::Vec3b::all(0));
//
//		cv::Mat vertices_mat = cv::Mat::zeros(depthSize, CV_32FC3);//cv::Mat(depthSize, CV_32FC3, cv::Scalar(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()));
//		cv::Mat texture_mat = cv::Mat::zeros(depthSize, CV_8UC3);
//
//		for (int32_t index = 0; index < points.size(); index++)
//		{
//			if (vertices[index].z)
//			{
//				// Set Vetices to cv::Mat
//				pcl::PointXYZRGB point;
//				const rs2::vertex vertex = vertices[index];
//				vertices_mat.at<cv::Vec3f>(index) = cv::Vec3f(vertex.x, vertex.y, vertex.z);
//
//				point.x = -vertex.x;
//				point.y = -vertex.y;
//				point.z = vertex.z;
//
//				// Set Texture to cv::Mat const
//				rs2::texture_coordinate texture_coordinate = texture_coordinates[index];
//				uint32_t x = static_cast<uint32_t>(texture_coordinate.u * static_cast<float>(colorSize.width)); // [0.0, 1.0) -> [0, width)
//				uint32_t y = static_cast<uint32_t>(texture_coordinate.v * static_cast<float>(colorSize.height)); // [0.0, 1.0) -> [0, height)
//				//wprintf_s(L"point:(u,v)=%lf,%lf (x,y)=%d,%d\n", texture_coordinate.u, texture_coordinate.v, x, y);
//				if ((0 <= x) && (x < colorSize.width) && (0 <= y) && (y < colorSize.height)&& !(x==0&&y==0))
//				{
//					texture_mat.at<cv::Vec3b>(index) = color_mat.at<cv::Vec3b>(y, x);
//					point.r = color_mat.at<cv::Vec3b>(y, x)[0];
//					point.g = color_mat.at<cv::Vec3b>(y, x)[1];
//					point.b = color_mat.at<cv::Vec3b>(y, x)[2];
//					/*point.r = texture_mat.at<cv::Vec3b>(index)[2];
//					point.g = texture_mat.at<cv::Vec3b>(index)[1];
//					point.b = texture_mat.at<cv::Vec3b>(index)[0];
//					point.r = 255;
//					point.g = 0;
//					point.b = 0;*/
//
//				}
//				else
//				{
//					point.r = 255;
//					point.g = 255;
//					point.b = 255;
//				}
//				cloud->points.push_back(point);
//			}
//		}
//
//		cv::imshow("img", texture_mat);
//
//		//for (auto& p : cloud->points)
//		//{
//		//	p.x = ptr->x;
//		//	p.y = ptr->y;
//		//	p.z = ptr->z;
//		//	//色の付け方がわからん
//		//	ptr++;
//		//}
//
//		return cloud;
//	}
//
//	//float3 colors[]{ { 0.8f, 0.1f, 0.3f },
//	//				  { 0.1f, 0.9f, 0.5f },
//	//};
//
//	cv::Size getFrameSize(const rs2::depth_frame frame)
//	{
//		return cv::Size(frame.as<rs2::video_frame>().get_width(), frame.as<rs2::video_frame>().get_height());
//	}
//	cv::Size getFrameSize(const rs2::video_frame frame)
//	{
//		return cv::Size(frame.as<rs2::video_frame>().get_width(), frame.as<rs2::video_frame>().get_height());
//	}
//
//
//	// Registers the state variable and callbacks to allow mouse control of the pointcloud
//	//void register_glfw_callbacks(window& app, state& app_state)
//	//{
//	//	app.on_left_mouse = [&](bool pressed)
//	//	{
//	//		app_state.ml = pressed;
//	//	};
//
//	//	app.on_mouse_scroll = [&](double xoffset, double yoffset)
//	//	{
//	//		app_state.offset_x += static_cast<float>(xoffset);
//	//		app_state.offset_y += static_cast<float>(yoffset);
//	//	};
//
//	//	app.on_mouse_move = [&](double x, double y)
//	//	{
//	//		if (app_state.ml)
//	//		{
//	//			app_state.yaw -= (x - app_state.last_x);
//	//			app_state.yaw = std::max(app_state.yaw, -120.0);
//	//			app_state.yaw = std::min(app_state.yaw, +120.0);
//	//			app_state.pitch += (y - app_state.last_y);
//	//			app_state.pitch = std::max(app_state.pitch, -80.0);
//	//			app_state.pitch = std::min(app_state.pitch, +80.0);
//	//		}
//	//		app_state.last_x = x;
//	//		app_state.last_y = y;
//	//	};
//
//	//	app.on_key_release = [&](int key)
//	//	{
//	//		if (key == 32) // Escape
//	//		{
//	//			app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
//	//		}
//	//	};
//	//}
//
//	// Handles all the OpenGL calls needed to display the point cloud
//	//void draw_pointcloud(window& app, glfw_state& app_state, const std::vector<pcl_ptr>& points)
//	//{
//	//	// OpenGL commands that prep screen for the pointcloud
//	//	glPopMatrix();
//	//	glPushAttrib(GL_ALL_ATTRIB_BITS);
//
//	//	float width = app.width(), height = app.height();
//
//	//	glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
//	//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//	//	glMatrixMode(GL_PROJECTION);
//	//	glPushMatrix();
//	//	gluPerspective(60, width / height, 0.01f, 10.0f);
//
//	//	glMatrixMode(GL_MODELVIEW);
//	//	glPushMatrix();
//	//	gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);
//
//	//	glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
//	//	glRotated(app_state.pitch, 1, 0, 0);
//	//	glRotated(app_state.yaw, 0, 1, 0);
//	//	glTranslatef(0, 0, -0.5f);
//
//	//	glPointSize(width / 640);
//	//	glEnable(GL_TEXTURE_2D);
//
//	//	int color = 0;
//
//	//	for (auto&& pc : points)
//	//	{
//	//		auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];
//
//	//		glBegin(GL_POINTS);
//	//		glColor3f(c.x, c.y, c.z);
//
//	//		/* this segment actually prints the pointcloud */
//	//		for (int i = 0; i < pc->points.size(); i++)
//	//		{
//	//			auto&& p = pc->points[i];
//	//			if (p.z)
//	//			{
//	//				// upload the point and texture coordinates only for points we have depth data for
//	//				glVertex3f(p.x, p.y, p.z);
//	//			}
//	//		}
//
//	//		glEnd();
//	//	}
//
//	//	// OpenGL cleanup
//	//	glPopMatrix();
//	//	glMatrixMode(GL_PROJECTION);
//	//	glPopMatrix();
//	//	glPopAttrib();
//	//	glPushMatrix();
//	//}
//};

//int main(int argc, char * argv[]) try
//{
//	PCL_Grabber grabber;
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//
//	viewer->addPointCloud(cloud, "cloud");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "cloud");
//
//	while (!viewer->wasStopped()) // Application still alive?
//	{
//		cloud = grabber.getPointCloud();
//		viewer->updatePointCloud(cloud, "cloud");
//		viewer->spinOnce();
//	}
//
//	return EXIT_SUCCESS;
//}
//catch (const rs2::error & e)
//{
//	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
//	return EXIT_FAILURE;
//}
//catch (const std::exception & e)
//{
//	std::cerr << e.what() << std::endl;
//	return EXIT_FAILURE;
//}

#include "multirealsense.h"

int main(int argc, char* argv[])
{
	try 
	{
		MultiRealSense multirealsense;
		multirealsense.run();
	}
	catch (rs2::error &e)
	{
		std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	}
	catch (std::exception& ex) 
	{
		std::cout << ex.what() << std::endl;
	}

	return 0;
}

/*
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "../../../examples/example.hpp" // Include short list of convenience functions for rendering

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>

// Struct for managing rotation of pointcloud view
struct state {
	state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
		ml(false), offset_x(0.0f), offset_y(0.0f) {}
	double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y;
};

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

// Helper functions
void register_glfw_callbacks(window& app, state& app_state);
void draw_pointcloud(window& app, glfw_state& app_state, const std::vector<pcl_ptr>& points);

pcl_ptr points_to_pcl(const rs2::points& points)
{
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto ptr = points.get_vertices();
	for (auto& p : cloud->points)
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}

	return cloud;
}

float3 colors[]{ { 0.8f, 0.1f, 0.3f },
				  { 0.1f, 0.9f, 0.5f },
};

int main(int argc, char * argv[]) try
{
	// Create a simple OpenGL window for rendering:
	window app(1280, 720, "RealSense PCL Pointcloud Example");
	// Construct an object to manage view state
	glfw_state app_state;
	// register callbacks to allow manipulation of the pointcloud
	register_glfw_callbacks(app, app_state);

	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;
	// Start streaming with default recommended configuration
	pipe.start();
	while (app) // Application still alive?
	{
		// Wait for the next set of frames from the camera
		auto frames = pipe.wait_for_frames();

		auto depth = frames.get_depth_frame();

		// Generate the pointcloud and texture mappings
		points = pc.calculate(depth);

		auto color = frames.get_color_frame();

		pc.map_to(color);

		app_state.tex.upload(color);

		auto pcl_points = points_to_pcl(points);

		pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(pcl_points);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, 1.0);
		pass.filter(*cloud_filtered);

		std::vector<pcl_ptr> layers;
		layers.push_back(pcl_points);
		//layers.push_back(cloud_filtered);


		draw_pointcloud(app, app_state, layers);
	}

	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

// Registers the state variable and callbacks to allow mouse control of the pointcloud
void register_glfw_callbacks(window& app, state& app_state)
{
	app.on_left_mouse = [&](bool pressed)
	{
		app_state.ml = pressed;
	};

	app.on_mouse_scroll = [&](double xoffset, double yoffset)
	{
		app_state.offset_x += static_cast<float>(xoffset);
		app_state.offset_y += static_cast<float>(yoffset);
	};

	app.on_mouse_move = [&](double x, double y)
	{
		if (app_state.ml)
		{
			app_state.yaw -= (x - app_state.last_x);
			app_state.yaw = std::max(app_state.yaw, -120.0);
			app_state.yaw = std::min(app_state.yaw, +120.0);
			app_state.pitch += (y - app_state.last_y);
			app_state.pitch = std::max(app_state.pitch, -80.0);
			app_state.pitch = std::min(app_state.pitch, +80.0);
		}
		app_state.last_x = x;
		app_state.last_y = y;
	};

	app.on_key_release = [&](int key)
	{
		if (key == 32) // Escape
		{
			app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
		}
	};
}

// Handles all the OpenGL calls needed to display the point cloud
void draw_pointcloud(window& app, glfw_state& app_state, const std::vector<pcl_ptr>& points)
{
	// OpenGL commands that prep screen for the pointcloud
	glPopMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	float width = app.width(), height = app.height();

	glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	gluPerspective(60, width / height, 0.01f, 10.0f);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

	glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
	glRotated(app_state.pitch, 1, 0, 0);
	glRotated(app_state.yaw, 0, 1, 0);
	glTranslatef(0, 0, -0.5f);

	glPointSize(width / 640);
	glEnable(GL_TEXTURE_2D);

	int color = 0;

	for (auto&& pc : points)
	{
		auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];

		glBegin(GL_POINTS);
		glColor3f(c.x, c.y, c.z);

		// this segment actually prints the pointcloud
		for (int i = 0; i < pc->points.size(); i++)
		{
			auto&& p = pc->points[i];
			if (p.z)
			{
				// upload the point and texture coordinates only for points we have depth data for
				glVertex3f(p.x, p.y, p.z);
			}
		}

		glEnd();
	}

	// OpenGL cleanup
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
	glPushMatrix();
}

*/