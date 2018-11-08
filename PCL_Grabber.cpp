#include "PCL_Grabber.h"



PCL_Grabber::PCL_Grabber()
{
	// Start streaming with default recommended configuration
	//pipe.start();

	//wprintf_s(L"start\n");
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCL_Grabber::getPointCloud(rs2::pipeline &pipe)
{
	_pipe = pipe;

	// Wait for the next set of frames from the camera
	auto frames = _pipe.wait_for_frames();

	auto depth = frames.get_depth_frame();
	depthSize = getFrameSize(depth);

	depth_mat = cv::Mat(depthSize, CV_16SC1, const_cast<void*>(depth.get_data()));

	wprintf_s(L"depth:%dx%d\n", depthSize.width, depthSize.height);

	// Generate the pointcloud and texture mappings
	points = pc.calculate(depth);

	auto color = frames.get_color_frame();
	colorSize = getFrameSize(color);

	color_mat = cv::Mat(colorSize, CV_8UC3, const_cast<void*>(color.get_data()));

	wprintf_s(L"color:%dx%d\n", colorSize.width, colorSize.height);

	pc.map_to(color);

	//app_state.tex.upload(color);

	auto pcl_points = calcPointCloud(points);

	return pcl_points;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCL_Grabber::calcPointCloud(const rs2::points & points)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	// Retrieve Vetrices
	auto vertices = points.get_vertices();

	// Retrieve Coordinated Texture
	auto texture_coordinates = points.get_texture_coordinates();

	// Create cv::Mat from Vertices and Texture
	cv::Mat vertices_mat = cv::Mat::zeros(depthSize, CV_32FC3);
	cv::Mat texture_mat = cv::Mat::zeros(depthSize, CV_8UC3);

	for (int32_t index = 0; index < points.size(); index++)
	{
		if (vertices[index].z)
		{
			// Set Vetices to cv::Mat
			pcl::PointXYZRGB point;
			auto vertex = vertices[index];
			vertices_mat.at<cv::Vec3f>(index) = cv::Vec3f(vertex.x, vertex.y, vertex.z);

			point.x = -vertex.x;
			point.y = -vertex.y;
			point.z = vertex.z;

			// Set Texture to cv::Mat const
			auto texture_coordinate = texture_coordinates[index];
			uint32_t x = static_cast<uint32_t>(texture_coordinate.u * static_cast<float>(colorSize.width)); // [0.0, 1.0) -> [0, width)
			uint32_t y = static_cast<uint32_t>(texture_coordinate.v * static_cast<float>(colorSize.height)); // [0.0, 1.0) -> [0, height)
			//wprintf_s(L"point:(u,v)=%lf,%lf (x,y)=%d,%d\n", texture_coordinate.u, texture_coordinate.v, x, y);
			if ((0 <= x) && (x < colorSize.width) && (0 <= y) && (y < colorSize.height) && !(x == 0 && y == 0))
			{
				texture_mat.at<cv::Vec3b>(index) = color_mat.at<cv::Vec3b>(y, x);
				point.r = color_mat.at<cv::Vec3b>(y, x)[0];
				point.g = color_mat.at<cv::Vec3b>(y, x)[1];
				point.b = color_mat.at<cv::Vec3b>(y, x)[2];
			}
			else
			{
				point.r = 255;
				point.g = 255;
				point.b = 255;
			}
			cloud->points.push_back(point);
		}
	}
	//cv::imshow("img", texture_mat);

	return cloud;
}


PCL_Grabber::~PCL_Grabber()
{
}

cv::Size PCL_Grabber::getFrameSize(const rs2::depth_frame frame)
{
	return cv::Size(frame.as<rs2::video_frame>().get_width(), frame.as<rs2::video_frame>().get_height());
}

cv::Size PCL_Grabber::getFrameSize(const rs2::video_frame frame)
{
	return cv::Size(frame.as<rs2::video_frame>().get_width(), frame.as<rs2::video_frame>().get_height());
}
