#include "realsense.h"

// Constructor
RealSense::RealSense(const std::string serial_number, const std::string friendly_name) :
	serial_number(serial_number),
	friendly_name(friendly_name),
	cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
	// Initialize
	initialize();
}

// Destructor
RealSense::~RealSense()
{
	// Finalize
	finalize();
}

// Initialize
void RealSense::initialize()
{
	cv::setUseOptimized(true);

	// Initialize Sensor
	initializeSensor();
}

// Initialize Sensor
inline void RealSense::initializeSensor()
{
	// Set Device Config
	rs2::config config;
	config.enable_device(serial_number);
	config.enable_stream(rs2_stream::RS2_STREAM_COLOR, color_width, color_height, rs2_format::RS2_FORMAT_BGR8, color_fps);
	config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, depth_width, depth_height, rs2_format::RS2_FORMAT_Z16, depth_fps);

	// Start Pipeline
	pipeline_profile = pipeline.start(config);
}

// Finalize
void RealSense::finalize()
{
	// Close Windows
	cv::destroyAllWindows();

	// Stop Pipline
	pipeline.stop();
}

// Update Data
void RealSense::update()
{
	// Update Frame
	updateFrame();

	// Update Depth
	updateDepth();
	// Generate the pointcloud and texture mappings
	points = pc.calculate(depth_frame);

	// Update Color
	updateColor();

	//pointcloudê∂ê¨
	pc.map_to(color_frame);
	cloud = calcPointCloud(points);
}

// Update Frame
inline void RealSense::updateFrame()
{
	// Update Frame
	frameset = pipeline.wait_for_frames();
}

// Update Color
inline void RealSense::updateColor()
{
	// Retrieve Color Flame
	color_frame = frameset.get_color_frame();

	// Retrive Frame Size
	color_width = color_frame.as<rs2::video_frame>().get_width();
	color_height = color_frame.as<rs2::video_frame>().get_height();
}

// Update Depth
inline void RealSense::updateDepth()
{
	// Retrieve Depth Flame
	depth_frame = frameset.get_depth_frame();

	// Retrive Frame Size
	depth_width = depth_frame.as<rs2::video_frame>().get_width();
	depth_height = depth_frame.as<rs2::video_frame>().get_height();
}

// Draw Data
void RealSense::draw()
{
	// Draw Color
	drawColor();

	// Draw Depth
	drawDepth();
}

// Draw Color
inline void RealSense::drawColor()
{
	// Create cv::Mat form Color Frame
	color_mat = cv::Mat(color_height, color_width, CV_8UC3, const_cast<void*>(color_frame.get_data()));
}

// Draw Depth
inline void RealSense::drawDepth()
{
	// Create cv::Mat form Depth Frame
	depth_mat = cv::Mat(depth_height, depth_width, CV_16SC1, const_cast<void*>(depth_frame.get_data()));
}

// Show Data
void RealSense::show()
{
	// Show Color
	showColor();

	// Show Depth
	showDepth();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RealSense::calcPointCloud(const rs2::points & points)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud_temp->width = sp.width();
	cloud_temp->height = sp.height();
	cloud_temp->is_dense = false;
	cloud_temp->points.resize(points.size());

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

			//mmÇmÇ…ïœä∑(ïÑçÜÇÕÇ®çDÇ›Ç≈)
			point.x = -vertex.x / 1000;
			point.y = -vertex.y / 1000;
			point.z = vertex.z / 1000;

			// Set Texture to cv::Mat const
			auto texture_coordinate = texture_coordinates[index];
			uint32_t x = static_cast<uint32_t>(texture_coordinate.u * static_cast<float>(colorSize.width)); // [0.0, 1.0) -> [0, width)
			uint32_t y = static_cast<uint32_t>(texture_coordinate.v * static_cast<float>(colorSize.height)); // [0.0, 1.0) -> [0, height)
			if ((0 <= x) && (x < colorSize.width) && (0 <= y) && (y < colorSize.height) && !(x == 0 && y == 0))
			{
				texture_mat.at<cv::Vec3b>(index) = color_mat.at<cv::Vec3b>(y, x);
				point.r = color_mat.at<cv::Vec3b>(y, x)[2];
				point.g = color_mat.at<cv::Vec3b>(y, x)[1];
				point.b = color_mat.at<cv::Vec3b>(y, x)[0];
			}
			else
			{
				point.r = 255;
				point.g = 255;
				point.b = 255;
			}
			cloud_temp->points.push_back(point);
		}
	}

	return cloud_temp;
}

// Show Color
inline void RealSense::showColor()
{
	if (color_mat.empty()) {
		return;
	}

	// Show Color Image
	cv::imshow("Color - " + friendly_name + " (" + serial_number + ")", color_mat);
}

// Show Depth
inline void RealSense::showDepth()
{
	if (depth_mat.empty()) {
		return;
	}

	// Scaling
	cv::Mat scale_mat;
	//depth_mat.convertTo(scale_mat, CV_8U, -255.0 / 10000.0, 255.0); // 0-10000 -> 255(white)-0(black)
	depth_mat.convertTo(scale_mat, CV_8U, 255.0 / 10000.0, 0.0); // 0-10000 -> 0(black)-255(white)

	// Apply False Colour
	//cv::applyColorMap( scale_mat, scale_mat, cv::COLORMAP_BONE );

	// Show Depth Image
	cv::imshow("Depth - " + friendly_name + " (" + serial_number + " )", scale_mat);
}