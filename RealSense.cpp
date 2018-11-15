#include "realsense.h"

// Constructor
//RealSense::RealSense(bool enableChangeLaserPower, const std::string serial_number, const std::string friendly_name) :
//	serial_number(serial_number),
//	friendly_name(friendly_name),
//	enableChangeLaserPower(enableChangeLaserPower),
//	cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
//{
//	// Initialize
//	initialize();
//}

RealSense::RealSense(const rs2::device & device) :
	device(device),
	camera_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
	tip_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
	hand_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>)
{
	// Initialize
	initialize();

	//setLaserPower(range.min);

	nowTime = std::chrono::system_clock::now();
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

	nowTime = std::chrono::system_clock::now();

	// Initialize Sensor
	initializeSensor();
}

// Initialize Sensor
inline void RealSense::initializeSensor()
{
	serial_number = device.get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER);
	friendly_name = device.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME);

	//if (friendly_name == "Intel RealSense SR300")
	//	enableChangeLaserPower = true;

	range = device.query_sensors()[0].get_option_range(optionType);
	// Set Device Config
	rs2::config config;
	config.enable_device(serial_number);
	config.enable_stream(rs2_stream::RS2_STREAM_COLOR, color_width, color_height, rs2_format::RS2_FORMAT_BGR8, color_fps);
	config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, depth_width, depth_height, rs2_format::RS2_FORMAT_Z16, depth_fps);

	// Start Pipeline
	pipeline_profile = pipeline.start(config);

	std::cout << friendly_name << "(" + serial_number + ") enabled" << std::endl;
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
	setLaserPower(range.def);

	excuteUpdate();

	setLaserPower(range.min);
}

// Update Frame
inline void RealSense::updateFrame()
{
	// Update Frame
	frameset = pipeline.wait_for_frames();
}

inline void RealSense::excuteUpdate()
{
	prevTime = nowTime;
	nowTime = std::chrono::system_clock::now();

	auto def = nowTime - prevTime;

	fps = 1000 / std::chrono::duration<double, std::milli>(def).count();

	// Update Frame
	updateFrame();

	// Update Depth
	updateDepth();

	// Generate the pointcloud and texture mappings
	points = pc.calculate(depth_frame);

	// Update Color
	updateColor();

	//pointcloud生成
	pc.map_to(color_frame);
	camera_cloud_ptr = calcPointCloud(points);
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

	//auto sp = points.get_profile().as<rs2::video_stream_profile>();
	//cloud_temp->width = sp.width();
	//cloud_temp->height = sp.height();
	//cloud_temp->is_dense = false;
	//cloud_temp->points.resize(points.size());

	// Retrieve Vetrices
	auto vertices = points.get_vertices();

	// Retrieve Coordinated Texture
	auto texture_coordinates = points.get_texture_coordinates();

	// Create cv::Mat from Vertices and Texture
	vertices_mat = cv::Mat::zeros(depthSize, CV_32FC3);
	texture_mat = cv::Mat::zeros(depthSize, CV_8UC3);

	for (int32_t index = 0; index < points.size(); index++)
	{
		if (vertices[index].z)
		{
			// Set Vetices to cv::Mat
			pcl::PointXYZRGB point;
			auto vertex = vertices[index];
			vertices_mat.at<cv::Vec3f>(index) = cv::Vec3f(vertex.x, vertex.y, vertex.z);

			//mmをmに変換(符号はお好みで)
			point.x = -vertex.x;
			point.y = -vertex.y;
			point.z = vertex.z;

			// Set Texture to cv::Mat const
			auto texture_coordinate = texture_coordinates[index];
			uint32_t x = static_cast<uint32_t>(texture_coordinate.u * static_cast<float>(colorSize.width)); // [0.0, 1.0) -> [0, width)
			uint32_t y = static_cast<uint32_t>(texture_coordinate.v * static_cast<float>(colorSize.height)); // [0.0, 1.0) -> [0, height)
			if (x == 0 && y == 0)
				continue;
			if ((0 <= x) && (x < colorSize.width) && (0 <= y) && (y < colorSize.height))
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
	if (color_mat.empty())
	{
		return;
	}

	// Show Color Image
	cv::imshow("Color - " + friendly_name + " (" + serial_number + ")", color_mat);
}

// Show Depth
inline void RealSense::showDepth()
{
	if (depth_mat.empty())
	{
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

inline void RealSense::setLaserPower(float num)
{
	//if (!enableChangeLaserPower)
	//{
	//	std::cout << "ret-laser" << std::endl;
	//	return;
	//}
	auto sensor = device.query_sensors()[0];
	if (!sensor.supports(optionType))
	{
		//std::cout << "ret-option" << std::endl;
		return;
	}

	//const char* description = sensor.get_option_description(optionType);
	//std::cout << description << std::endl;

	//auto range = sensor.get_option_range(optionType);
	if (range.min <= num && range.max >= num)
	{
		//std::cout << "change" << std::endl;
		try
		{
			std::cout << friendly_name << "laser power changed" << std::endl;
			sensor.set_option(optionType, num);
		}
		catch (const rs2::error& e)
		{
			// Some options can only be set while the camera is streaming,
			// and generally the hardware might fail so it is good practice to catch exceptions from set_option
			std::cerr << friendly_name << "(" << serial_number << ") Failed to set option " << optionType << ". (" << e.what() << ")" << std::endl;
		}
	}
}

bool RealSense::saveData(std::string directory, std::string name)
{
	cv::Mat tmp;

	flip(color_mat, tmp, 1); // 反転
	cv::imwrite(directory + "-Color" + name + ".tif", tmp); // color画像保存
	flip(depth_mat, tmp, 1); // 反転
	imwrite(directory + "-Depth見る用" + name + ".tif", tmp * 0x60 / 0x100); // depth見る用画像保存
	writeDepth(directory + "-Depth" + name); // depth画像保存
	//if (isCloudArrived[CLOUD_CAMERA])
	if (camera_cloud_ptr->size() != 0)
		pcl::io::savePCDFileBinary(directory + "-PCLCamera" + name + ".pcd", *camera_cloud_ptr);
	//if (isCloudArrived[CLOUD_NEAR])
	/*if (near_point_cloud_ptr->size() != 0)
		pcl::io::savePCDFileBinary(directory + "-PCLNear" + name + ".pcd", *near_point_cloud_ptr);*/
		//if (tip_point_cloud_ptr->size() != 0)
		//	pcl::io::savePCDFileBinary(directory + "-PCLTip" + name + ".pcd", *tip_point_cloud_ptr);


	tmp = readDepth(directory + "-Depth" + name) * 0x60 / 0x10000;

	cv::imshow("保存済み", tmp); // 保存したものの表示
	return true;
}

cv::Mat RealSense::readDepth(const std::string name)
{
	std::fstream fs;

	std::string dir = name;

	// 拡張子がついているか
	std::string suffix = "." + extension;
	if (dir.size() < suffix.size() || dir.find(suffix, dir.size() - suffix.size()) == std::string::npos)
		dir += suffix;

	fs.open(dir, std::ios::in | std::ios::binary);
	if (!fs.is_open())
		throw std::runtime_error("depth32f画像の読み込みファイルの呼び出しに失敗");

	dptHeader header;

	if (!fs.read((char*)(&header.size), sizeof(header.size))
		|| !fs.read((char*)(&header.identifier), sizeof(header) - sizeof(header.size))
		|| !fs.seekg(header.size))
		throw std::runtime_error("depth32f画像のヘッダの読み込みに失敗");

	cv::Mat img(header.height, header.width, header.type);

	if (!fs.read((char*)(img.data), img.total() * img.elemSize()))
		throw std::runtime_error("depth32f画像のデータの読み込みに失敗");

	fs.close();

	if (enableMirror != (header.data1 & 1 << d1_mirror)) // ミラーするか
	{
		cv::flip(img.clone(), img, 1); // 反転
	}

	return img.clone();
}

void RealSense::writeDepth(const std::string name)
{
	cv::Mat matDepth = depth_mat;
	std::fstream fs;

	std::string dir = name;

	// 拡張子がついているか
	std::string suffix = "." + extension;
	if (dir.size() < suffix.size() || dir.find(suffix, dir.size() - suffix.size()) == std::string::npos)
		dir += suffix;

	fs.open(dir, std::ios::out | std::ios::binary);

	if (!fs.is_open())
		throw std::runtime_error("depth32f画像の書き込みファイルの呼び出しに失敗");

	if (enableMirror != isSaveMirror) // ミラーするか
	{
		cv::flip(matDepth.clone(), matDepth, 1); // 反転
	}

	dptHeader header;
	header.width = matDepth.cols;
	header.height = matDepth.rows;
	header.type = matDepth.type();
	if (isSaveMirror) // ミラーしているか
	{
		header.data1 |= 1 << d1_mirror; // ヘッダにミラー情報書き込み
	}

	if (!fs.write((const char*)(&header), sizeof(header)) // ヘッダ書き込み
		|| !fs.seekp(header.size) // ポインタ位置移動
		|| !fs.write((const char*)(matDepth.data), matDepth.total() * matDepth.elemSize())) // データ書き込み
		throw std::runtime_error("depth32f画像の書き込みに失敗");

	fs.close();
}

void RealSense::setTipCloud()
{
	HandDetect det(0.2, 0.6);
	cv::Mat colorMappedToDepthTemp = texture_mat.clone();
	cv::Mat tipPosMat = cv::Mat::zeros(depthSize, CV_8U);

	std::vector<cv::Point> tipPos = det.getTipData(depth_mat.clone(), texture_mat.clone());
	//colorMappedToDepth = det.colorMarked.clone();
	cv::Mat handMask = det.contourMask.clone();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tip_cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (tipPos[0] == cv::Point(-1, -1))
	{
		tip_cloud_ptr = tip_cloud_temp;
		hand_cloud_ptr = hand_cloud_temp;
		return;
	}

	for (int i = 0; i < tipPos.size(); i++)
	{
		uchar *tipPosMatPtr = tipPosMat.ptr<uchar>(tipPos[i].y);
		tipPosMatPtr[tipPos[i].x] = 255;
	}

	for (int y = 0; y < depth_mat.rows; y++)
	{
		cv::Vec3f *vertices_mat_ptr = vertices_mat.ptr<cv::Vec3f>(y);
		uchar *handMaskPtr = handMask.ptr<uchar>(y);
		cv::Vec3b *colorMappedToDepthTempPtr = colorMappedToDepthTemp.ptr<cv::Vec3b>(y);
		uchar* tipPosMatPtr = tipPosMat.ptr<uchar>(y);

		for (int x = 0; x < depth_mat.cols; x++)
		{
			if (tipPosMatPtr[x] != 255 && handMaskPtr[x] != 255)
				continue;
			pcl::PointXYZRGB point;//Unit:m

			point.x = -vertices_mat_ptr[x][0];
			point.y = -vertices_mat_ptr[x][1];
			point.z = vertices_mat_ptr[x][2];
			if (handMaskPtr[x] == 255)
			{
				point.r = colorMappedToDepthTempPtr[x][2];
				point.g = colorMappedToDepthTempPtr[x][1];
				point.b = colorMappedToDepthTempPtr[x][0];

				hand_cloud_temp->points.push_back(point);
			}
			if (tipPosMatPtr[x] == 255)
			{
				point.r = 255;
				point.g = 0;
				point.b = 0;

				tip_cloud_temp->points.push_back(point);
			}
		}
	}

	tip_cloud_ptr = tip_cloud_temp;
	hand_cloud_ptr = hand_cloud_temp;
}

SR300::SR300(const rs2::device & device) :
	RealSense(device)
{
	setLaserPower(range.min);
}

SR300::~SR300()
{
	//RealSense::finalize();
}

void SR300::update()
{
	setLaserPower(range.def);

	std::this_thread::sleep_for(std::chrono::milliseconds(80));

	excuteUpdate();

	setLaserPower(range.min);
}

D400::D400(const rs2::device & device) :
	RealSense(device)
{
}

D400::~D400()
{
	//RealSense::finalize();
}

void D400::update()
{
	excuteUpdate();
}
