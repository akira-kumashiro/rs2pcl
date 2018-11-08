#include "multirealsense.h"

// Constructor
MultiRealSense::MultiRealSense() :
	viewer(new pcl::visualization::PCLVisualizer("3D Viewer"))//viewerのコンストラクタ
{
	// Initialize
	initialize();
}

// Destructor
MultiRealSense::~MultiRealSense()
{
	// Finalize
	finalize();
}

// Processing
void MultiRealSense::run()
{
	// Main Loop
	while (true) {
		for (std::unique_ptr<RealSense>& realsense : realsenses) {
			// Update Data
			realsense->update();

			// Draw Data
			realsense->draw();

			// Show Data
			realsense->show();

			//viewerに生成したpointcloudを渡して更新
			viewer->updatePointCloud(realsense->cloud, realsense->cloudName);
			viewer->spinOnce();
		}

		// Key Check
		const int32_t key = cv::waitKey(10);
		if (key == 'q') {
			break;
		}
	}
}

// Initialize
void MultiRealSense::initialize()
{
	cv::setUseOptimized(true);

	// Retrive Connected Sensors List
	rs2::context context;
	const rs2::device_list device_list = context.query_devices();
	
	// Initialize Connected Sensors
	for (const rs2::device& device : device_list) {
		// Check Device
		// "Platform Camera" is not RealSense Devices
		const std::string friendly_name = device.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME);
		if (friendly_name == "Platform Camera") {
			continue;
		}

		// Initialize Sensor
		initializeSensor(device);
	}

	//PCLのviewerの初期化
	initializeViewer();
}

// Initialize Sensor
inline void MultiRealSense::initializeSensor(const rs2::device& device)
{
	// Retrive Serial Number (and Friendly Name)
	const std::string serial_number = device.get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER);
	const std::string friendly_name = device.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME);

	// Add Sensor to Container
	realsenses.push_back(std::make_unique<RealSense>(serial_number, friendly_name));
}

inline void MultiRealSense::initializeViewer()
{
	for (auto i = 0; i < realsenses.size(); i++)
	{
		realsenses[i]->cloudName = "cloud" + std::to_string(i);
		viewer->addPointCloud(realsenses[i]->cloud, realsenses[i]->cloudName);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, realsenses[i]->cloudName);
	}
}

// Finalize
void MultiRealSense::finalize()
{
	// Close Windows
	cv::destroyAllWindows();
	viewer->close();
}