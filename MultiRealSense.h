#ifndef __MULTIREALSENSE__
#define __MULTIREALSENSE__

#include "realsense.h"

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <memory>

class MultiRealSense
{
private:
	// RealSense
	std::vector<std::unique_ptr<RealSense>> realsenses;

	//pcl‚Ìviewer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

public:
	// Constructor
	MultiRealSense();

	// Destructor
	~MultiRealSense();

	// Processing
	void run();

private:
	// Initialize
	void initialize();

	// Initialize Sensor ƒZƒ“ƒT[‚Ì“o˜^
	inline void initializeSensor(const rs2::device& device);

	//pcl‚Ìviewer‚Ì‰Šúİ’è
	inline void initializeViewer();

	// Finalize
	void finalize();
};

#endif // __MULTIREALSENSE__