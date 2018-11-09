#ifndef __MULTIREALSENSE__
#define __MULTIREALSENSE__

#include "realsense.h"

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <memory>

#include <windows.h>

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

	bool keyboardCallBackSettings(int key);
	void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*);

	enum
	{
		CV_WAITKEY_CURSORKEY_TOP = 2490368,
		CV_WAITKEY_CURSORKEY_BOTTOM = 2621440,
		CV_WAITKEY_CURSORKEY_RIGHT = 2555904,
		CV_WAITKEY_CURSORKEY_LEFT = 2424832,
	};

	//ƒNƒ‰ƒX“à•Ï”
	wchar_t directoryName[20];
	char nallowDirectoryName[20];
	std::string dataFileName;
	std::ofstream dataFile;
	inline std::string makeNameFolder(int hrgn);
	inline std::string makeNameFail(int hrgn, int num);
	inline void printText(int hrgn, int num);
	inline std::string getTime(void);

	const int numMax = 9; // •Û‘¶‚·‚éˆê•¶š‚Ì”

	const std::string dataFolderName = "Data";

	int num = 0; // ”Ô†Ši”[—p
	int hrgn = 0; // •¶šŠi”[—p

	std::string _time;
};

#endif // __MULTIREALSENSE__