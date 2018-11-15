#ifndef __MULTIREALSENSE__
#define __MULTIREALSENSE__

#include "realsense.h"

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

#include <vector>
#include <memory>

#include <windows.h>

class MultiRealSense
{
private:
	// RealSense
	std::vector<std::unique_ptr<RealSense>> realsenses;

	//pcl��viewer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

public:
	// Constructor
	MultiRealSense();

	// Destructor
	~MultiRealSense();

	// Processing
	void run();

	//�f�o�C�X���O���ꂽ���̏���
	//void removeDevice(const rs2::event_information& info);
	
private:
	// Initialize Sensor �Z���T�[�̓o�^
	inline void initializeSensor(const rs2::device& device);

	// Initialize
	void initialize();

	//pcl��viewer�̏����ݒ�
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

	//�N���X���ϐ�
	wchar_t directoryName[20];
	char nallowDirectoryName[20];
	std::string dataFileName;
	std::ofstream dataFile;
	inline std::string makeNameFolder(int hrgn);
	inline std::string makeNameFail(int hrgn, int num);
	inline void printText(int hrgn, int num);
	inline std::string getTime(void);
	void updateViewerText(void);

	const int numMax = 9; // �ۑ�����ꕶ���̐�

	const std::string dataFolderName = "Data";

	int num = 0; // �ԍ��i�[�p
	int hrgn = 0; // �����i�[�p

	std::string _time;
	
	//�ǂ��ł�����Web�J�����Ƃ��͑S�����̖��O�ɂȂ�
	const std::string platformCameraName = "Platform Camera";
};

#endif // __MULTIREALSENSE__