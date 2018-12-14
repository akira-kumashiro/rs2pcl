#ifndef __REALSENSE__
#define __REALSENSE__

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <chrono>
#include <thread>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "HandDetect.h"

class RealSense
{
private:
	// RealSense
	rs2::pipeline pipeline;
	rs2::pipeline_profile pipeline_profile;
	rs2::frameset frameset;
	
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




	//pc��points��librealsense�Ǝ��K�i�̓_�Q�f�[�^�֘A
	//openGL�ł͎g���₷���炵��
	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;

	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	rs2_option optionType = static_cast<rs2_option>(13);

	cv::Mat vertices_mat, texture_mat, rawDepthMat;

	//std::vector<cv::Size> colorSizes = {
	//cv::Size(320,180),//0
	//cv::Size(320,240),//1
	//cv::Size(424,240),//2
	//cv::Size(640,360),//3
	//cv::Size(848,480),//4
	//cv::Size(960,540),//5
	//cv::Size(1280,720),//6
	//cv::Size(1920,1080)//7//���d������color�����Ƃ�Ȃ��Ȃ����肷��
	//};

	//std::vector<cv::Size> depthSizes = {
	//	cv::Size(640,240),//0
	//	cv::Size(640,480)//1
	//};



public:
	// Constructor
	//RealSense(bool enableChangeLaserPower, const std::string serial_number, const std::string friendly_name = "");
	RealSense(const rs2::device& device);

	// Destructor
	virtual ~RealSense();

	// Update Data
	virtual void update();

	// Draw Data
	void draw();

	// Show Data
	void show();

	rs2::device device;
	
	class PCL_Container
	{
	public:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		std::string name;
		PCL_Container(std::string name) :
			name(name),
			cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
		{

		}

	};
	
	std::string serial_number;
	std::string friendly_name;

	std::map<std::string, PCL_Container> clouds;
	std::vector<std::string> cloud_names = { "camera","tip","hand" };

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr calcPointCloud(const rs2::points& points);
	double fps = 0;

	bool saveData(std::string directory, std::string name);

	int sleepTime = 30;

	int frameNum = 0;

	void setLaserMax(void);
	void setLaserMin(void);

private:
	// Initialize
	void initialize();

	// Initialize Sensor
	inline void initializeSensor();

	// Finalize
	void finalize();

private:
	// Update Frame
	inline void updateFrame();

protected:
	//���z�֐��Ŏg�����
	inline void excuteUpdate();

	inline void setLaserPower(float num);

private:
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

private:
	void setTipCloud();

	cv::Mat readDepth(const std::string name);
	void writeDepth(const std::string name);


	inline void setPoint(pcl::PointXYZRGB &point, const cv::Vec3b color);
	inline void setPoint(pcl::PointXYZRGB &point, const cv::Vec3b color, const cv::Vec3f vertex);
	inline void setPoint(pcl::PointXYZRGB &point, const cv::Vec3b color, const rs2::vertex vertex);

	inline void saveFile(std::string directory, std::string name, cv::Mat data);
	inline void saveFile(std::string directory, std::string name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr data);

protected:
	rs2::option_range range;

	class Mat_Container
	{
	public:
		cv::Mat mat;
		std::string name;
		Mat_Container(std::string name, cv::Mat mat) :
			name(name),
			mat(mat)
		{

		}

		void saveImage(std::string directory)
		{
			cv::Mat temp;

			CreateDirectory((directory).c_str(), NULL); // �t�H���_�쐬

			flip(mat, temp, 1); // ���]
			cv::imwrite(directory +"\\"+ name + ".tif", temp); // �摜�ۑ�
		}

	};

	std::vector<Mat_Container> depth_mat_vector;

private:

	// init()�����s����O�ɐݒ肷��@���F1
	bool enableReadColor = false; // �J���[�摜�̎擾����
	bool enableReadDepth = false; // depth8i�摜�̎擾����
	bool enableHandTracking = false; // �n���h�g���b�L���O�̋���
	bool enableMirror = false; // �~���[�\���̋���

	bool isSaveMirror = false; // �~���[�ł̉摜�ۑ�

							   // ��̈ʒu�̓_�̃T�C�Y
	int handSize = true;

	const std::string extension = "dpt";

	enum
	{
		d1_mirror
	};

	// dpt�f�[�^�̃w�b�_�f�[�^
	typedef struct
	{
		unsigned char size = 0x10;
		unsigned char identifier = 0;
		unsigned short width = 0;
		unsigned short height = 0;
		unsigned short resoHori = 0;
		unsigned short resoVert = 0;
		unsigned char type = 0;
		// | �\�� | �\�� | �\�� | �\�� | �\�� | �\�� | �\�� | �~���[ |
		unsigned char data1 = 0x00;
	}dptHeader;

	const int numMax = 9; // �ۑ�����ꕶ���̐�
	const int distMin = 375; // ��O�̋���
	const int distMax = 425; // ���̋���
	const int sizeLine = 20; // ���̑���

	int num = 0; // �ԍ��i�[�p
	int hrgn = 0; // �����i�[�p

	//std::string _time;

	int morph_elem = 0;
	int morph_size = 1;
	int const max_elem = 2;

	std::chrono::system_clock::time_point nowTime, prevTime;
};

//SR300��D400�V���[�Y�ł͋������Ⴄ�̂ŉ��z�֐��Ŏ���
//update()��setLaserPower()�����s���邩�ǂ�����if����p�����ɔ���
class SR300 : public RealSense
{
public:
	SR300(const rs2::device& device);
	~SR300();

	//	std::vector<cv::Size> colorSizes = {
	//cv::Size(320,180),//0
	//cv::Size(320,240),//1
	//cv::Size(424,240),//2
	//cv::Size(640,360),//3
	//cv::Size(848,480),//4
	//cv::Size(960,540),//5
	//cv::Size(1280,720),//6
	//cv::Size(1920,1080)//7//���d������color�����Ƃ�Ȃ��Ȃ����肷��
	//	};
	//
	//	std::vector<cv::Size> depthSizes = {
	//		cv::Size(640,240),//0
	//		cv::Size(640,480)//1
	//	};

	void update() override;
};

class D400 :public RealSense
{
public:
	D400(const rs2::device& device);
	~D400();

	//		std::vector<cv::Size> colorSizes = {
	//cv::Size(320,180),//0
	//cv::Size(320,240),//1
	//cv::Size(424,240),//2
	//cv::Size(640,360),//3
	//cv::Size(848,480),//4
	//cv::Size(960,540),//5
	//cv::Size(1280,720),//6
	//cv::Size(1920,1080)//7//���d������color�����Ƃ�Ȃ��Ȃ����肷��
	//	};
	//
	//	std::vector<cv::Size> depthSizes = {
	//		//cv::Size(640,240),//0
	//		cv::Size(640,480),//1
	//		cv::Size(424,240),
	//		cv::Size(480,270),
	//		cv::Size(640,360),
	//		cv::Size(848,480),
	//		cv::Size(1280,720),
	//	};

	void update() override;
};

#endif // __REALSENSE__