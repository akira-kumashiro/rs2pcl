#include "multirealsense.h"

// Constructor
MultiRealSense::MultiRealSense() :
	viewer(new pcl::visualization::PCLVisualizer("3D Viewer"))//viewer�̃R���X�g���N�^
{
	// Initialize
	initialize();

	viewer->registerKeyboardCallback(&MultiRealSense::keyboardCallback, *this);
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
	if (realsenses.size() == 0)
	{
		throw std::runtime_error("�L����RealSense�J�������ڑ�����Ă��܂���I");
		return;
	}
	// Main Loop
	while (!viewer->wasStopped())//while(1)�Ƃ���������G���[���o��
	{
		for (auto& realsense : realsenses)
		{
			// Update Data
			realsense.second->update();

			// Draw Data
			realsense.second->draw();

			// Show Data
			//realsense->show();

			//viewer�ɐ�������pointcloud��n���čX�V
			updateViewerText();
			for (const auto& pair : realsense.second->clouds)
			{
				if (pair.first == "camera")
					continue;
				//viewer->updatePointCloud(pair.second.cloud, pair.second.name);
				viewer->updatePointCloud(regist_near.at(realsense.first).transformPointcloud(pair.second.cloud), pair.second.name);
			}
			viewer->spinOnce();

			//if (realsense->sleepTime >= 120)
			//	return;
		}

		// Key Check
		if (!keyboardCallBackSettings(cv::waitKey(10)))
			break;


	}
}

// Initialize
void MultiRealSense::initialize()
{
	cv::setUseOptimized(true);

	_time = getTime();

	// Retrive Connected Sensors List
	rs2::context context;
	const rs2::device_list device_list = context.query_devices();

	//int cameraNum = 0;
	// Initialize Connected Sensors
	for (const rs2::device& device : device_list)
	{
		// Check Device
		// "Platform Camera" is not RealSense Devices
		const std::string friendly_name = device.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME);
		if (friendly_name == platformCameraName)
			continue;

		//cameraNum++;

		// Initialize Sensor
		initializeSensor(device);
	}

	//PCL��viewer�̏�����
	initializeViewer();
}

// Initialize Sensor
inline void MultiRealSense::initializeSensor(const rs2::device& device)
{
	const std::string friendly_name = device.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME);
	const std::string serial_number = device.get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER);

	// Add Sensor to Container
	//realsenses.push_back(std::make_unique<RealSense>(enableChangeLaserPower,serial_number, friendly_name));
	if (friendly_name == "Intel RealSense SR300")
		realsenses.emplace(serial_number, std::make_unique<SR300>(device));
	else if (friendly_name == "Intel RealSense D415" || friendly_name == "Intel RealSense D435")
		realsenses.emplace(serial_number, std::make_unique<D400>(device));
	else
		realsenses.emplace(serial_number, std::make_unique<RealSense>(device));

	regist_near.emplace(serial_number, PCL_Regist(1e-5, 0.2, 1000, 5, 2.0e-3));
	regist_tip.emplace(serial_number, PCL_Regist(1e-5, 1.0, 1000, 20, 0.0));
	transformMat.emplace(serial_number, Eigen::Matrix4f::Identity());
}

inline void MultiRealSense::initializeViewer()
{
	for (const auto&realsense : realsenses)
	{
		for (const auto& pair : realsense.second->clouds)
		{
			viewer->addPointCloud(pair.second.cloud, pair.second.name);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pair.first == "tip" ? 20.0 : 1.0, pair.second.name);
		}
	}

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(0.01);
	viewer->initCameraParameters();
}

// Finalize
void MultiRealSense::finalize()
{
	// Close Windows
	cv::destroyAllWindows();
	viewer->close();
}

bool MultiRealSense::keyboardCallBackSettings(int key)
{
	cv::Mat tmp;
	const auto beginItr = realsenses.begin();
	if (key == 0)
		return true;

	switch (key)
	{
	case CV_WAITKEY_CURSORKEY_TOP:num++;	break; // ���� + 1
	case CV_WAITKEY_CURSORKEY_BOTTOM:num--;	break; // ���� - 1
	case CV_WAITKEY_CURSORKEY_RIGHT:hrgn++;	break; // ���� + 1
	case CV_WAITKEY_CURSORKEY_LEFT:hrgn--;	break; // ���� - 1
	case ' ': // �ۑ�
		CreateDirectory(dataFolderName.c_str(), NULL);
		CreateDirectory((dataFolderName + "\\" + _time).c_str(), NULL); // ��{�̃t�H���_�쐬�i���O�����ԁj
		for (const auto &realsense : realsenses)
		{
			realsense.second->saveData(dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn), "\\" + makeNameFail(hrgn, num));
		}
		num++;
		break;
	case 'q': // �I��
		viewer->close();
		return false;
	case 't':
		for (auto itr = std::next(beginItr, 1); itr != realsenses.end(); itr++)
		{
			//if (itr == realsenses.begin())
			//	continue;
			if (beginItr->second->clouds.at("tip").cloud->size() < 5)
			{
				std::cout << "The number of points #" + beginItr->first + " is too small!" << std::endl;
				return true;
			}
			if (itr->second->clouds.at("tip").cloud->size() < 5)
			{
				std::cout << "The number of points #" + itr->first + " is too small!" << std::endl;
				continue;
			}
			//if (realsenses.begin()->second->clouds.at("tip").cloud->size() >= 5 && itr->second->clouds.at("tip").cloud->size() >= 5)
			//{
			//transformMat.at(itr->first) = transformMat.at(itr->first) * regist_tip.at(itr->first).getTransformMatrix(beginItr->second->clouds.at("tip").cloud, itr->second->clouds.at("tip").cloud, Eigen::Matrix4f::Identity());//, transformMat[i]
			transformMat.at(itr->first) = transformMat.at(itr->first) * regist_near.at(itr->first).getTransformMatrix(beginItr->second->clouds.at("hand").cloud, itr->second->clouds.at("hand").cloud, transformMat.at(itr->first));
			//}
			//else
			//{
			//	if (realsenses.begin()->second->clouds.at("tip").cloud->size() < 5)
			//		std::cout << "The number of points #" + realsenses.begin()->first + " is too small!" << std::endl;
			//	if (itr->second->clouds.at("tip").cloud->size() < 5)
			//		std::cout << "The number of points #" + itr->first + " is too small!" << std::endl;
			//}
		}

		//for (int i = 1; i < realsenses.size(); i++)
		//{
		//	if (realsenses[0]->clouds.at("tip").cloud->size() >= 5 && realsenses[i]->clouds.at("tip").cloud->size() >= 5)
		//	{
		//		//transformMat[i] = transformMat[i] * regist_tip[i].getTransformMatrix(realsenses[0]->clouds.at("tip").cloud, realsenses[i]->clouds.at("tip").cloud, Eigen::Matrix4f::Identity());//, transformMat[i]
		//		transformMat[i] = transformMat[i] * regist_near[i].getTransformMatrix(realsenses[0]->clouds.at("hand").cloud, realsenses[i]->clouds.at("hand").cloud, transformMat[i]);
		//	}
		//	else
		//	{
		//		if (realsenses[0]->clouds.at("tip").cloud->size() < 5)
		//			std::cout << "The number of points #0 is too small!" << std::endl;
		//		if (realsenses[i]->clouds.at("tip").cloud->size() < 5)
		//			std::cout << "The number of points #" + std::to_string(i) + " is too small!" << std::endl;
		//	}
		//}
		break;
	default:
		return true;
	}

	if (numMax - 1 < num)
	{
		num = 0;
		hrgn++;
	}
	else if (num < 0)
	{
		num = numMax - 1;
		hrgn--;
	}
	if (hrgn < 0)
	{
		num = 0;
		hrgn = 0;
	}
	else if ('�' - '�' < hrgn)
	{
		num = numMax - 1;
		hrgn = '�' - '�';
	}
	printText(hrgn, num);
	// �{�^���������ꂽ�Ƃ��̂ݕ\��
	return true;
}

inline std::string MultiRealSense::makeNameFolder(int hrgn)
{
	std::string nameFolder = "�";
	nameFolder[0] = '�' + hrgn;
	return nameFolder;
}

inline std::string MultiRealSense::makeNameFail(int hrgn, int num)
{
	std::string nameFail = "�-00";
	nameFail[0] = '�' + hrgn;
	nameFail[2] = '0' + num / 10;
	nameFail[3] = '0' + num % 10;
	return nameFail;
}

inline void MultiRealSense::printText(int hrgn, int num)
{
	cout << "�t�@�C���ɂ���" << endl;
	cout << "�����F" + makeNameFolder(hrgn) << "(" << hrgn << ")" << "  ";
	cout << "�ԍ��F" << num << "  ";
	cout << "�f�B���N�g���F" << makeNameFolder(hrgn) + "\\" + makeNameFail(hrgn, num) << "  ";
	cout << endl;
	cout << "������@" << endl;
	cout << "�����F+1�c��  -1�c��" << endl;
	cout << "�����F+1�c��  -1�c��" << endl;
	cout << "�ۑ��F�X�y�[�X�L�[(�����Ŏ��̕����Ɉڍs)" << endl;
	cout << "���W�X�g���[�V�����Ft" << endl;
	cout << "�I���Fq" << endl;
	//cout << endl;
	//cout << "�E�s���N�Ɏ���d�˂ĎB��" << endl;
	//cout << "�E�Ȃ�ׂ��肪���F���͈͂ŎB��" << endl;
	//cout << "�E�����g���ŎB��" << endl;
	cout << endl;
}

inline std::string MultiRealSense::getTime(void)
{
	//���ݓ������擾����
	time_t t = time(nullptr);

	//�`����ϊ�����    
	const tm* lt = localtime(&t);

	//s�ɓƎ��t�H�[�}�b�g�ɂȂ�悤�ɘA�����Ă���
	std::stringstream s;
	s << "20";
	s << lt->tm_year - 100; // 100���������Ƃ�20xx��xx�̕����ɂȂ�
	s << "�N";
	s << lt->tm_mon + 1; // ����0����J�E���g���Ă��邽��
	s << "��";
	s << lt->tm_mday; // ���̂܂�
	s << "��";
	s << lt->tm_hour; // ����
	s << "��";
	s << lt->tm_min; // ��
	s << "��";

	return s.str();
}

void MultiRealSense::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*)
{
	if (event.keyDown())
	{
		int key = event.getKeyCode();
		if (key != 0)
			keyboardCallBackSettings(key);
		else
		{
			std::string str = event.getKeySym();
			int key;
			if (str == "Right")
				key = CV_WAITKEY_CURSORKEY_RIGHT;
			else if (str == "Left")
				key = CV_WAITKEY_CURSORKEY_LEFT;
			else if (str == "Up")
				key = CV_WAITKEY_CURSORKEY_TOP;
			else if (str == "Down")
				key = CV_WAITKEY_CURSORKEY_BOTTOM;
			keyboardCallBackSettings(key);
		}
	}
}

void MultiRealSense::updateViewerText(void)
{
	std::vector<boost::format> entries;
	for (const auto&realsense : realsenses)
	{
		entries.push_back(boost::format("Cam #%i FPS:%i Num of cloud:%i") % realsense.second->serial_number % realsense.second->fps % (int)(realsense.second->clouds.at("camera").cloud->size()));
	}

	const int dx = 5;
	const int dy = 14;
	const int fs = 10;
	boost::format name_fmt("text%i");

	for (size_t i = 0; i < entries.size(); ++i)
	{
		std::string name = boost::str(name_fmt % i);
		std::string entry = boost::str(entries[i]);
		if (!viewer->updateText(entry, dx, dy + i * (fs + 2), fs, 1.0, 1.0, 1.0, name))
			viewer->addText(entry, dx, dy + i * (fs + 2), fs, 1.0, 1.0, 1.0, name);
	}
}