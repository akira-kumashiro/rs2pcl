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
	// Main Loop
	while (true) {
		for (std::unique_ptr<RealSense>& realsense : realsenses) {
			// Update Data
			realsense->update();

			// Draw Data
			realsense->draw();

			// Show Data
			realsense->show();

			//viewer�ɐ�������pointcloud��n���čX�V
			viewer->updatePointCloud(realsense->cloud, realsense->cloudName);
			viewer->spinOnce();
		}

		// Key Check
		if (!keyboardCallBackSettings(cv::waitKey(10)))
			break;
		//const int32_t key = cv::waitKey(10);
		//if (key == 'q') {
		//	break;
		//}
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

	//PCL��viewer�̏�����
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

	switch (key)
	{
	case CV_WAITKEY_CURSORKEY_TOP:num++;	break; // ���� + 1
	case CV_WAITKEY_CURSORKEY_BOTTOM:num--;	break; // ���� - 1
	case CV_WAITKEY_CURSORKEY_RIGHT:hrgn++;	break; // ���� + 1
	case CV_WAITKEY_CURSORKEY_LEFT:hrgn--;	break; // ���� - 1
	case ' ': // �ۑ�
		CreateDirectory(dataFolderName.c_str(), NULL);
		CreateDirectory((dataFolderName + "\\" + _time).c_str(), NULL); // ��{�̃t�H���_�쐬�i���O�����ԁj
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-Color").c_str(), NULL); // color�摜�t�H���_�쐬
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-Depth").c_str(), NULL); // depth�摜�t�H���_�쐬
		//CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-HandImage").c_str(), NULL); // HandImage�摜�t�H���_�쐬
		//CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-HandPoint").c_str(), NULL); // HandPoint�摜�t�H���_�쐬
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLHand").c_str(), NULL); // PCLHand�摜�t�H���_�쐬
		//CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLJoint").c_str(), NULL); // PCLJoint�摜�t�H���_�쐬
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLCamera").c_str(), NULL); // PCLCamera�摜�t�H���_�쐬
		//CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLNear").c_str(), NULL); // PCLNear�摜�t�H���_�쐬
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-Depth����p").c_str(), NULL); // depth����p�摜�t�H���_�쐬
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLTip").c_str(), NULL); // depth����p�摜�t�H���_�쐬
		for (int i = 0; i < realsenses.size(); i++)
		{
			realsenses[i]->saveData(dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn), "\\" + makeNameFail(hrgn, num) + "(" + std::to_string(i) + ")");

		}
		num++;
		break;
	case 'q': // �I��
		return false;
	//case '+':
	//	for (int i = 0; i < NUM; i++)
	//	{
	//		rsu[i].changeThreshold(true);
	//	}
	//	break;
	//case '-':
	//	for (int i = 0; i < NUM; i++)
	//	{
	//		rsu[i].changeThreshold(false);
	//	}
	//	break;
	//case 't':
	//	for (int i = 1; i < realsenses.size(); i++)
	//	{
	//		if (realsenses[0].tip_point_cloud_ptr->size() >= 5 && realsenses[i].tip_point_cloud_ptr->size() >= 5)
	//		{
	//			transformMat[i] = transformMat[i] * regist_tip[i].getTransformMatrix(rsu[0].tip_point_cloud_ptr, rsu[i].tip_point_cloud_ptr, Eigen::Matrix4f::Identity());//, transformMat[i]
	//			transformMat[i] = transformMat[i] * regist_near[i].getTransformMatrix(rsu[0].hand_point_cloud_ptr, rsu[i].hand_point_cloud_ptr, transformMat[i]);
	//		}
	//		else
	//		{
	//			if (realsenses[0].tip_point_cloud_ptr->size() < 5)
	//				std::cout << "The number of points #0 is too small!" << std::endl;
	//			if (realsenses[i].tip_point_cloud_ptr->size() < 5)
	//				std::cout << "The number of points #" + std::to_string(i) + " is too small!" << std::endl;
	//		}
	//	}
	//	break;
	default:
		/*if (key != -1)
		wColorIO(wColorIO::PRINT_VALUE, L"%d\n", key);*/
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
	cout << "�����F+1�cw  -1�cs" << endl;
	cout << "�����F+1�cd  -1�ca" << endl;
	cout << "�ۑ��F�X�y�[�X�L�[(�����Ŏ��̕����Ɉڍs)" << endl;
	cout << "�I���Fq" << endl;
	cout << endl;
	cout << "�E�s���N�Ɏ���d�˂ĎB��" << endl;
	cout << "�E�Ȃ�ׂ��肪���F���͈͂ŎB��" << endl;
	cout << "�E�����g���ŎB��" << endl;
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