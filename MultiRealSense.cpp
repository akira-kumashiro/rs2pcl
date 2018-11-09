#include "multirealsense.h"

// Constructor
MultiRealSense::MultiRealSense() :
	viewer(new pcl::visualization::PCLVisualizer("3D Viewer"))//viewerのコンストラクタ
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

			//viewerに生成したpointcloudを渡して更新
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
	case CV_WAITKEY_CURSORKEY_TOP:num++;	break; // 数字 + 1
	case CV_WAITKEY_CURSORKEY_BOTTOM:num--;	break; // 数字 - 1
	case CV_WAITKEY_CURSORKEY_RIGHT:hrgn++;	break; // 文字 + 1
	case CV_WAITKEY_CURSORKEY_LEFT:hrgn--;	break; // 文字 - 1
	case ' ': // 保存
		CreateDirectory(dataFolderName.c_str(), NULL);
		CreateDirectory((dataFolderName + "\\" + _time).c_str(), NULL); // 大本のフォルダ作成（名前が時間）
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-Color").c_str(), NULL); // color画像フォルダ作成
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-Depth").c_str(), NULL); // depth画像フォルダ作成
		//CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-HandImage").c_str(), NULL); // HandImage画像フォルダ作成
		//CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-HandPoint").c_str(), NULL); // HandPoint画像フォルダ作成
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLHand").c_str(), NULL); // PCLHand画像フォルダ作成
		//CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLJoint").c_str(), NULL); // PCLJoint画像フォルダ作成
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLCamera").c_str(), NULL); // PCLCamera画像フォルダ作成
		//CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLNear").c_str(), NULL); // PCLNear画像フォルダ作成
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-Depth見る用").c_str(), NULL); // depth見る用画像フォルダ作成
		CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLTip").c_str(), NULL); // depth見る用画像フォルダ作成
		for (int i = 0; i < realsenses.size(); i++)
		{
			realsenses[i]->saveData(dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn), "\\" + makeNameFail(hrgn, num) + "(" + std::to_string(i) + ")");

		}
		num++;
		break;
	case 'q': // 終了
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
	else if ('ﾝ' - 'ｱ' < hrgn)
	{
		num = numMax - 1;
		hrgn = 'ﾝ' - 'ｱ';
	}
	printText(hrgn, num);
	// ボタンが押されたときのみ表示
	return true;
}

inline std::string MultiRealSense::makeNameFolder(int hrgn)
{
	std::string nameFolder = "ｱ";
	nameFolder[0] = 'ｱ' + hrgn;
	return nameFolder;
}

inline std::string MultiRealSense::makeNameFail(int hrgn, int num)
{
	std::string nameFail = "ｱ-00";
	nameFail[0] = 'ｱ' + hrgn;
	nameFail[2] = '0' + num / 10;
	nameFail[3] = '0' + num % 10;
	return nameFail;
}

inline void MultiRealSense::printText(int hrgn, int num)
{
	cout << "ファイルについて" << endl;
	cout << "文字：" + makeNameFolder(hrgn) << "(" << hrgn << ")" << "  ";
	cout << "番号：" << num << "  ";
	cout << "ディレクトリ：" << makeNameFolder(hrgn) + "\\" + makeNameFail(hrgn, num) << "  ";
	cout << endl;
	cout << "操作方法" << endl;
	cout << "文字：+1…w  -1…s" << endl;
	cout << "数字：+1…d  -1…a" << endl;
	cout << "保存：スペースキー(自動で次の文字に移行)" << endl;
	cout << "終了：q" << endl;
	cout << endl;
	cout << "・ピンクに手を重ねて撮る" << endl;
	cout << "・なるべく手が黄色い範囲で撮る" << endl;
	cout << "・白い枠内で撮る" << endl;
	cout << endl;
}

inline std::string MultiRealSense::getTime(void)
{
	//現在日時を取得する
	time_t t = time(nullptr);

	//形式を変換する    
	const tm* lt = localtime(&t);

	//sに独自フォーマットになるように連結していく
	std::stringstream s;
	s << "20";
	s << lt->tm_year - 100; // 100を引くことで20xxのxxの部分になる
	s << "年";
	s << lt->tm_mon + 1; // 月を0からカウントしているため
	s << "月";
	s << lt->tm_mday; // そのまま
	s << "日";
	s << lt->tm_hour; // 時間
	s << "時";
	s << lt->tm_min; // 分
	s << "分";

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