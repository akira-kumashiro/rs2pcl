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
	if (realsenses.size() == 0)
		return;
	// Main Loop
	while (!viewer->wasStopped())//while(1)とかだったらエラーが出る
	{
		for (std::unique_ptr<RealSense>& realsense : realsenses)
		{
			// Update Data
			realsense->update();

			// Draw Data
			realsense->draw();

			// Show Data
			//realsense->show();

			//viewerに生成したpointcloudを渡して更新
			updateViewerText();
			for (const auto& pair : realsense->clouds)
			{
				if (pair.first == "hand")
					continue;
				viewer->updatePointCloud(pair.second.cloud, pair.second.name);
			}
			//viewer->updatePointCloud(realsense->camera_cloud_ptr, realsense->camera_cloud_name);
			//viewer->updatePointCloud(realsense->tip_cloud_ptr, realsense->tip_cloud_name);
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

	//context.set_devices_changed_callback([&](rs2::event_information& info)
	//{
	//	std::cout << "remove" << std::endl;
	//	this->removeDevice(info);
	//	for (auto&&dev : info.get_new_devices())
	//	{
	//		if (dev.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME) == platformCameraName)
	//			continue;
	//		this->initializeSensor(dev);
	//	}
	//});

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

	//if (cameraNum == 0)
	//	return;

	//PCLのviewerの初期化
	initializeViewer();
}

// Initialize Sensor
inline void MultiRealSense::initializeSensor(const rs2::device& device)
{
	// Retrive Serial Number (and Friendly Name)
	//const std::string serial_number = device.get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER);
	const std::string friendly_name = device.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME);
	//std::unique_ptr<RealSense> ptr(new SR300());

	//bool enableChangeLaserPower = false;

	//if (device.supports(RS2_CAMERA_INFO_NAME))
	//{
	//	if (device.get_info(RS2_CAMERA_INFO_NAME) == "Intel RealSense SR300")
	//	{
	//		enableChangeLaserPower = true;
	//	}
	//}

	//auto sensor = device.query_sensors()[0];

	// Add Sensor to Container
	//realsenses.push_back(std::make_unique<RealSense>(enableChangeLaserPower,serial_number, friendly_name));
	if (friendly_name == "Intel RealSense SR300")
		realsenses.push_back(std::make_unique<SR300>(device));
	else if (friendly_name == "Intel RealSense D415" || friendly_name == "Intel RealSense D435")
		realsenses.push_back(std::make_unique<D400>(device));
	else
		realsenses.push_back(std::make_unique<RealSense>(device));

	regist_near.push_back(PCL_Regist(1e-2, 0.01, 1000, 5, 1.0e-3));
	regist_tip.push_back(PCL_Regist(1e-2, 0.2, 1000, 100, 0.0));
	transformMat.push_back(Eigen::Matrix4f::Identity());
	//RealSense *realsense = new D400(device);
}

inline void MultiRealSense::initializeViewer()
{
	//for (auto i = 0; i < realsenses.size(); i++)
	//{
	//	realsenses[i]->camera_cloud_name = "camera_cloud" + std::to_string(i);
	//	viewer->addPointCloud(realsenses[i]->camera_cloud_ptr, realsenses[i]->camera_cloud_name);
	//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, realsenses[i]->camera_cloud_name);
	//}

	//for (auto i = 0; i < realsenses.size(); i++)
	//{
	//	realsenses[i]->tip_cloud_name = "tip_cloud" + std::to_string(i);
	//	viewer->addPointCloud(realsenses[i]->tip_cloud_ptr, realsenses[i]->tip_cloud_name);
	//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20.0, realsenses[i]->tip_cloud_name);
	//}

	for (const auto&realsense : realsenses)
	{
		for (const auto& pair : realsense->clouds)
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

//void MultiRealSense::removeDevice(const rs2::event_information & info)
//{
//	//std::lock_guard<std::mutex> lock(_mutex);
//	// Go over the list of devices and check if it was disconnected
//	auto itr = realsenses.begin();
//	while (itr != realsenses.end())
//	{
//		if (info.was_removed((*itr)->device))
//		{
//			itr = realsenses.erase(itr);
//		}
//		else
//		{
//			++itr;
//		}
//	}
//}

bool MultiRealSense::keyboardCallBackSettings(int key)
{
	cv::Mat tmp;

	if (key == 0)
		return true;

	switch (key)
	{
	case CV_WAITKEY_CURSORKEY_TOP:num++;	break; // 数字 + 1
	case CV_WAITKEY_CURSORKEY_BOTTOM:num--;	break; // 数字 - 1
	case CV_WAITKEY_CURSORKEY_RIGHT:hrgn++;	break; // 文字 + 1
	case CV_WAITKEY_CURSORKEY_LEFT:hrgn--;	break; // 文字 - 1
	case ' ': // 保存
		CreateDirectory(dataFolderName.c_str(), NULL);
		CreateDirectory((dataFolderName + "\\" + _time).c_str(), NULL); // 大本のフォルダ作成（名前が時間）
		//CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-Color").c_str(), NULL); // color画像フォルダ作成
		//CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-Depth").c_str(), NULL); // depth画像フォルダ作成
		////CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-HandImage").c_str(), NULL); // HandImage画像フォルダ作成
		////CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-HandPoint").c_str(), NULL); // HandPoint画像フォルダ作成
		//CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLHand").c_str(), NULL); // PCLHand画像フォルダ作成
		////CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLJoint").c_str(), NULL); // PCLJoint画像フォルダ作成
		//CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLCamera").c_str(), NULL); // PCLCamera画像フォルダ作成
		////CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLNear").c_str(), NULL); // PCLNear画像フォルダ作成
		//CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-Depth見る用").c_str(), NULL); // depth見る用画像フォルダ作成
		//CreateDirectory((dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn) + "-PCLTip").c_str(), NULL); // depth見る用画像フォルダ作成
		//for (int i = 0; i < realsenses.size(); i++)
		//{
		//	realsenses[i]->saveData(dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn), "\\" + makeNameFail(hrgn, num));
		//}
		for (const auto &realsense : realsenses)
		{
			realsense->saveData(dataFolderName + "\\" + _time + "\\" + makeNameFolder(hrgn), "\\" + makeNameFail(hrgn, num));
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
	case 't':
		for (int i = 1; i < realsenses.size(); i++)
		{
			if (realsenses[0]->clouds.at("tip").cloud->size() >= 5 && realsenses[i]->clouds.at("tip").cloud->size() >= 5)
			{
				transformMat[i] = transformMat[i] * regist_tip[i].getTransformMatrix(realsenses[0]->clouds.at("tip").cloud, realsenses[i]->clouds.at("tip").cloud, Eigen::Matrix4f::Identity());//, transformMat[i]
				transformMat[i] = transformMat[i] * regist_near[i].getTransformMatrix(realsenses[0]->clouds.at("hand").cloud, realsenses[i]->clouds.at("hand").cloud, transformMat[i]);
			}
			else
			{
				if (realsenses[0]->clouds.at("tip").cloud->size() < 5)
					std::cout << "The number of points #0 is too small!" << std::endl;
				if (realsenses[i]->clouds.at("tip").cloud->size() < 5)
					std::cout << "The number of points #" + std::to_string(i) + " is too small!" << std::endl;
			}
		}
		break;
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

void MultiRealSense::updateViewerText(void)
{
	std::vector<boost::format> entries;
	for (const auto&realsense : realsenses)
	{
		entries.push_back(boost::format("Cam #%i FPS:%i Num of cloud:%i") % realsense->serial_number % realsense->fps % (int)(realsense->clouds.at("camera").cloud->size()));
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