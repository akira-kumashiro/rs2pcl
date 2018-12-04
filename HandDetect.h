#pragma once

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <iostream>

#ifdef _DEBUG
//Debugモードの場合
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_core2411d.lib")
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_imgproc2411d.lib")
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_highgui2411d.lib")
#else
//Releaseモードの場合
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_core2411.lib")
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_imgproc2411.lib")
#pragma comment(lib,"C:\\opencv\\opencv-2.4.11\\build\\x64\\vc12\\lib\\opencv_highgui2411.lib")
#endif

class HandDetect
{
public:
	HandDetect(double nearThreshold, double farThreshold);
	std::vector<cv::Point> getTipData(cv::Mat depth, cv::Mat color);
	cv::Mat colorMarked, contourMask;
	~HandDetect();
	class MatContainer
	{
	public:
		cv::Mat _mat;
		std::string name;
		MatContainer(std::string name, cv::Mat mat) :
			name(name),
			_mat(mat)
		{

		}

		MatContainer(std::string name, IplImage* mat) :
			name(name)
		{
			_mat = cv::Mat::zeros(cv::Size(mat->width, mat->height), CV_8UC3);
			for (int y = 0; y < mat->height; y++)
			{
				cv::Vec3b* _matPtr = _mat.ptr<cv::Vec3b>(y);
				for (int x = 0; x < mat->width; x++)
				{
					for (int i = 0; i < 3; i++)
					{
						_matPtr[x][i] = mat->imageData[mat->widthStep * y + x * 3 + i];
					}
				}
			}
		}

		//		MatContainer(std::string name, IplImage* mat, int cvType) :
		//			name(name)
		//		{
		//			int _cvType;
		//			switch (mat->depth)
		//			{
		//			case IPL_DEPTH_8U:
		//				switch (mat->nChannels)
		//				{
		//				case 1:_cvType = CV_8U; break;
		//				case 2:_cvType = CV_8UC2; break;
		//				case 3:_cvType = CV_8UC3; break;
		//				case 4:_cvType = CV_8UC4; break;
		//
		//				}
		//				break;
		//			case IPL_DEPTH_32F:
		//				switch (mat->nChannels)
		//				{
		//				case 1:_cvType = CV_32FC1; break;
		//				case 2:_cvType = CV_32FC2; break;
		//				case 3:_cvType = CV_32FC3; break;
		//				case 4:_cvType = CV_32FC4; break;
		//
		//				}
		//				break;
		//			case IPL_DEPTH_16S:
		//				switch (mat->nChannels)
		//				{
		//				case 1:_cvType = CV_16SC1; break;
		//				case 2:_cvType = CV_16SC2; break;
		//				case 3:_cvType = CV_16SC3; break;
		//				case 4:_cvType = CV_16SC4; break;
		//
		//				}
		//				break;
		//			default:
		//				return;
		//			}
		//			
		//			_mat = cv::Mat::zeros(cv::Size(mat->width, mat->height), _cvType);
		//			for (int y = 0; y < mat->height; y++)
		//			{
		//for (int x = 0; x < mat->width; x++)
		//{
		//	for (int c = 0; c < mat->nChannels; c++)
		//	{
		//		_mat.data[y*mat->width+x*mat->widthStep+c]
		//	}
		//
		//
		//				cv::Vec3b* colorMarkedPtr = colorMarked.ptr<cv::Vec3b>(y);
		//				uchar* contourMaskPtr = contourMask.ptr<uchar>(y);
		//				
		//				
		//					for (int i = 0; i < mat->depth; i++)
		//					{
		//						colorMarkedPtr[x][i] = mat->imageData[mat->widthStep * y + x * 3 + i];
		//					}
		//					contourMaskPtr[x] = mat->imageData[mat->widthStep*y + x * 3];
		//				}
		//			}
		//		}
	};

	//std::vector<MatContainer> imageList;
private:
	cv::Mat colorImage, depthImage, depthBinaryImage;
	double _nearThreshold, _farThreshold;
	cv::Mat getBinaryImage(void);
};

