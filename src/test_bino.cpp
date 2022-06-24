#include <iostream>
#include </usr/local/zed/include/sl/Camera.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "Binocular.h"

#define TEST

using namespace std;

void onMouse(int event, int x, int y, int flags, void* param) 
{

    cv::Mat *im = reinterpret_cast<cv::Mat*>(param);

    switch (event) 	// dispatch the event
    {

    case cv::EVENT_LBUTTONDOWN: // 鼠标左键按下

        cout << "imagePoint2d   x: " << x << "   y: " << y << endl;

        break;
    }
}

int getOCVtype(sl::MAT_TYPE type) 
{
    int cv_type = -1;
    switch (type) 
    {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    return cv_type;
}

cv::Mat slMat2cvMat(sl::Mat& input) {
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}


int main() 
{

    int i = 0;

    sl::Camera zed; // Create a ZED camera object
    double fps;
    // Set configuration parameters
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::VGA;
    init_params.depth_mode = sl::DEPTH_MODE::ULTRA;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.camera_fps = 60; // Set fps at 60
    // Open the camera
    sl::ERROR_CODE err = zed.open(init_params);
    zed.setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, 2);
    if (err != sl::ERROR_CODE::SUCCESS) 
    {
        printf("%s\n", toString(err).c_str());
        zed.close();
        return ; // Quit if an error occurred
    }

    sl::RuntimeParameters runtime_param;
    runtime_param.sensing_mode = sl::SENSING_MODE::STANDARD; // Use STANDARD sensing mode
    
	cv::Mat image_ocv_left;
	sl::Mat image_zed_left;	

	cv::Mat image_ocv_right;
	sl::Mat image_zed_right;

	cv::Point2f image_left_2d;
	cv::Point2f image_right_2d;

	cv::Point3f world;

	Binocular binocular;

	while(true)
	{
		if (zed.grab(runtime_param) == sl::ERROR_CODE::SUCCESS) // A new image is available if grab() returns SUCCESS
		{

            zed.retrieveImage(image_zed_left, sl::VIEW::LEFT); 	// Get the left image
            image_ocv_left = slMat2cvMat(image_zed_left);
            
			zed.retrieveImage(image_zed_right,sl::VIEW::RIGHT);
			image_ocv_right = slMat2cvMat(image_zed_right);

			
			cv::namedWindow("Original Image left",0); 			 // 命名
			cv::imshow("Original Image left", image_ocv_left); 	 // 显示原始图像

			cv::namedWindow("Original Image right",0);
			cv::imshow("Original Image right", image_ocv_right); // 显示原始图像

			cv::setMouseCallback("Original Image left", onMouse, reinterpret_cast<void*>(&image_ocv_left));

			cv::setMouseCallback("Original Image right", onMouse, reinterpret_cast<void*>(&image_ocv_left));

#ifdef TEST
			world = binocular.uv2xyz(image_left_2d,image_right_2d);

			char test[100];
            sprintf(test, "tz:%0.4f",world.z);
            cv::putText(image_ocv_left, test, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

            sprintf(test, "tx:%0.4f",world.x);
            cv::putText(image_ocv_left, test, cv::Point(image_ocv_left.cols/3, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

            sprintf(test, "ty:%0.4f",world.y);
            cv::putText(image_ocv_left, test, cv::Point(2*image_ocv_left.cols/3, 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

#else
        if(key == ' ')
        {
            string str="/home/liqianqi/BinocularWorld/ImageBino/imageLeft/image"+to_string(i)+".jpg";
            imwrite(str, image_ocv_left);
            cv::namedWindow("image_save",0); 			 // 命名
            imshow("image_save", image_ocv_left);

            string str1="/home/liqianqi/BinocularWorld/ImageBino/imageRight/image"+to_string(i)+".jpg";
            imwrite(str1, image_ocv_right);
            cv::namedWindow("image_save1",0); 			 // 命名
            imshow("image_save1", image_ocv_right);

            i++;
        }

#endif
            cv::waitKey(1);
        }
	
	}

	return 0;
}