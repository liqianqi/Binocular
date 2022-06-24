#include<opencv2/opencv.hpp>
#include<iostream>
using namespace std;

#define BINO_CFG "../Binocular.yaml"

class Binocular
{
public:
    Binocular();                                                    // 构造函数

    ~Binocular(){}                                                  // 析构函数

    cv::Point3f uv2xyz(cv::Point2f uvLeft,cv::Point2f uvRight);     // 像素坐标转世界坐标

    void setParam();                                                // 输入参数

private:
    cv::Mat leftIntrinsic;                                          // 左相机内参矩阵
    cv::Mat leftDistortion;                                         // 左相机畸变参数
    cv::Mat leftRotation;                                           // 左相机旋转矩阵
    cv::Mat leftTranslation;                                        // 左相机平移向量

    cv::Mat rightIntrinsic;                                         // 右相机内参矩阵
    cv::Mat rightDistortion;                                        // 右相机畸变矩阵
    cv::Mat rightRotation;                                          // 右相机旋转矩阵
    cv::Mat rightTranslation;                                       // 右相机平移向量

};

