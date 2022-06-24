#include "Binocular.h"

Binocular::Binocular()
{
    setParam();
}

void Binocular::setParam()
{
    cv::FileStorage fs(BINO_CFG, cv::FileStorage::READ);
    fs["M1"] >> leftIntrinsic;
    fs["D1"] >> leftDistortion;
    fs["M2"] >> rightIntrinsic;
    fs["D2"] >> rightDistortion;
    fs["R1"] >> leftRotation;
    fs["R2"] >> rightRotation;
    fs["P1"] >> leftTranslation;
    fs["P2"] >> rightTranslation;
    fs.release();
}

cv::Point3f Binocular::uv2xyz(cv::Point2f uvLeft, cv::Point2f uvRight)
{
    cv::Mat leftRT;
    hconcat(leftRotation, leftTranslation, leftRT);
    cv::Mat LeftM = leftIntrinsic * leftRT;

    cv::Mat rightRT;
    hconcat(rightRotation, rightTranslation, rightRT);
    cv::Mat RightM = rightIntrinsic * rightRT;

    //最小二乘法A矩阵
	cv::Mat A = cv::Mat(4, 3, CV_32F);
	A.at<float>(0, 0) = uvLeft.x * LeftM.at<float>(2, 0) - LeftM.at<float>(0, 0);
	A.at<float>(0, 1) = uvLeft.x * LeftM.at<float>(2, 1) - LeftM.at<float>(0, 1);
	A.at<float>(0, 2) = uvLeft.x * LeftM.at<float>(2, 2) - LeftM.at<float>(0, 2);

	A.at<float>(1, 0) = uvLeft.y * LeftM.at<float>(2, 0) - LeftM.at<float>(1, 0);
	A.at<float>(1, 1) = uvLeft.y * LeftM.at<float>(2, 1) - LeftM.at<float>(1, 1);
	A.at<float>(1, 2) = uvLeft.y * LeftM.at<float>(2, 2) - LeftM.at<float>(1, 2);

	A.at<float>(2, 0) = uvRight.x * RightM.at<float>(2, 0) - RightM.at<float>(0, 0);
	A.at<float>(2, 1) = uvRight.x * RightM.at<float>(2, 1) - RightM.at<float>(0, 1);
	A.at<float>(2, 2) = uvRight.x * RightM.at<float>(2, 2) - RightM.at<float>(0, 2);

	A.at<float>(3, 0) = uvRight.y * RightM.at<float>(2, 0) - RightM.at<float>(1, 0);
	A.at<float>(3, 1) = uvRight.y * RightM.at<float>(2, 1) - RightM.at<float>(1, 1);
	A.at<float>(3, 2) = uvRight.y * RightM.at<float>(2, 2) - RightM.at<float>(1, 2);

    //最小二乘法B矩阵
	cv::Mat B = cv::Mat(4, 1, CV_32F);
	B.at<float>(0, 0) = LeftM.at<float>(0, 3) - uvLeft.x * LeftM.at<float>(2, 3);
	B.at<float>(1, 0) = LeftM.at<float>(1, 3) - uvLeft.y * LeftM.at<float>(2, 3);
	B.at<float>(2, 0) = RightM.at<float>(0, 3) - uvRight.x * RightM.at<float>(2, 3);
	B.at<float>(3, 0) = RightM.at<float>(1, 3) - uvRight.y * RightM.at<float>(2, 3);

    cv::Mat XYZ = cv::Mat(3, 1, CV_32F);
	//采用SVD最小二乘法求解XYZ
	solve(A, B, XYZ, cv::DECOMP_SVD);

    cv::Point3f world;
	world.x = XYZ.at<float>(0, 0);
	world.y = -XYZ.at<float>(1, 0);
	world.z = XYZ.at<float>(2, 0);

	return world;

}
