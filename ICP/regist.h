#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

//这个是将点云dstPoint利用RT配到srcPoint上的  srcPoint=dstPoint*R+T
void registrateNPoint(std::vector<cv::Point3d>& srcPoints, std::vector<cv::Point3d>& dstPoints, cv::Mat& R, cv::Mat& T) {
	if (srcPoints.size() != dstPoints.size() || srcPoints.size() < 3 || dstPoints.size() < 3)
	{
		std::cout << "srcPoints.size():\t" << srcPoints.size();
		std::cout << "dstPoints.size():\t" << dstPoints.size();
		std::cout << "registrateNPoint points size donot match!";

	}
	double srcSumX = 0.0f;
	double srcSumY = 0.0f;
	double srcSumZ = 0.0f;

	double dstSumX = 0.0f;
	double dstSumY = 0.0f;
	double dstSumZ = 0.0f;

	size_t pointsNum = srcPoints.size();
	for (size_t i = 0; i < pointsNum; i++) {
		srcSumX += srcPoints[i].x;
		srcSumY += srcPoints[i].y;
		srcSumZ += srcPoints[i].z;

		dstSumX += dstPoints[i].x;
		dstSumY += dstPoints[i].y;
		dstSumZ += dstPoints[i].z;
	}
	cv::Point3d srcCentricPt(srcSumX / pointsNum, srcSumY / pointsNum, srcSumZ / pointsNum);
	cv::Point3d dstCentricPt(dstSumX / pointsNum, dstSumY / pointsNum, dstSumZ / pointsNum);
	cv::Mat srcMat;
	srcMat = cv::Mat::zeros(3, pointsNum, CV_64F);
	cv::Mat dstMat;
	dstMat = cv::Mat::zeros(3, pointsNum, CV_64F);
	for (size_t i = 0; i < pointsNum; ++i)
	{

		srcMat.at<double>(0, i) = srcPoints[i].x - srcCentricPt.x;
		srcMat.at<double>(1, i) = srcPoints[i].y - srcCentricPt.y;
		srcMat.at<double>(2, i) = srcPoints[i].z - srcCentricPt.z;

		dstMat.at<double>(0, i) = dstPoints[i].x - dstCentricPt.x;
		dstMat.at<double>(1, i) = dstPoints[i].y - dstCentricPt.y;
		dstMat.at<double>(2, i) = dstPoints[i].z - dstCentricPt.z;
	}

	cv::Mat matS = srcMat * dstMat.t();

	cv::Mat matU, matW, matV;
	cv::SVDecomp(matS, matW, matU, matV);

	cv::Mat matTemp = matU * matV;
	double det = cv::determinant(matTemp);

	double datM[] = { 1, 0, 0, 0, 1, 0, 0, 0, det };
	cv::Mat matM(3, 3, CV_64FC1, datM);

	cv::Mat matR = matV.t() * matM * matU.t();
	double tx, ty, tz;
	tx = dstCentricPt.x - (srcCentricPt.x * matR.at<double>(0, 0) + srcCentricPt.y * matR.at<double>(0, 1) + srcCentricPt.z * matR.at<double>(0, 2));
	ty = dstCentricPt.y - (srcCentricPt.x * matR.at<double>(1, 0) + srcCentricPt.y * matR.at<double>(1, 1) + srcCentricPt.z * matR.at<double>(1, 2));
	tz = dstCentricPt.z - (srcCentricPt.x * matR.at<double>(2, 0) + srcCentricPt.y * matR.at<double>(2, 1) + srcCentricPt.z * matR.at<double>(2, 2));
	double datT[] = { tx,ty,tz };
	cv::Mat matT(3, 1, CV_64F, datT);
	matR.copyTo(R);
	matT.copyTo(T);
}
