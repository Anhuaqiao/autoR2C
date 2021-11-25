#ifndef ELiminate_distortion
#define ELiminate_distortion
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>


using namespace std;
using namespace cv;


class Eliminator{
    public:
    Eliminator(Mat img_1, Mat camera_matrix,  Mat camera_distortion, Size imageSize, int imageWidth, const int imageHeight);
};


Eliminator::Eliminator(Mat img_1,  Mat cameraMatrix,  Mat distCoeff, Size imageSize, int imageWidth=1920, int imageHeight = 1200){
    Mat mapx, mapy;
    Mat R = Mat::eye(3, 3, CV_32F);
    initUndistortRectifyMap(cameraMatrix, distCoeff, R, cameraMatrix, imageSize, CV_32FC1, mapx, mapy);
    remap(img_1,img_1,mapx,mapy, INTER_LINEAR);
}
#endif