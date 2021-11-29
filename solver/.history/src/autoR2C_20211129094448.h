#ifndef AUTOR2C
#define AUTOR2C
#include "pic_rd_thing.h"
#include <opencv2/core/eigen.hpp>

class autoR2C
{
private:
    int imageWidth;
    int imageHeight;
    Size imageSize = Size(imageWidth, imageHeight);
    Mat cameraMatrix, distCoeff;
    string path_config = "../config.yaml";
    int distortion_flag;
    string path_pic_points = "../train/pic_points.txt";
    vector<Point2f> pic_points;    
    vector<Point2f> pic_points_test;
    string path_rad = "../train/";
    Get_rd rd;
    Mat1d rVec = Mat::zeros(3, 1, CV_64FC1);//init rvec 
    Mat1d tVec = Mat::zeros(3, 1, CV_64FC1);//init tvec
    string path_pic_verify = "../test";
    string result_path = "../results/transformation.yaml";
    string output_path = "../results/";
    string output_path_ba = "../results_ba/";
    string result_path_ba = "../results_ba/transformation.yaml";

    void get_camera_intrinsic(string path, Size& imageSize, Mat& cameraMatrix, Mat& distCoeff, int& distortion_flag);
    void get_pic_points(string path, vector<Point2f>& pic_points, Size imageSize,  Mat cameraMatrix, Mat distCoeff, int distortion_flag);
    void get_rad_points(string path, Get_rd& rd);
    void verification(string path, string output_path, string path_rad, string result_path, Mat rVec, Mat tVec, Mat cameraMatrix, Mat distCoeff, vector<Point2f>& pic_points, Get_rd rd, Size imageSize);
public:
    void run_calibrate();
};

#endif