#ifndef PIC_THING
#define PIC_THING

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace cv;

extern Point pt;
extern bool newCoords;

tuple<double, double, double> binarySearch(FileNode objects, int x, int low, int high);

class Get_rd{
    private:
        int FLAG;
        int n;    
        vector<int> id_all; 
        vector<int> id;
    public:
        vector<Point3f> loc;
        vector<int> obj_num;
        void store_rd(FileNode f, string path);
        int print_rd();
};

class click_out_point{
    public:
    vector<Point2f> pts;
    void click(Mat frame);
    void print_point();
    void store_point(string path);
};


class verify
{
public:
    static double error_(  vector<Point2f> true_, vector<Point2f> out_ );
};

class Eliminator{
    public:
    Eliminator(Mat img_1, Mat camera_matrix,  Mat camera_distortion, Size imageSize, int imageWidth, const int imageHeight);
};

class ba
{
private:
    Mat cameraMatrix, distCoeff, rvec, tvec;
    vector<Point3f> objectPoints;
    vector<Point2f> pixelPoints;
    double* ObjectPoints;
    double* PixelPoints;
    double* Rvec;
    double* Tvec;
    double* CameraMatrix;
    double* DistCoeff;

public:
    int TotalObserv;
    ba(string path);
    ~ba();
    int OberserveTotal(){ return TotalObserv; }
    double* CameraIntrin(){ return CameraMatrix; }
    double* DistCoefficient(){ return DistCoeff; }
    double* RvecInit(){ return Rvec; }
    double* TvecInit(){ return Tvec; }
    double* get_ObjectPoints(int i){ return ObjectPoints + i*3; }
    double* get_PixelPoints(int i){ return PixelPoints + i*2; }
};

struct ReprojectAdjuster{
    ReprojectAdjuster(double pixel_u, double pixel_v, double* ObjectPoints) : pixel_u_(pixel_u), pixel_v_(pixel_v), ObjectPoints_(ObjectPoints){}

    template<typename T>
    bool operator()(const T* const rvec, const T* const tvec, T *residuals) const {
        T p[3];
        T p_[3];
        T p__[3];
        T k1, k2, p1, p2, r2;
        T u, v;
        T obj[3];

        obj[0] = T(ObjectPoints_[0]);
        obj[1] = T(ObjectPoints_[1]);
        obj[2] = T(ObjectPoints_[2]);

        ceres::AngleAxisRotatePoint(rvec, obj, p);
        p[0] = p[0] + tvec[0];
        p[1] = p[1] + tvec[1];
        p[2] = p[2] + tvec[2];

        p_[0] = p[0]/p[2];
        p_[1] = p[1]/p[2];

        k1 = T(-0.031066151822322162);
        k2 = T(0.034324349626048335);
        p1 = T(-0.062279666374504286);
        p2 = T(0.034648884538340426);
        
        r2 = T(p_[0]*p_[0] + p_[1]*p_[1]);

        p__[0] = p_[0]*(T(1)+k1*r2+k2*r2*r2) + T(2)*p1*p_[0]*p_[1] + p2*(r2+T(2)*p_[0]*p_[0]);
        p__[1] = p_[1]*(T(1)+k1*r2+k2*r2*r2) + p1*(r2 + T(2)*p_[1]*p_[1]) + T(2)*p2*p_[0]*p_[1];

        u = T(1014.887185159176) * p__[0] + T(930.3792328037258);
        v = T(1019.6356956271103) * p__[1] + T(487.9419547383717);
        
        residuals[0] = pixel_u_ - u;
        residuals[1] = pixel_v_ - v;
        
        return true;
    }

    static ceres::CostFunction *Create(double pixel_u, double pixel_v,  double* ObjectPoints) {
        return (new ceres::AutoDiffCostFunction<ReprojectAdjuster, 2, 3, 3>(
                new ReprojectAdjuster(pixel_u, pixel_v, ObjectPoints)));
    }


    double pixel_u_;
    double pixel_v_;
    double* ObjectPoints_;

};

int SolveBA(ba adjuster);

#endif