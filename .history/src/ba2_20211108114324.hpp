#ifndef BA
#define BA
#include <cmath>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace cv;

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

ba::ba(string path)
{
    FileStorage fs(path, FileStorage::READ);
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeff"] >> distCoeff;
    fs["rvec"] >> rvec;
    fs["tvec"] >> tvec;
    fs["ObjectPoints"] >> objectPoints;
    fs["PixelPoints"] >> pixelPoints;
    fs.release();

    Rvec = new double[3];
    Tvec = new double[3];
    CameraMatrix = new double[9];
    DistCoeff = new double[4];

    Rvec = (double*)rvec.data;
    Tvec = (double*)tvec.data;
    CameraMatrix = (double*)cameraMatrix.data;
    DistCoeff = (double*)distCoeff.data;

    ObjectPoints = new double[3*objectPoints.size()];
    PixelPoints = new double[2*pixelPoints.size()];

    for ( auto i=0 ; i < objectPoints.size() ; i++){
        ObjectPoints[3*i] = objectPoints[i].x;
        ObjectPoints[3*i+1] = objectPoints[i].y;
        ObjectPoints[3*i+2] = objectPoints[i].z;
    }

    for ( auto i=0 ; i < pixelPoints.size() ; i++){
        PixelPoints[2*i] = pixelPoints[i].x;
        PixelPoints[2*i+1] = pixelPoints[i].y;
    }

    TotalObserv = pixelPoints.size();
}

ba::~ba()
{
}

struct ReprojectAdjuster{
    ReprojectAdjuster(int pixel_u, int pixel_v, double* ObjectPoints, double* CameraMatrix, double* DistCoeff) : pixel_u_(pixel_u), pixel_v_(pixel_u), 
                                                                            ObjectPoints_(ObjectPoints), CameraMatrix_(CameraMatrix), DistCoeff_(DistCoeff){}

    template<typename T>
    bool operator()(const T* const rvec, const T* const tvec, T *residuals) const {
        T p[3];
        T p_[3];
        T p__[3];
        T u, v;
        T k1, k2, p1, p2, r2;
        T obj[3];

        obj[0] = T(ObjectPoints_[0]);
        obj[1] = T(ObjectPoints_[1]);
        obj[2] = T(ObjectPoints_[2]);

        ceres::AngleAxisRotatePoint(rvec, obj, p);
        p[0] += tvec[0];
        p[1] += tvec[1];
        p[2] += tvec[2];

        p_[0] = p[0]/p[2];
        p_[1] = p[1]/p[2];
        
        k1 = T(DistCoeff_[0]);
        k2 = T(DistCoeff_[1]);
        p1 = T(DistCoeff_[2]);
        p2 = T(DistCoeff_[3]);
        r2 = p_[0]*p_[0] + p_[1]*p_[1];

        p__[0] = p_[0]*(T(1)+k1*r2+k2*r2*r2) + T(2)*p1*p_[0]*p_[1] + p2*(r2+T(2)*p_[0]*p_[0]);
        p__[1] = p_[1]*(T(1)+k1*r2+k2*r2*r2) + p1*(r2 +  T(2) *p_[1]*p_[1]) + T(2)*p2*p_[0]*p_[1];

        u = T(CameraMatrix_[0]) * p__[0] + T(CameraMatrix_[2]);
        v = T(CameraMatrix_[4]) * p__[1] + T(CameraMatrix_[5]);
        
        residuals[0] = T(pixel_u_) - u;
        residuals[1] = T(pixel_v_) - v;
        
        return true;
    }

    static ceres::CostFunction *Create(int pixel_u, int pixel_v, double* ObjectPoints, double* CameraMatrix, double* DistCoeff) {
        return (new ceres::AutoDiffCostFunction<ReprojectAdjuster, 2, 3, 3>(
                new ReprojectAdjuster(pixel_u, pixel_v, ObjectPoints, CameraMatrix, DistCoeff)));
    }


    int pixel_u_;
    int pixel_v_;
    double* CameraMatrix_;
    double* DistCoeff_;
    double* ObjectPoints_;

};

int SolveBA(ba adjuster) {
    
    cout << "here1" << *(adjuster.DistCoefficient()) << "\n" ;
    cout << "here2" << *(adjuster.DistCoefficient()+1) << "\n" ;
    cout << "here3" << *(adjuster.DistCoefficient()+2) << "\n" ;
    cout << "here4" << *(adjuster.DistCoefficient()+3) << "\n" ;

    ceres::Problem problem;
    for (int i = 0; i < adjuster.TotalObserv; ++i) {

        ceres::CostFunction *cost_function =
                ReprojectAdjuster::Create(*(adjuster.get_PixelPoints(i)),
                                                 *(adjuster.get_PixelPoints(i)+1),
                                                 adjuster.get_ObjectPoints(i),
                                                 adjuster.CameraIntrin(),
                                                 adjuster.DistCoefficient());

        problem.AddResidualBlock(cost_function,
                                 NULL,
                                 adjuster.RvecInit(),
                                 adjuster.TvecInit());
            
                                 
    }
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.FullReport() << "\n";
    return 0;

}




#endif