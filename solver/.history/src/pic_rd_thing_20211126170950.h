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
void mouse_callback(int  event, int  x, int  y, int  flag, void *param)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        // Store point coordinates
        pt.x = x;
        pt.y = y;
        newCoords = true;
    }
};



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
    void store_point(string path){
};





class verify
{
private:
    /* data */
    Mat rmat;
    Point3f _tvec;
public:
    static double error_(  vector<Point2f> true_, vector<Point2f> out_ );
};

double verify::error_( vector<Point2f> true_, vector<Point2f> out_){
    try
    {
        if(true_.size()==out_.size()){
            double e=0, lost=0;
            for (size_t i = 0; i < true_.size(); i++)
                {   
                    e += cv::norm(true_[i] - out_[i]);
                    lost+= cv::norm(true_[i] - out_[i])* cv::norm(true_[i] - out_[i]);
                }
            cout << " Lost: " << lost << endl;
            
            return (e/true_.size());
        }else{
            throw "Size for calibration error computation is not in accordance!";
        }

    }
    catch(const std::exception& e)
    {
        std::cerr << "Exception" << e.what() << '\n';
    }



}

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

void Get_rd::store_rd(FileNode f, string path){

    string path_json;
    for (FileNodeIterator it= f.begin(); it!= f.end();++it)
    {
        int filename = (int)(*it)["file_name"];
        path_json = path + to_string(filename) + ".json";
        obj_num.push_back((int)(*it)["id"].size());
        for (size_t i = 0; i < (int)(*it)["id"].size(); i++)
        {   
            id.push_back((int)(*it)["id"][i]);
        }

        FileStorage reflects(path_json, FileStorage::READ);
        FileNode objects = reflects["objects"];

        double x, y, z;

        for (int i=0 ; i < id.size() ; i++) {
            tie(x, y, z) = binarySearch(objects, id[i], 0, objects.size());
            loc.push_back(Point3f(x, y, z));
            
            // !!!!The order here need to in accordance with the clicked point in the picutre!!!

        }   
        id_all.insert(id_all.end(), id.begin(), id.end());
        id.clear();
    }
}

int Get_rd::print_rd(){
        
        cout <<"User input: " << endl;
        for (auto it=id_all.begin(); it!=id_all.end(); ++it) {     
            cout << "id " << *it <<endl;
        };
        cout <<"loc" <<endl;
        cout <<"json searched results: "<<endl;
        for (const auto &i : loc ){
            cout<<"x = "<<i.x <<", y = " << i.y << ", z = " << i.z <<endl;
        
    }
}

tuple<double, double, double> binarySearch(FileNode objects, int x, int low, int high){
    double height_of_reflector=0.65;
    for (int i=0 ; i<objects.size(); i++)
        if( int(objects[i]["objId"]) == x) return make_tuple(floorf((float)objects[i]["objContour"]["x"])/100.0, floorf((float)objects[i]["objContour"]["y"])/100.0, height_of_reflector);
}

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

int SolveBA(ba adjuster) {

    ceres::Problem problem;
    for (int i = 0; i < adjuster.TotalObserv; ++i) {

        ceres::CostFunction *cost_function =
                ReprojectAdjuster::Create(*(adjuster.get_PixelPoints(i)),
                                                 *(adjuster.get_PixelPoints(i)+1),
                                                 adjuster.get_ObjectPoints(i));

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