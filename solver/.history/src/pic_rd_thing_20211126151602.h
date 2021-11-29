#ifndef PIC_THING
#define PIC_THING

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "get_rd.h"

using namespace std;
using namespace cv;


double height_of_reflector=0.65;

tuple<double, double, double> binarySearch(FileNode objects, int x, int low, int high);


void mouse_callback(int  event, int  x, int  y, int  flag, void *param);

Point pt(-1,-1);
bool newCoords = false;

class click_out_point{
    public:
    vector<Point2f> pts;
    void click(Mat frame){
        namedWindow("img", 1);
        // Set callback
        setMouseCallback("img", mouse_callback);

        int index=0;
        for (;;)
        {   
            // Show last point clicked, if valid
            if (pt.x != -1 && pt.y != -1)
            {
                // circle(frame, pt, 3, Scalar(0, 0, 255));
                if (newCoords)
                {   
                    drawMarker(frame, pt, Scalar(0,0,255), MarkerTypes::MARKER_CROSS, 20);
                    std::cout << "Clicked coordinates: " << pt << std::endl;
                    newCoords = false;
                    pts.push_back(Point2f(pt.x, pt.y));
                    index += 1;
                }
            }



            imshow("img", frame);

            // Exit if 'q' is pressed
            if ((waitKey(1) & 0xFF) == 'q'){
                pt.x = -1;
                pt.y = -1;
                break;
                }
        }
        // the camera will be deinitialized automatically in VideoCapture destructor
        };
    void print_point(){
        for (const auto &i : pts ){
            cout<<"x = "<< i.x <<", y = " << i.y << endl;
        }
        };

    void store_point(string path){
        ofstream store_point;
        store_point.open(path);
        for (const auto &i : pts ){
            store_point << i.x <<" " << i.y << "\n";
        }
        store_point.close();

    }
};

void mouse_callback(int  event, int  x, int  y, int  flag, void *param)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        // Store point coordinates
        pt.x = x;
        pt.y = y;
        newCoords = true;
    }
}



class verify
{
private:
    /* data */
    Mat rmat;
    Point3f _tvec;
public:
    verify(Mat1d rvec, Mat1d tvec, Mat camera_matrix, string path_img, string path_json);
    static double error_(  vector<Point2f> true_, vector<Point2f> out_ );
};

verify::verify(Mat1d rvec, Mat1d tvec, Mat camera_matrix, string path_img, string path_yaml)
{
    Rodrigues(rvec, rmat);

    _tvec.x = tvec(0,0);
    _tvec.y = tvec(1,0);
    _tvec.z = tvec(2,0);

    FileStorage f(path_yaml, FileStorage::READ);
    FileNode features = f["features"];

    Get_rd rd;
    rd.store_rd(features, "../radar_data/");
    vector<Point3f> rd_loc_t(rd.loc);
    vector<Point3f> rd_loc_t1(rd.loc);
    transform(rd.loc, rd_loc_t, rmat);

    for(auto &i : rd_loc_t) cout << i.x << " " << i.y << " " << i.z<<endl;

    for(auto &i : rd_loc_t) i = i+_tvec;

    transform(rd_loc_t, rd_loc_t1, camera_matrix);
    for(auto &i : rd_loc_t1) cout << i.x << " " << i.y << " " << i.z<<endl;

    for(int i=0; i<rd_loc_t1.size() ; i++){
        rd_loc_t1[i] = rd_loc_t1[i]*rd_loc_t[i].z;
    }

    vector<Point2i> results;
    for(int i=0; i<rd_loc_t1.size() ; i++){
        results.push_back(Point2i((int)rd_loc_t1[i].x, (int)rd_loc_t1[i].y));
    }

    for(int i=0; i<results.size() ; i++){
        cout << results[i].x <<" " << results[i].y << endl;
    }

    click_out_point ver_pts;
    Mat frame = imread( path_img, CV_LOAD_IMAGE_COLOR );

    imshow("img", frame);

    for(int i=0; i<results.size() ; i++){
        cout << results[i].x <<" " << results[i].y << endl;
        drawMarker(frame, pt, Scalar(0,0,255), MarkerTypes::MARKER_CROSS, 20);
    }
    waitKey();
}

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
    for (int i=0 ; i<objects.size(); i++)
        if( int(objects[i]["objId"]) == x) return make_tuple(floorf((float)objects[i]["objContour"]["x"])/100.0, floorf((float)objects[i]["objContour"]["y"])/100.0, height_of_reflector);
}

#endif