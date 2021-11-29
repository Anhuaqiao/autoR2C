# include <iostream>
# include <vector>
# include <opencv2/opencv.hpp>
# include <opencv2/highgui/highgui.hpp>
# include "click_pic.h"
# include "get_rd.h"

using namespace std;
using namespace cv;

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
    rd.store_rd(features);
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

