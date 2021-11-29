# include "calib_pic.h"
# include "click_pic.h"
# include "get_rd.h"
# include "calib_verification.h"
# include "ba.h"
# include <eigen3/Eigen/Core>
# include <eigen3/Eigen/Geometry>
# include <opencv2/core/eigen.hpp>

#define DegreeToRadians 0.017453293

using namespace std;
using namespace cv;

void get_camera_intrinsic(string path, Size& imageSize, Mat& cameraMatrix, Mat& distCoeff, int& distortion_flag){

    FileStorage fs(path, FileStorage::READ);
    fs["imageSize"] >> imageSize;
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeff"] >> distCoeff;
    fs["distortionFlag"] >> distortion_flag;
    fs.release();

    cout << "cameraMatrix" << cameraMatrix<< endl;
    cout << "distCoeff" << distCoeff<< endl;
}

void get_pic_points(string path, vector<Point2f>& pic_points, Size imageSize,  Mat cameraMatrix, Mat distCoeff, int distortion_flag){
    ifstream ifile;
    ifile.open(path);
        if (ifile) {
        string line;
        while ( getline(ifile, line) ){
            istringstream iss(line);
            int a, b;
            if(!( iss >> a >> b )) { break; }
            pic_points.push_back(Point2f(a, b));
        }
        
    }else{
        // read all the images inside one folder, and click points of interests
        String folderpath = "../img_data/*.jpg";
        vector<String> filenames;
        cv::glob(folderpath, filenames);
        click_out_point click_it;
        for (size_t i=0; i<filenames.size(); i++)
        {
            Mat img_1 = imread( filenames[i], CV_LOAD_IMAGE_COLOR );
            if (distortion_flag) Eliminator(img_1, cameraMatrix, distCoeff, imageSize);// 消除畸变
            // 点击角反射器中心 图片坐标系坐标
            click_it.click(img_1);
        
    }
    destroyAllWindows;
    // click_it.print_point();
    click_it.store_point(path);

    pic_points = click_it.pts;
    }
    for( auto &i : pic_points){
        cout << i.x << " " << i.y << endl;
    }
}
void get_rad_points(string path, Get_rd& rd){
    // read config yaml file
    string path_config = path +"config.yaml";
    FileStorage f(path_config, FileStorage::READ);
    FileNode features = f["features"];
    // search and store the json data
    rd.store_rd(features, path);
    rd.print_rd();
}

void verification(string path, string output_path, string path_rad, string result_path, Mat rVec, Mat tVec, Mat cameraMatrix, Mat distCoeff, vector<Point2f>& pic_points, Get_rd rd, Size imageSize){
    string path_config = path + "/config.yaml";
    cout << path_config << endl;
    FileStorage f_(path_config, FileStorage::READ);
    FileNode features_ = f_["features"];
    Get_rd rd_ver;
    rd_ver.store_rd(features_, path_rad);
    // project on to the pixel coordinate
    vector<Point2f>check_front_image_pts;
    projectPoints( rd_ver.loc, rVec, tVec, cameraMatrix, distCoeff, check_front_image_pts );

    // draw it on images
    String folderpath = path + "/*.jpg";
    vector<String> filenames;
    cv::glob(folderpath, filenames);

    cout << " size: " << check_front_image_pts.size() << endl;

    int start_num=0;
    for (int j=0; j<filenames.size(); j++)
    {   
        Mat img_ver = imread( filenames[j], CV_LOAD_IMAGE_COLOR );
        for(int i=start_num; i < start_num + rd_ver.obj_num[j] ; i++){
            cout << check_front_image_pts[i].x << " " << check_front_image_pts[i].y << endl;
            drawMarker(img_ver, check_front_image_pts[i], Scalar(0,0,255), MarkerTypes::MARKER_CROSS, 20);         
    }
    start_num += rd_ver.obj_num[j];
    std::ostringstream name;
    name << output_path << j << ".jpg";
    imwrite(name.str(), img_ver);
    }

    Mat Rotat(3, 3, CV_32F);
    Rodrigues(rVec, Rotat);
    Eigen::Matrix3d Rotation;
    cv::cv2eigen(Rotat, Rotation);
    cout << "\n" << " Rotation matrix: " << "\n";
    cout << Rotation << endl;
    Eigen::Vector3d euler_ = Rotation.eulerAngles(2,1,0); // euler angles here is right multiplication convention mat == AngleAxisf(ea[0], Vector3f::UnitZ())
                                                                                                                        // * AngleAxisf(ea[1], Vector3f::UnitX())
    Mat1d euler_mat;
    cv::eigen2cv(euler_, euler_mat);
    cout << "Euler angles: " << "\n";                                                                                                                    // * AngleAxisf(ea[2], Vector3f::UnitZ()); 
    cout << euler_(2)/DegreeToRadians <<" "<<euler_(1)/DegreeToRadians <<" "<< euler_(0)/DegreeToRadians << endl;
    cout << "Translation: " << "\n";
    cout << tVec << endl;

    double e = verify::error_(pic_points, check_front_image_pts);
    cout << " Calibration error: " << "\n";
    cout << e << endl;


   FileStorage fs(result_path, FileStorage::WRITE);

    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeff" << distCoeff;
    fs << "rvec" << rVec;
    fs << "tvec" << tVec;
    fs << "rotationMatrix" << Rotat;
    fs << "MSE" << e;
    fs << "ObjectPoints" << rd.loc;
    fs << "PixelPoints" << pic_points;
    fs.release();

    Mat_<double> trans_matrix(4,4);
    trans_matrix << Rotat.at<double>(0,0), Rotat.at<double>(0,1), Rotat.at<double>(0,2), tVec.at<double>(0,0), 
                    Rotat.at<double>(1,0), Rotat.at<double>(1,1), Rotat.at<double>(1,2), tVec.at<double>(1,0),
                    Rotat.at<double>(2,0), Rotat.at<double>(2,1), Rotat.at<double>(2,2), tVec.at<double>(2,0),
                    0, 0, 0, 1;

    string result_path_param = "../results/transformation_param.yaml";
    FileStorage fs_(result_path_param, FileStorage::WRITE);
    fs_ << "imageSize" << imageSize;
    fs_ << "cameraMatrix" << cameraMatrix;
    fs_ << "distCoeff" << distCoeff;
    fs_ << "rvec" << rVec;
    fs_ << "tvec" << tVec;
    fs_ << "transformationMatrix" << trans_matrix;
    fs_ << "eulerAngle" << euler_mat;
    fs_ << "MSE" << e;
    fs_.release();
}

 


int main(int argc, char** argv){
    int imageWidth;
    int imageHeight;
    Size imageSize = Size(imageWidth, imageHeight);
    Mat cameraMatrix, distCoeff;
    string path_config = "../config.yaml";
    int distortion_flag;
    string path_pic_points = "../img_data/pic_points.txt";
    vector<Point2f> pic_points;    
    string path_rad = "../radar_data/";
    Get_rd rd;
    Mat1d rVec = Mat::zeros(3, 1, CV_64FC1);//init rvec 
    Mat1d tVec = Mat::zeros(3, 1, CV_64FC1);//init tvec
    string path_pic_verify = "../img_data_verification";
    string result_path = "../results/transformation.yaml";
    string output_path = "../test/";
    ba adjuster(result_path);
    
    get_camera_intrinsic(path_config, imageSize, cameraMatrix, distCoeff, distortion_flag);
    get_pic_points(path_pic_points, pic_points, imageSize, cameraMatrix, distCoeff, distortion_flag);
    get_rad_points(path_rad, rd);
    solvePnP(rd.loc, pic_points, cameraMatrix, distCoeff, rVec, tVec, false, CV_ITERATIVE);
    cout << rVec << endl;
    cout << tVec << endl;
    verification(path_pic_verify, output_path, path_rad, result_path, rVec, tVec, cameraMatrix, distCoeff, pic_points, rd, imageSize);


    SolveBA(adjuster);

    cv::Mat rvec_ba(3, 1, CV_64F, adjuster.RvecInit());
    cv::Mat tvec_ba(3, 1, CV_64F, adjuster.TvecInit());
    vector<Point2f> check_front_image_pts;
    projectPoints( rd.loc, rvec_ba, tvec_ba, cameraMatrix, distCoeff, check_front_image_pts);

    // draw it on images
    String folderpath_ba = "../img_data_verification/*.jpg";
    vector<String> filenames_ba;
    cv::glob(folderpath_ba, filenames_ba);

    cout << " size: " << check_front_image_pts.size() << endl;

    for (int j=0; j<filenames_ba.size(); j++)
    {   
        Mat img_ver = imread( filenames_ba[j], CV_LOAD_IMAGE_COLOR );
        
        for(int i=j*4; i<j*4+4 ; i++){
            cout << check_front_image_pts[i].x << " " << check_front_image_pts[i].y << endl;
            drawMarker(img_ver, check_front_image_pts[i], Scalar(0,0,255), MarkerTypes::MARKER_CROSS, 20);
    }
        std::ostringstream name;
        name << "../test_ba/" << j << ".jpg";
        imwrite(name.str(), img_ver);
    }

    Mat Rotat_ba(3, 3, CV_64F);
    Rodrigues(rvec_ba, Rotat_ba);
    Eigen::Matrix3d Rotation_ba;
    cv::cv2eigen(Rotat_ba, Rotation_ba);
    cout << "\n" << " Rotation matrix: " << "\n";
    cout << Rotation_ba << endl;
    Eigen::Vector3d euler_ba = Rotation_ba.eulerAngles(2,1,0); // euler angles here is right multiplication convention mat == AngleAxisf(ea[0], Vector3f::UnitZ())
                                                                                                                        // * AngleAxisf(ea[1], Vector3f::UnitX())

    cout << "Euler angales: " << "\n";                                                                                                                    // * AngleAxisf(ea[2], Vector3f::UnitZ()); 
    cout << euler_ba(2)/DegreeToRadians <<" "<<euler_ba(1)/DegreeToRadians <<" "<< euler_ba(0)/DegreeToRadians << endl;

    cout << "Translation: " << "\n";
    cout << tvec_ba << endl;

    double e_ba = verify::error_(pic_points, check_front_image_pts);
    cout << " Calibration error: " << "\n";
    cout << e_ba << endl;

return 0;
}
