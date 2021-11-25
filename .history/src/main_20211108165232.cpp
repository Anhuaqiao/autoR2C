# include "calib_pic.h"
# include "click_pic.h"
# include "get_rd.hpp"
# include "calib_verification.hpp"
# include "ba.hpp"
# include <eigen3/Eigen/Core>
# include <eigen3/Eigen/Geometry>
# include <opencv2/core/eigen.hpp>

#define DegreeToRadians 0.017453293

using namespace std;
using namespace cv;



int main(int argc, char** argv){
    const int imageWidth = 1920; //定义图片大小，即摄像头的分辨率  
    const int imageHeight = 1200;
    Size imageSize = Size(imageWidth, imageHeight);

    vector<Point2f> pic_points;

    // 相机内参
    Mat cameraMatrix = (Mat_<double>(3, 3) << 1014.887185159176, 0.0, 930.3792328037258, 
                                                0.0, 1019.6356956271103, 487.9419547383717, 
                                                0., 0., 1.);
    // 相机畸变
    Mat distCoeff = (Mat_<double>(1, 5) << -0.031066151822322162, 0.034324349626048335, -0.062279666374504286, 0.034648884538340426); //-8.2932482995413426e-01


    Mat1d rVec;//init rvec 
    Mat1d tVec;//init tvec
    rVec << 1.4836460324023821e+00, -3.7036077218133831e-02, 5.9895973152910266e-02;
    tVec <<  -9.7594351432367624e-02, 1.6374086484437917e+00, 1.5281568834104853e+00;

    // get the json date for verification
    FileStorage f_("../img_data_verification/config.yaml", FileStorage::READ);
    FileNode features_ = f_["features"];
    Get_rd rd_ver;
    rd_ver.store_rd(features_);
    
    // project on to the pixel coordinate
    vector<Point2f>check_front_image_pts;
    projectPoints( rd_ver.loc, rVec, tVec, cameraMatrix, distCoeff, check_front_image_pts );

    // draw it on images
    String folderpath = "../img_data_verification/*.jpg";
    vector<String> filenames;
    cv::glob(folderpath, filenames);

    cout << " size: " << check_front_image_pts.size() << endl;

    for (int j=0; j<filenames.size(); j++)
    {   
        Mat img_ver = imread( filenames[j], CV_LOAD_IMAGE_COLOR );
        
        for(int i=j*4; i<j*4+4 ; i++){
            cout << check_front_image_pts[i].x << " " << check_front_image_pts[i].y << endl;
            drawMarker(img_ver, check_front_image_pts[i], Scalar(0,0,255), MarkerTypes::MARKER_CROSS, 20);
    }
    std::ostringstream name;
    name << "../test/" << j << ".jpg";
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
    cout << "Euler angales: " << "\n";                                                                                                                    // * AngleAxisf(ea[2], Vector3f::UnitZ()); 
    cout << euler_(2)/DegreeToRadians <<" "<<euler_(1)/DegreeToRadians <<" "<< euler_(0)/DegreeToRadians << endl;
    cout << "Translation: " << "\n";
    cout << tVec << endl;

    double e = verify::error_(pic_points, check_front_image_pts);
    cout << " Calibration error: " << "\n";
    cout << e << endl;

    string result_path = "../results/transformation.yaml";
    FileStorage fs(result_path, FileStorage::WRITE);

    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeff" << distCoeff;
    fs << "rvec" << rVec;
    fs << "tvec" << tVec;
    fs << "rotationMatrix" << Rotat;
    fs << "MSE" << e;
    fs.release();

    Mat_<double> trans_matrix(4,4);
    trans_matrix << Rotat.at<double>(0,0), Rotat.at<double>(0,1), Rotat.at<double>(0,2), tVec.at<double>(0,0), 
                    Rotat.at<double>(1,0), Rotat.at<double>(1,1), Rotat.at<double>(1,2), tVec.at<double>(1,0),
                    Rotat.at<double>(2,0), Rotat.at<double>(2,1), Rotat.at<double>(2,2), tVec.at<double>(2,0),
                    0, 0, 0, 1;
return 0;
}
