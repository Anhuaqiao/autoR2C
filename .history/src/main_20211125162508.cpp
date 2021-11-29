# include "calib_pic.h"
# include "click_pic.h"
# include "get_rd.hpp"
# include "calib_verification.h"
# include "ba.h"
# include <eigen3/Eigen/Core>
# include <eigen3/Eigen/Geometry>
# include <opencv2/core/eigen.hpp>

#define DegreeToRadians 0.017453293

using namespace std;
using namespace cv;



int main(int argc, char** argv){
        cout << " here " << "\n";
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


    Mat_<double> rVec(3,1);//init rvec 
    Mat_<double> tVec(3,1);//init tvec

    // get the json date for verification
    FileStorage f_("../img_data_verification/config.yaml", FileStorage::READ);
    FileNode features_ = f_["features"];
    Get_rd rd_ver;
       cout << " here " << endl;
    rd_ver.store_rd(features_);
    
    cout << " here " << endl;

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
        
        for(int i=0; i<8 ; i++){
            cout << check_front_image_pts[i].x << " " << check_front_image_pts[i].y << endl;
            drawMarker(img_ver, check_front_image_pts[i], Scalar(0,0,255), MarkerTypes::MARKER_CROSS, 20);
    }
    std::ostringstream name;
    name << "../test/" << j << ".jpg";
    imwrite(name.str(), img_ver);
    }

return 0;
}
