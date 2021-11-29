#include "pic_rd_thing.h"

Point pt(-1,-1);
bool newCoords = false;

void mouse_callback(int  event, int  x, int  y, int  flag, void *param);

void click_out_point::click(Mat frame){
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

void click_out_point::print_point(){
    for (const auto &i : pts ){
        cout<<"x = "<< i.x <<", y = " << i.y << endl;
    }
};

void click_out_point::store_point(string path){
        ofstream store_point;
        store_point.open(path);
        for (const auto &i : pts ){
            store_point << i.x <<" " << i.y << "\n";
        }
        store_point.close();
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
};

Eliminator::Eliminator(Mat img_1,  Mat cameraMatrix,  Mat distCoeff, Size imageSize, int imageWidth=1920, int imageHeight = 1200){
    Mat mapx, mapy;
    Mat R = Mat::eye(3, 3, CV_32F);
    initUndistortRectifyMap(cameraMatrix, distCoeff, R, cameraMatrix, imageSize, CV_32FC1, mapx, mapy);
    remap(img_1,img_1,mapx,mapy, INTER_LINEAR);
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
    double height_of_reflector=0.65;
    for (int i=0 ; i<objects.size(); i++)
        if( int(objects[i]["objId"]) == x) return make_tuple(floorf((float)objects[i]["objContour"]["x"])/100.0, floorf((float)objects[i]["objContour"]["y"])/100.0, height_of_reflector);
}