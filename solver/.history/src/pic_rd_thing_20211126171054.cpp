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