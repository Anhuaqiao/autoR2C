#include "pic_rd_thing.h"

Point pt(-1,-1);
bool newCoords = false;

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