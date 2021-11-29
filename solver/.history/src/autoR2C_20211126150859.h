#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

double height_of_reflector=0.65;
Point pt(-1,-1);
bool newCoords = false;

tuple<double, double, double> binarySearch(FileNode objects, int x, int low, int high);
void mouse_callback(int  event, int  x, int  y, int  flag, void *param);

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

