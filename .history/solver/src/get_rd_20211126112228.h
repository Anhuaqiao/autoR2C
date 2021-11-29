# ifndef Get_RD
# define Get_RD

# include <opencv2/opencv.hpp>
# include <iostream>
# include <vector>
# include <string>

using namespace cv;
using namespace std;

double height_of_reflector=0.65;

tuple<double, double, double> binarySearch(FileNode objects, int x, int low, int high);


class Get_rd{
    private:
        int FLAG;
        int n;    
        vector<int> id_all; 
        vector<int> id;
    public:
        vector<Point3f> loc;
        void store_rd(FileNode f);
        int print_rd();
};

void Get_rd::store_rd(FileNode f){
    string path;

    for (FileNodeIterator it= f.begin(); it!= f.end();++it)
    {
        int filename = (int)(*it)["file_name"];
        path = "../radar_data/" + to_string(filename) + ".json";
        for (size_t i = 0; i < (int)(*it)["id"].size(); i++)
        {
            id.push_back((int)(*it)["id"][i]);
        }

        FileStorage reflects(path, FileStorage::READ);
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

# endif