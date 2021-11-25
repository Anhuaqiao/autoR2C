#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>

using namespace cv;
using namespace std;

class nh
{
private:
    Mat Tp_, Tr_;

public:

void normalization2( vector<Point3f>& in, vector<Point3f>& out){
    float s1, s2, m1, m2;
    float n = 0;
    float d, c;
    s1 = 0;
    s2 = 0;
    for(int i=0; i< in.size(); i++){
        s1 += in[i].x;
        s2 += in[i].y;
    }

    m1 = s1/in.size();
    m2 = s2/in.size();

    for(int i=0; i< in.size(); i++){
        n += sqrt((in[i].x - m1)*(in[i].x - m1) + (in[i].y - m2)*(in[i].y - m2));
    }
    

    d = n/in.size();
    c = sqrt(2)/d;

    // cout << "n: " << n << endl;
    // cout << "d: " << d << endl;
    // cout << "c: " << c << endl;

    Mat T = (Mat_<float>(3, 3) << c, 0., -c*m1, 
                                    0.,c, -c*m2, 
                                    0., 0., 1.);

    Tp_ = T;

    cv::transform(in, out, T);
}

void normalization3( vector<Point3f>& in, vector<Point3f>& out){
        float s1, s2, m1, m2;
        float n = 0;
        float d, c;
        s1 = 0;
        s2 = 0;
        for(int i=0; i< in.size(); i++){
            s1 += in[i].x;
            s2 += in[i].y;
        }

        m1 = s1/in.size();
        m2 = s2/in.size();

        for(int i=0; i< in.size(); i++){
            n += sqrt((in[i].x - m1)*(in[i].x - m1) + (in[i].y - m2)*(in[i].y - m2));
        }

        d = n/in.size();
        c = sqrt(2)/d;

        Mat T = (Mat_<double>(3, 3) << c, 0., -c*m1, 
                                        0.,c, -c*m2, 
                                        0., 0., 1.);

        Tr_ = T;

        cv::transform(in, out, T);
    }

    void nohomo2( vector<Point3f>& pic, vector<Point2f>& npic){
        for ( int i=0 ; i<pic.size() ; i++ ){
            npic.push_back(Point2f( pic[i].x, pic[i].y ));
        }
    }

    void nohomo3(vector<Point3f>& ra, vector<Point3f>& nra ){
        for ( int i=0 ; i<ra.size() ; i++ ){
            nra.push_back(Point3f(ra[i].x, ra[i].y, 0));
        }
    }

    void homo2( vector<Point3f>& pic, vector<Point2f>& npic ){
        for ( int i=0 ; i<npic.size() ; i++ ){
            pic.push_back(Point3f( npic[i].x, npic[i].y, 1 ));
        }
    }
    void homo3( vector<Point3f>& ra, vector<Point3f>& nra ){
        for ( int i=0 ; i<nra.size() ; i++ ){
            ra.push_back(Point3f(nra[i].x, nra[i].y, 1));
        }
    }

    void de_normalization( vector<Point3f>& in, vector<Point3f>& out){cv::transform(in, out, Tp_.inv());}

    void rdata_preprocessor( vector<Point3f>& in, vector<Point3f>& out ){
        vector<Point3f> hom, normalhom, nohom;
        homo3(hom, in);
        normalization3(hom, normalhom);
        nohomo3(normalhom, out);
    }

    void rdata_preprocessor2( vector<Point3f>& in, vector<Point3f>& out ){
        vector<Point3f> hom, normalhom, nohom;
        homo3(hom, in);
        cv::transform(hom, normalhom, Tr_);
        nohomo3(normalhom, out);
    }

    void pdata_preprocessor( vector<Point2f>& in, vector<Point2f>& out){
        vector<Point3f> phom, pnormalhom, pnohom;
        homo2(phom, in);
        normalization2(phom, pnormalhom);
        nohomo2( pnormalhom, out);
    }

    void pdata_postprocessor( vector<Point2f>& in, vector<Point2f>& out){
        vector<Point3f> phom, pdenormalhom, pnohom;
        homo2(phom, in);
        de_normalization(phom, pdenormalhom);
        nohomo2( pdenormalhom, out);
    }
};
