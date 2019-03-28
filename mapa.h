#ifndef MAPA_H
#define MAPA_H

#include <opencv2\calib3d.hpp>
#include <opencv2\imgproc.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\rgbd.hpp>

#include "types.h"

using namespace cv;

class mapa
{
    private:
        punkt shift;
        Size rozmiary, rozmiaryWycinka, rozmiaryPoObroceniu;
        static const int shiftx = 500, shifty = 500;
		double alfa;
		double height;
		
		Mat disparity;
        Mat disp3d; // wartosci wyliczone z disparity w funkcji reprojectImageTo3D
		
		rgbd::RgbdPlane podloga;
		Mat efekt;
		Mat plaszczyzny;

		double a, b, c, d, err, minerr;
		int wybor;

		Mat objects; // z disparity na podstawie bestLine odrzucone pod³o¿e i pozosta³y same przeszkody

		static const int objScale = 0, freeScale = 80, addMap = 70;
		int rotation;
		Mat subMap;
		Mat aktualne;
		Mat kara;
		Mat mMapa;
		  
		bool bFindLines;
		int threshold, minLinLength, maxLineGap, ang;
		std::vector<Vec4i> lines;
		std::vector<Vec4i> pomocniczeLinie;
		Mat lineObjects;
		Mat lineObjectsMap;
        
		Mat nDisp3d;

        void findObjects();
        void visibility();
		void addObjectsToMap();
		void multiply(Mat& A, Mat& B, Mat& C);
		void rotate(cv::Mat& src, double angle, cv::Mat& dst);
		void calcNewDepthMap();
    public:
        mapa();
		~mapa();
		void init(Mat& disp3d1);
        void run(Mat& disp1, Mat& disp3d1);
        void move(pos p){shift.x = shiftx - p.y;shift.y = shifty - p.x; rotation = p.o*radtodeg;};
        Mat getMap() {return mMapa;};
        Mat getSubMap() {return mMapa(Rect((shift.x - rozmiaryPoObroceniu.width/2), (shift.y-rozmiaryPoObroceniu.height/2), rozmiaryPoObroceniu.width, rozmiaryPoObroceniu.height));};
        Mat getObjects() {return objects;};
		void setFindLines(bool b) {bFindLines = b;};
		Mat* getnDisp3d() {return &nDisp3d;};
};

#endif // MAPA_H

