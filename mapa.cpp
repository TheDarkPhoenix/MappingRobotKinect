#include "mapa.h"
#include <iostream>

using namespace std;

void mapa::rotate(cv::Mat& src, double angle, cv::Mat& dst)
{
    int len = std::max(src.cols, src.rows);
    cv::Point2f pt(src.cols/2, src.rows/2);
    cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);
    cv::warpAffine(src, dst, r, cv::Size(len, len));
}

void mapa::multiply(Mat& A, Mat& B, Mat& C)
{
    for (int i = 0; i < A.cols; ++i)
        for (int j = 0; j < A.rows; ++j)
            C.at<uint8>(i,j) = A.at<uint8>(i,j) * B.at<uint8>(i,j);
}

mapa::mapa()// : alfa(0.267705973) // 17 stopni
{
    rozmiary.height = 1000;
    rozmiary.width = 1000;
    rozmiaryWycinka.height = 200;
    rozmiaryWycinka.width = 160;

    rozmiaryPoObroceniu.width = rozmiaryPoObroceniu.height = sqrt((rozmiaryWycinka.height*rozmiaryWycinka.height) + ((rozmiaryWycinka.width/2)*(rozmiaryWycinka.width/2)))*2;
   
	mMapa = Mat(rozmiary, CV_8U, Scalar(0));
    lineObjectsMap = Mat(rozmiary, CV_8U, Scalar(0));

    shift.x = rozmiary.width/2;
    shift.y = rozmiary.height/2;
    
    kara = Mat(rozmiaryWycinka.height, rozmiaryWycinka.width, CV_8U);

    aktualne = Mat(kara.size(), CV_8U, Scalar(freeScale));

    rotation = 0;
	podloga.setMinSize(20000);
	podloga.setThreshold(0.016);
	minerr = 1000;

	alfa = 0;

	/*threshold = 5;
	minLinLength = 4;
	maxLineGap = 4;
	ang = 180;*/

	/*namedWindow("edges", 0);
	resizeWindow("edges", rozmiaryWycinka.width * 3, rozmiaryWycinka.height *3);
	createTrackbar("threshold","edges",&threshold,20);
	createTrackbar("minLinLength","edges",&minLinLength,20);
	createTrackbar("maxLineGap","edges",&maxLineGap,20);
	createTrackbar("ang","edges",&ang,360);*/

	bFindLines = false;
}

mapa::~mapa()
{
	/*imwrite("D:/mapki/mapa3.jpg", mMapa);
	imwrite("D:/mapki/linemap3.jpg", lineObjectsMap);*/
}


void mapa::visibility()
{
    Vec3f point;
    Vec3i pointmap;
    int val = 1;
    kara = Mat(rozmiaryWycinka.height, rozmiaryWycinka.width, CV_8U, Scalar(0));
    for (int i = 0; i < disp3d.size().height; ++i)
    {
        for (int j = 0; j < disp3d.size().width; ++j)
        {
            point = disp3d.at<Vec3f>(i, j);
            pointmap[0] = (point[0]*100) + rozmiaryWycinka.width/2;
            pointmap[2] = (-point[2]*100) + rozmiaryWycinka.height;
            if (pointmap[0] > 0 && pointmap[0] < rozmiaryWycinka.width && pointmap[2] > 0 && pointmap[2] < rozmiaryWycinka.height)
            {
				kara.at<uint8>(pointmap[2], pointmap[0]) = val;
			}
        }
    }
    Mat mapar;
    Mat element = getStructuringElement( 0, Size( 9, 9 ), Point( 2, 2 ) );
    morphologyEx(kara, mapar, 3, element);
    kara = mapar;
}

void mapa::findObjects()
{
	Vec3f point;
	for (int i = 0; i < nDisp3d.size().height; ++i)
    {
        for (int j = 0; j < nDisp3d.size().width; ++j)
        {
            point = nDisp3d.at<Vec3f>(i, j);
			if (fabs(point[1] + height) < 0.04 || point[2] > 1.5)
			{
				objects.at<uint8>(i, j) = 0;
			}
        }
    }
}

void mapa::addObjectsToMap()
{
	Vec3f point;
	Vec3i pointmap;
	subMap = Mat(rozmiaryWycinka, CV_8U, Scalar(0));
	aktualne = Mat(kara.size(), CV_8U, Scalar(freeScale));
	for (int i = 0; i < nDisp3d.size().height; ++i)
    {
        for (int j = 0; j < nDisp3d.size().width; ++j)
        {
            point = nDisp3d.at<Vec3f>(i, j);
            if(objects.at<uint8>(i, j) > 0)
            {
                pointmap[0] = (point[0]*100) + rozmiaryWycinka.width/2;
                pointmap[2] = (-point[2]*100) + rozmiaryWycinka.height;
                if (pointmap[0] > 0 && pointmap[0] < rozmiaryWycinka.width && pointmap[2] > 0 && pointmap[2] < rozmiaryWycinka.height &&(mMapa.at<__int8>(pointmap[2] + shift.y, pointmap[0] + shift.x)+addMap)< 255)
                {
                    if (aktualne.at<uint8>(pointmap[2], pointmap[0]) == freeScale)
                    {
                        aktualne.at<uint8>(pointmap[2], pointmap[0]) = objScale;
                        subMap.at<uint8>(pointmap[2], pointmap[0]) += addMap;
                    }
                }
            }
        }
    }
	int w = (rozmiaryPoObroceniu.width - rozmiaryWycinka.width)/2;
	int h = ((rozmiaryPoObroceniu.height/2) - rozmiaryWycinka.height);

	Mat aktualnyFragmentMapy = mMapa(Rect((shift.x - rozmiaryPoObroceniu.width/2), (shift.y-rozmiaryPoObroceniu.height/2), rozmiaryPoObroceniu.width, rozmiaryPoObroceniu.height));
	
	visibility();
    Mat wynik = Mat(kara.size(), CV_8U, Scalar(0));
    multiply(aktualne, kara, wynik);

    Mat wynikObrocony = Mat(rozmiaryPoObroceniu.width, rozmiaryPoObroceniu.height, CV_8U, Scalar(0));
    Mat wynikDoObrocenia = wynikObrocony(Rect(w, h, rozmiaryWycinka.width, rozmiaryWycinka.height));
    wynik.copyTo(wynikDoObrocenia);
    rotate(wynikObrocony, rotation, wynikObrocony);
 
    aktualnyFragmentMapy -= wynikObrocony;

	Mat obroconyFragment = Mat(rozmiaryPoObroceniu.width, rozmiaryPoObroceniu.height, CV_8U, Scalar(0));
    Mat fragmentDoObrocenia = obroconyFragment(Rect(w, h, rozmiaryWycinka.width, rozmiaryWycinka.height));
    subMap.copyTo(fragmentDoObrocenia);
    rotate(obroconyFragment, rotation, obroconyFragment);
    aktualnyFragmentMapy += obroconyFragment;
}

void mapa::calcNewDepthMap()
{
	nDisp3d = Mat(disp3d.size(), CV_32FC3, Scalar(0));
	Vec3f point;
	Vec3f pointp;
	for (int i = 0; i < disp3d.size().height; ++i)
    {
        for (int j = 0; j < disp3d.size().width; ++j)
        {
            point = disp3d.at<Vec3f>(i, j);
			pointp[0] = point[0];
			pointp[1] = point[1]*cos(alfa) - point[2]*sin(alfa);
			pointp[2] = point[2]*cos(alfa) + point[1]*sin(alfa);
			nDisp3d.at<Vec3f>(i, j) = pointp;
        }
    }
}

void mapa::init(Mat& disp3d1)
{
    disp3d = disp3d1;

	calcNewDepthMap();

	podloga(disp3d, efekt, plaszczyzny);
	for (int i = 0; i < plaszczyzny.size().height; ++i)
	{
		a = plaszczyzny.at<float>(i, 0);
		b = plaszczyzny.at<float>(i, 1);
		c = plaszczyzny.at<float>(i, 2);
		d = plaszczyzny.at<float>(i, 3);
		err = abs((c/b)+0.342);
		if (err < minerr)
		{
			minerr = err;
			wybor = i;
		}
	}

	b = plaszczyzny.at<float>(wybor, 1);
	c = plaszczyzny.at<float>(wybor, 2);
	d = plaszczyzny.at<float>(wybor, 3);
	//cout << c/b << endl;

	alfa = atan((-c/b));
	height = d;
	cout << alfa << endl;
	cout << d << endl;
	cout << endl;
}

void mapa::run(Mat& disp1, Mat& disp3d1)
{
    disparity = disp1;
    disp3d = disp3d1;

	calcNewDepthMap();
	
	disparity.copyTo(objects);
	findObjects();

	minerr = 1000;

    addObjectsToMap();
}

