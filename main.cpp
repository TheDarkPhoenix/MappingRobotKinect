#include <opencv2\calib3d.hpp>
#include <opencv2\imgproc.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\rgbd.hpp>

#include <iostream>
#include "motor.h"
#include "types.h"
#include "util.h"
#include "robot.h"
#include "mapa.h"

#define UART
//#define MOTOR

using namespace cv;
using namespace std;

void CallBackFunc(int event, int x, int y, int flags, void* vdisp)
{
     const double max_z = 1.0e4;
     if  ( event == EVENT_LBUTTONDOWN )
     {
        Mat* dis = (Mat*) vdisp;
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        Vec3f point = dis->at<Vec3f>(y,x);
        if(!(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z))
            cout << point[0] << ' ' << point[1] << ' ' << point[2] << endl;
     }
}

int main()
{
    VideoCapture capture( CV_CAP_OPENNI );
	if (!capture.isOpened())
	{
		cout << "fail" << endl;
		getchar();
		return -1;
	}
#ifdef MOTOR
	KinectMotor motor;
	int mangle = 0, lmangle = 0;

	if (!motor.Open()) // Open motor device
		return 1;
#endif

#ifdef UART
	HANDLE hSerial;
    DCB dcbSerialParams = {0};
    COMMTIMEOUTS timeouts={0};

    init(hSerial, dcbSerialParams, timeouts);

    sendread(hSerial, 1);
    sendread(hSerial, 125);
    sendread(hSerial, 2);
    sendread(hSerial, 125);
    sendread(hSerial, 5);
    sendread(hSerial, 6);

	robot robocik;
#endif

	int ch;

	Mat disp, disp3d, dm, img;

#ifdef MOTOR
	namedWindow("img");
	createTrackbar("angle","img",&mangle,62);
#endif

	mapa mapka;

	for (int i = 0; i < 10; ++i)
	{
		capture >> dm;
		capture.retrieve(disp, CV_CAP_OPENNI_DISPARITY_MAP);
		capture.retrieve(disp3d, CV_CAP_OPENNI_POINT_CLOUD_MAP);
		capture.retrieve(img, CV_CAP_OPENNI_BGR_IMAGE);
	}

	mapka.init(disp3d);

    while(true)
    {
		capture >> dm;
		capture.retrieve(disp, CV_CAP_OPENNI_DISPARITY_MAP);
        capture.retrieve(disp3d, CV_CAP_OPENNI_POINT_CLOUD_MAP);
		capture.retrieve(img, CV_CAP_OPENNI_BGR_IMAGE);
		
		setMouseCallback("img", CallBackFunc, mapka.getnDisp3d());

		mapka.run(disp, disp3d);

		imshow("objects", mapka.getObjects());
		imshow("submap", mapka.getSubMap());
		//imshow("mapa", mapka.getMap());
		imshow("img", img);
        imshow("disparity", disp);

        ch = waitKey(1);
        if((char)ch==27 ) break;

#ifdef MOTOR
		if(mangle != lmangle)
		{
			mangle -= 31;
			motor.Move(mangle);
		}
		lmangle = mangle;
#endif

#ifdef UART
		switch (ch)
        {
            case 'w':
                robocik.accelerate(1, hSerial);
                break;
            case 's':
                robocik.accelerate(-1, hSerial);
                break;
            case 'd':
                robocik.turn(1, hSerial);
                break;
            case 'a':
                robocik.turn(-1, hSerial);
                break;
            case 'q':
                robocik.stop(hSerial);
                break;
			case 'o':
				mapka.setFindLines(true);
				break;
        }
        robocik.update(hSerial);
		mapka.move(robocik.getPosition());
#endif
    }
#ifdef UART
	sendread(hSerial, 1);
    sendread(hSerial, 125);
    sendread(hSerial, 2);
    sendread(hSerial, 125);
    CloseHandle(hSerial);
#endif
    return 0;
}
