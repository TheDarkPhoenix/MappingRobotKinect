#ifndef MOTOR_H
#define MOTOR_H

#include <XnUSB.h>

class KinectMotor
{
	private:
			XN_USB_DEV_HANDLE m_dev;
			bool m_isOpen;	
	public:
			KinectMotor();
			virtual ~KinectMotor();

			bool Open();
			void Close();
			bool Move(int angle);
};

#endif