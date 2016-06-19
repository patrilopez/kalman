#include <stdio.h>

#include <math.h>

#include <vector>
#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
//#include <opencv2/highgui/highgui_c.h>

using namespace cv;
using namespace std;
using namespace std::chrono;


struct mouse_info_struct { int x, y; };
struct mouse_info_struct mouse_info = { -1,-1 }, last_mouse;

vector<Point> mousev, kalmanv;

void on_mouse(int event, int x, int y, int flags, void* param) {
	//if (event == CV_EVENT_LBUTTONUP) 
	{
		last_mouse = mouse_info;
		mouse_info.x = x;
		mouse_info.y = y;

		//		cout << "got mouse " << x <<","<< y <<endl;
	}
}

int main(int argc, char * const argv[]) {
	Mat img(500, 500, CV_8UC3);
	//KalmanFilter KF(4, 2, 0);
	Mat_<float> x(4, 1); /* (x, y, Vx, Vy) */
	Mat processNoise(4, 1, CV_32F);
	Mat_<float> measurement(2, 1); measurement.setTo(Scalar(0));
	char code = (char)-1;
//Inicializamos los parametros de kalman
	x_pred = Mat::zeros(4, 1, CV_32F);
	    x_est = Mat::zeros(4, 1, CV_32F);
	    A = Mat::eye(4, 4, CV_32F);
	A = ( 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1); //Jacobiana de funcion f, en la primera iteración deltat=0
		At = Mat::eye(4, 4, CV_32F);

	    Q = Mat::eye(4, 4, CV_32F);
	    H = Mat::zeros(2, 4, CV_32F);
	Ht = Mat::zeros(4, 2, CV_32F);
	    R = Mat::eye(2, 2, CV_32F);

	    P_ant = Mat::zeros(4, 4, CV_32F);
	    P = Mat::zeros(4, 4, CV_32F);
	    K = Mat::zeros(4, 2, CV_32F);
		Id = Mat::eye(2, 2, CV_32F);
		aux=Mat::zeros(2,2, CV_32F);

		f=Mat::zeros(4, 1, CV_32F);
		h=Mat::zeros(2, 1, CV_32F);

	
//-------------------------------------------

	namedWindow("mouse kalman");
	setMouseCallback("mouse kalman", on_mouse, 0);

	for (;;)
	{
		if (mouse_info.x < 0 || mouse_info.y < 0) {
			imshow("mouse kalman", img);
			waitKey(30);
			continue;
		}
		x(0) = mouse_info.x;
		x(1) = mouse_info.y;
		x(2) = 0;
		x(3) = 0;
	
		
		H=(1,0,0,0,0,1,0,0); //jacobiana de la funcion h
		transpose(A, At);
		//transpose(W, Wt);
		//transpose(V, Vt);
		transpose(H, Ht);
		mousev.clear();
		kalmanv.clear();
	

		for (;;)
		{
			
	auto start = std::chrono::high_resolution_clock::now();

	
	transpose(A, At);		
		//--------------Predicción------------------

					x_pred=0;
					P=A*P_ant*At+W*Q*Wt;
		//------------------------------------------------
			
			Point predictPt(x_pred.at<float>(0), x_pred.at<float>(1)); //esta linea no la entiendo muy bien, es necesaria?
			measurement(0) = mouse_info.x;
			measurement(1) = mouse_info.y;

			Point measPt(measurement(0), measurement(1));
			mousev.push_back(measPt);
			
			//-------------------Correcion----------------
							
				aux=(1/(H*P*Ht+V*R*Vt);
				K=P*Ht*aux;
				x_est=x_pred+K*(measurement-h);
				P=(Id-K*H)*P;

				P_ant=P;
			//--------------------------------------

			
			Point statePt(x_est.at<float>(0), x_est.at<float>(1));
			kalmanv.push_back(x);

			// plot points
#define drawCross( center, color, d )                                 \
line( img, Point( center.x - d, center.y - d ),                \
Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); \
line( img, Point( center.x + d, center.y - d ),                \
Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )

			img = Scalar::all(0);
			drawCross(x, Scalar(255, 255, 255), 5);
			drawCross(measPt, Scalar(0, 0, 255), 5);
			

			for (int i = 0; i < mousev.size() - 1; i++) {
				line(img, mousev[i], mousev[i + 1], Scalar(255, 255, 0), 1);
			}
			for (int i = 0; i < kalmanv.size() - 1; i++) {
				line(img, kalmanv[i], kalmanv[i + 1], Scalar(0, 255, 0), 1);
			}
e;

			imshow("mouse kalman", img);
			code = (char)waitKey(100);

			if (code > 0)
				break;
		auto end = std::chrono::high_resolution_clock::now();

		   std::chrono::duration<double> deltat = end-start;
		A = ( 1, 0, deltat.count, 0, 0, 1, 0, deltat.count, 0, 0, 1, 0, 0, 0, 0, 1); //Jacobiana de funcion f
	f=(x(0)+deltat.count*x(2),x(1)+deltat.count*x(3),x(2),x(3));
	h=(x(0),x(1));
		}
		if (code == 27 || code == 'q' || code == 'Q')
			break;
	}

	return 0;
}

