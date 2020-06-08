// Cut_image.cpp : 定义控制台应用程序的入口点。

//#include<stddef.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include<direct.h>
#include<string>
#include <vector>
#include<io.h>
#include<omp.h>
using namespace std;
using namespace cv;
//剪切图片为m * n 块

 const string datasetPath = "E:\\Chrome下载\\20200606\\变化70-200\\";

void Cut_img(Mat src_img, int m, int n, vector<Mat> ceil_img, string file_name) {

	int t = m * n;
	int height = src_img.rows;
	int width = src_img.cols;
	int ceil_height = height / m;
	int ceil_width = width / n;

	Mat roi_img, tmp_img;
	Point p1, p2;
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++) {
			//p1 = 
			Rect rect(j * ceil_width, i * ceil_height, ceil_width, ceil_height);
			src_img(rect).copyTo(roi_img);
			ceil_img.push_back(roi_img);


			
			//			imshow("roi_img", roi_img);
			//getchar();
			//			waitKey(0);

			//			cout << "下一层" << endl;
			if (j == 0)
			{
				//				cout << "file_name:"<<file_name << endl;

				imwrite(datasetPath+"image_0\\" + file_name, roi_img);
			}
			else
			{
				imwrite(datasetPath+"image_1\\" + file_name, roi_img);
			}
			//			waitKey(0);
			tmp_img.release();
			roi_img.release();
			cvDestroyWindow("roi_img");
			//rectangle(i+j*ceil_width,j+i*ceil_height,);

		}
}


//void show_images(Vector<Mat> imgs, int n) {
//
//
//
//	//do something
//
//}


int main()

{
	string pattern = datasetPath+"*.png";
	vector<Mat> images;
	vector<String> fn;
	glob(pattern, fn, false);
	int count = fn.size();
	cout << "总共" << count << "张图片" << endl;
	if (_access((datasetPath+ "image_0\\").c_str(), 0)) {
		_mkdir((datasetPath + "image_0\\").c_str());
	}
	if (_access((datasetPath + "image_1\\").c_str(), 0)) {
		_mkdir((datasetPath + "image_1\\").c_str());
	}
	for (int i = 0; i < count; i++)
	{
		string file_name = fn[i].substr(datasetPath.length(), fn[i].length() - 4);
		//		cout << "文件路径："<<fn[i]<<endl;
		cout << "正在切割" << file_name << endl;
		Mat img = imread(fn[i], 1);
		//		imshow("src img", img);
		int m = 1;
		int n = 2;
		vector<Mat> ceil_img;
		Cut_img(img, m, n, ceil_img, file_name);
		img.release();
	}
	cv::waitKey(6000);

	return 0;

}
