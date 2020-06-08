
//需要修改的参数：程序中共计5个位置需要根据应用场景来修改参数（已在代码旁做了备注）

//注意：第5处需要修改的地方，用来选择是运行SGBM或者运行BM算法，二者选一运行即可。


#include <opencv2/opencv.hpp>  
#include <iostream>  

using namespace std;
using namespace cv;

const int imageWidth  = 1920;                      //摄像头单目的分辨率########--【需要调整参数的位置1】--#############
const int imageHeight = 1080;

Size imageSize = Size(imageWidth, imageHeight);

Mat rgbImageL, grayImageL;
Mat rgbImageR, grayImageR;
Mat rectifyImageL, rectifyImageR;

Rect validROIL;                                   //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域  
Rect validROIR;

Mat mapLx, mapLy, mapRx, mapRy;                   //映射表  
Mat Rl, Rr, Pl, Pr, Q;                            //校正旋转矩阵R，投影矩阵P, 重投影矩阵Q
Mat xyz;                                          //三维坐标

Point origin;                                     //鼠标按下的起始点
Rect selection;                                   //定义矩形选框
bool selectObject = false;                        //是否选择对象




Ptr<StereoBM> bm = StereoBM::create(16, 9);


//########--【以下双目的标定参数为：需要调整参数的位置2】--#############
//相机双目标定的结果与如下各参数的对应关系见：双目标定结果说明.pdf，pdf文档位于main.cpp（即本文档）同级文件夹--#############

/*左目相机标定参数------------------------
fc_left_x   0            cc_left_x
0           fc_left_y    cc_left_y
0           0            1
-----------------------------------------*/
/*  下面是40CM标定参数     */
//
//Mat cameraMatrixL = (Mat_<double>(3, 3) << 1382.4856345064            ,             0     ,     906.617950626835,
//	0     ,     1384.93784798172     ,     498.149671751999,
//	0    ,                     0     ,                    1);
//
//
//Mat distCoeffL = (Mat_<double>(5, 1) << -0.124641589889793 ,-0.056290302726278,     0,    0,     0.00000);
//                                     //[kc_left_01,  kc_left_02,  kc_left_03,  kc_left_04,   kc_left_05]
//
//
///*右目相机标定参数------------------------
//fc_right_x   0              cc_right_x
//0            fc_right_y     cc_right_y
//0            0              1
//-----------------------------------------*/
//Mat cameraMatrixR = (Mat_<double>(3, 3) << 1363.38553436494        ,                 0    ,      985.836461780174,
//	0  ,        1366.55930868123      ,    530.013548428435,
//	0  ,                       0      ,                   1);
//
//
//Mat distCoeffR = (Mat_<double>(5, 1) << -0.15166972533886   ,      0.012440527135291,     0,     0,      0.00000);
//                                     //[kc_right_01,  kc_right_02,  kc_right_03,  kc_right_04,   kc_right_05]
//
//
//Mat T = (Mat_<double>(3, 1) << -120.546576192684     ,     2.69938798963161 ,- 3.35500489837967);    //T平移向量
//							 //[T_01,        T_02,       T_03]
//
//Mat R = (Mat_<double>(3, 3) << 0.999871725138258       ,0.00210612454263908  ,      0.0158775787970298,
//	- 0.00203296756672541      ,   0.999987250620167, - 0.00462229812893056,
//	- 0.0158871115032796  ,     0.00458942660154466       ,  0.999863259076736);   //R旋转矩阵
//							  //[rec_01,     rec_02,     rec_03]

//**********************************************************
//下面是70CM标定结果

//Mat cameraMatrixL = (Mat_<double>(3, 3) << 1259.07112590554     ,                 0   ,     892.562529319583,
//	0    ,       1259.6121706671     ,     513.122924246335,
//	0     ,                    0         ,                1);
//
//
//Mat distCoeffL = (Mat_<double>(5, 1) << -0.0895475239880661   ,    0.00732660494976466, 0, 0, 0.00000);
////[kc_left_01,  kc_left_02,  kc_left_03,  kc_left_04,   kc_left_05]
//
//
///*右目相机标定参数------------------------
//fc_right_x   0              cc_right_x
//0            fc_right_y     cc_right_y
//0            0              1
//-----------------------------------------*/
//Mat cameraMatrixR = (Mat_<double>(3, 3) <<   1234.64178416587       ,                  0       ,   957.541578387652,
//	0      ,    1236.28107550559      ,    546.108334759383,
//	0   ,                      0       ,                  1);
//
//
//Mat distCoeffR = (Mat_<double>(5, 1) <<-0.0934604627155616      ,  0.0590793971205513, 0, 0, 0.00000);
////[kc_right_01,  kc_right_02,  kc_right_03,  kc_right_04,   kc_right_05]
//
//
//Mat T = (Mat_<double>(3, 1) << -120.06704094085     ,     2.22269874062499, - 6.1548885472868);    //T平移向量
//							 //[T_01,        T_02,       T_03]
//
//Mat R = (Mat_<double>(3, 3) << 0.999630920891178   ,    0.00121122057306768     ,   0.0271395457400981,
//	- 0.001134427024345    ,     0.999995310085054, - 0.00284479932920529,
//	- 0.0271428641374107    ,   0.00281296153908808      ,   0.999627606748532);   //R旋转矩阵
//							  //[rec_01,     rec_02,     rec_03]

//下面是变换距离标定结果

Mat cameraMatrixL = (Mat_<double>(3, 3) << 1367.07495048576,0,0,
	0	,1369.39137685934,	0,
	888.713546417017,	508.595752458459	,1);


Mat distCoeffL = (Mat_<double>(5, 1) << -0.121236003308311 ,- 0.0156428432900074, 0, 0, 0.00000);
//[kc_left_01,  kc_left_02,  kc_left_03,  kc_left_04,   kc_left_05]


/*右目相机标定参数------------------------
fc_right_x   0              cc_right_x
0            fc_right_y     cc_right_y
0            0              1
-----------------------------------------*/
Mat cameraMatrixR = (Mat_<double>(3, 3) <<
	1358.16737768494        ,                 0     ,     954.568556468041,
	0    ,      1361.05484604869      ,    548.118385198964,
	0   ,                      0   ,                      1);


Mat distCoeffR = (Mat_<double>(5, 1) << -0.132140657541619 ,- 0.0168774409980463, 0, 0, 0.00000);
//[kc_right_01,  kc_right_02,  kc_right_03,  kc_right_04,   kc_right_05]


Mat T = (Mat_<double>(3, 1) << -119.750621692446 ,- 0.0186862706428725,	1.37454754952548);    //T平移向量
							 //[T_01,        T_02,       T_03]

Mat R = (Mat_<double>(3, 3) << 0.999712359444322   ,   0.000886388225623053    ,    0.0239669082315799,
	- 0.000787170221971175  ,       0.999991083418665, - 0.00414890903818027,
	- 0.0239703720728139  ,     0.00412884960720645   ,      0.999704143166173);   //R旋转矩阵
							  //[rec_01,     rec_02,     rec_03]


//########--双目的标定参数填写完毕-----------------------------------------------------------------------


//保存点云文件                                                   
static void saveXYZ(string filename, const Mat& mat)
{
	const double max_z = 16.0e4;
	FILE* fp;
	fopen_s(&fp, filename.c_str(), "wt");

	printf("%d %d \n", mat.rows, mat.cols);
	for (int y = 0; y < mat.rows; y++)
	{
		for (int x = 0; x < mat.cols; x++)
		{
			Point3f point = mat.at<Point3f>(y, x);
			//if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
			//fprintf(fp, "%f %f %f\n", point.x, point.y, point.z);
			//int lastx, int lasty;
			if (!isinf(point.z)) {
				fprintf(fp, "%f %f %f\n", point.x, point.y, point.z);
			}
		}
	}
	fclose(fp);
}

//--------------------------------------------------------------------------------------------------------
void GenerateFalseMap(cv::Mat &src, cv::Mat &disp)                             //颜色变换
{ 
	float max_val = 255.0f;
	float map[8][4] = { { 0,0,0,114 },{ 0,0,1,185 },{ 1,0,0,114 },{ 1,0,1,174 },
	{ 0,1,0,114 },{ 0,1,1,185 },{ 1,1,0,114 },{ 1,1,1,0 } };
	float sum = 0;
	for (int i = 0; i<8; i++)
		sum += map[i][3];

	float weights[8];   
	float cumsum[8];  
	cumsum[0] = 0;
	for (int i = 0; i<7; i++) {
		weights[i] = sum / map[i][3];
		cumsum[i + 1] = cumsum[i] + map[i][3] / sum;
	}

	int height_ = src.rows;
	int width_ = src.cols;
 
	for (int v = 0; v<height_; v++) {
		for (int u = 0; u<width_; u++) {
 
			float val = std::min(std::max(src.data[v*width_ + u] / max_val, 0.0f), 1.0f);

			int i;
			for (i = 0; i<7; i++)
				if (val<cumsum[i + 1])
					break;
 
			float   w = 1.0 - (val - cumsum[i])*weights[i];
			uchar r = (uchar)((w*map[i][0] + (1.0 - w)*map[i + 1][0]) * 255.0);
			uchar g = (uchar)((w*map[i][1] + (1.0 - w)*map[i + 1][1]) * 255.0);
			uchar b = (uchar)((w*map[i][2] + (1.0 - w)*map[i + 1][2]) * 255.0);
			 
			disp.data[v*width_ * 3 + 3 * u + 0] = b;                               //rgb内存连续存放 
			disp.data[v*width_ * 3 + 3 * u + 1] = g;
			disp.data[v*width_ * 3 + 3 * u + 2] = r;
		}
	}
}

//--BM算法立体匹配--------------------------------------------------------------------
void stereo_match_bm(int, void*)
{
	int blockSize = 18, uniquenessRatio = 5,  numDisparities = 11; //BM算法相关的参数，【需要调整参数的位置3，仅用于BM算法有效】--############

	bm->setBlockSize(2 * blockSize + 5);                           //SAD窗口大小，5~21之间为宜
	bm->setROI1(validROIL);                                        //左右视图的有效像素区域
	bm->setROI2(validROIR);
	bm->setPreFilterCap(61);                                       //预处理滤波器值
	bm->setMinDisparity(32);                                       //最小视差，默认值为0, 可以是负值，int型
	bm->setNumDisparities(numDisparities * 16 );                   //视差窗口，即最大视差值与最小视差值之差,16的整数倍
	bm->setTextureThreshold(10);
	bm->setUniquenessRatio(uniquenessRatio);                       //视差唯一性百分比,uniquenessRatio主要可以防止误匹配
	bm->setSpeckleWindowSize(100);                                 //检查视差连通区域变化度的窗口大小
	bm->setSpeckleRange(32);                                       //32视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零
	bm->setDisp12MaxDiff(-1);
	Mat disp;
	Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);               //显示  
	bm->compute(rectifyImageL, rectifyImageR, disp);               //输入图像必须为灰度图

	reprojectImageTo3D(disp, xyz, Q, true);                        //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)
	xyz = xyz * 16;

	disp.convertTo(disp, CV_32F, 1.0 / 16);                        //除以16得到真实视差值,disp.convertTo(disp, CV_32F, 1.0 );
	normalize(disp, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);

	medianBlur(disp8U, disp8U, 9);  //中值滤波

	Mat dispcolor(disp8U.size(), CV_8UC3);
	GenerateFalseMap(disp8U, dispcolor);

	imshow("disparity", dispcolor);
}

void stereo_match_sgbm(int, void*)                                         //SGBM匹配算法
{
	int mindisparity = 32;                                                 //最小视差
	int SADWindowSize = 16;                                                //滑动窗口的大小
	int ndisparities = 512;                                                //最大的视差，要被16整除
	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(mindisparity, ndisparities, SADWindowSize);

	int P1 = 4 * rectifyImageL.channels() * SADWindowSize* SADWindowSize;  //惩罚系数1
	int P2 = 32 * rectifyImageL.channels() * SADWindowSize* SADWindowSize; //惩罚系数2
	sgbm->setP1(P1);
	sgbm->setP2(P2);

	sgbm->setPreFilterCap(60);                                             //滤波系数
	sgbm->setUniquenessRatio(30);                                          //代价方程概率因子
	sgbm->setSpeckleRange(2);                                              //相邻像素点的视差值浮动范围
	sgbm->setSpeckleWindowSize(200);                                       //针对散斑滤波的窗口大小
	sgbm->setDisp12MaxDiff(1);                                             //视差图的像素点检查
	//sgbm->setMode(cv::StereoSGBM::MODE_HH);  

	Mat disp;
	sgbm->compute(rectifyImageL, rectifyImageR, disp);

	Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);                       //显示  

	reprojectImageTo3D(disp, xyz, Q, true);                                //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)
	xyz = xyz * 16;
	saveXYZ("res.xyz", xyz);
	disp.convertTo(disp, CV_32F, 1.0 / 16);                                //除以16得到真实视差值,disp.convertTo(disp, CV_32F, 1.0 );
	normalize(disp, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);

	medianBlur(disp8U, disp8U, 9);                                             //中值滤波

	Mat dispcolor(disp8U.size(), CV_8UC3);
	GenerateFalseMap(disp8U, dispcolor);

	imshow("disparity", rectifyImageL);

}
double last_x = 0, last_y = 0, last_z = 0;
double d = 0;
//--描述：鼠标操作回调--------------------------------------------------
static void onMouse(int event, int x, int y, int, void*)
{
	if (selectObject)
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);
	}
	
	switch (event)
	{
	case EVENT_LBUTTONDOWN:             //鼠标左按钮按下的事件
		origin = Point(x, y);
		selection = Rect(x, y, 0, 0);
		selectObject = true;
		cout << origin << "in world coordinate is: " << xyz.at<Vec3f>(origin) << endl;
		d =  sqrt((xyz.at<Point3f>(origin).x - last_x)* (xyz.at<Point3f>(origin).x - last_x) + (xyz.at<Point3f>(origin).y - last_y)* (xyz.at<Point3f>(origin).y - last_y)+ (xyz.at<Point3f>(origin).z - last_z)* (xyz.at<Point3f>(origin).z - last_z));
		if (fabs(xyz.at<Point3f>(origin).x - last_x) < 1.0 || fabs(xyz.at<Point3f>(origin).y - last_y) < 1.0) {   
			cout << "The distance between last ponit and current point is :" << d << endl;
		}
		last_x = xyz.at<Point3f>(origin).x;
		last_y = xyz.at<Point3f>(origin).y;
		last_z = xyz.at<Point3f>(origin).z;
		break;
	case EVENT_LBUTTONUP:               //鼠标左按钮释放的事件
		selectObject = false;
		if (selection.width > 0 && selection.height > 0)
			break;
	}
}


//--主函数---------------------------------------------------------------------
int main()
{	
	string imgLname;
	string imgRname;
	cin >> imgLname;
	cin >> imgRname;
//--立体校正-------------------------------------------------------------------
	
	stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
		0, imageSize, &validROIL, &validROIR);
	initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);

//--读取图片，【需要调整参数的位置4】----------------------------------------------------------------
	rgbImageL = imread(imgLname, CV_LOAD_IMAGE_COLOR);
	cvtColor(rgbImageL, grayImageL, CV_BGR2GRAY);
	rgbImageR = imread(imgRname, CV_LOAD_IMAGE_COLOR);
	cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);

	namedWindow("ImageL Before Rectify", WINDOW_NORMAL);  imshow("ImageL Before Rectify", grayImageL);
	namedWindow("ImageR Before Rectify", WINDOW_NORMAL);  imshow("ImageR Before Rectify", grayImageR);

//--经过remap之后，左右相机的图像已经共面并且行对准----------------------------------------------
	remap(grayImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
	remap(grayImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

//--把校正结果显示出来---------------------------------------------------------------------------
	Mat rgbRectifyImageL, rgbRectifyImageR;
	cvtColor(rectifyImageL, rgbRectifyImageL, CV_GRAY2BGR);  
	cvtColor(rectifyImageR, rgbRectifyImageR, CV_GRAY2BGR);
	imwrite("rectifyImageL.jpg", rectifyImageL);imwrite("rectifyImageR.jpg", rectifyImageR);

	namedWindow("ImageL After Rectify", WINDOW_NORMAL); imshow("ImageL After Rectify", rgbRectifyImageL);
	namedWindow("ImageR After Rectify", WINDOW_NORMAL); imshow("ImageR After Rectify", rgbRectifyImageR);


//--显示在同一张图上-----------------------------------------------------------------------------
	/*Mat canvas;
	double sf;
	int w, h;
	sf = 600. / MAX(imageSize.width, imageSize.height);
	w = cvRound(imageSize.width * sf);
	h = cvRound(imageSize.height * sf);
	canvas.create(h, w * 2, CV_8UC3);                                             //注意通道
	*/
//--左图像画到画布上-----------------------------------------------------------------------------
	/*Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                                //得到画布的一部分  
	resize(rgbRectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);    //把图像缩放到跟canvasPart一样大小  
	Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),                  //获得被截取的区域    
		cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
	//rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);                     
	cout << "Painted ImageL" << endl;

//--右图像画到画布上-----------------------------------------------------------------------------
	canvasPart = canvas(Rect(w, 0, w, h));                                        //获得画布的另一部分  
	resize(rgbRectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
	Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
		cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
	//rectangle(canvasPart, vroiR, Scalar(0, 0, 255), 3, 8);
	cout << "Painted ImageR" << endl;
	*/

//--显示结果-------------------------------------------------------------------------------------
	namedWindow("disparity", WINDOW_NORMAL);


//--鼠标响应函数setMouseCallback(窗口名称, 鼠标回调函数, 传给回调函数的参数，一般取0)------------
	setMouseCallback("disparity", onMouse, 0);

	stereo_match_sgbm(0, 0);   //--【需要调整参数的位置5】，本行调用sgbm算法，下一行调用BM算法，二选一进行距离测量。
	//stereo_match_bm(0, 0);

	waitKey(0);
	return 0;
}