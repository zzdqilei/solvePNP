#include <iostream>  
#include "opencv2/highgui/highgui.hpp"  
#include "opencv2/imgproc/imgproc.hpp"  
#include <vector>
#include "opencv2/opencv.hpp"
//#include <contours2.cpp>
using namespace cv;  
using namespace std;  
void bwLabel(const Mat& imgBw, Mat & imgLabeled);
 void brightnessException (Mat InputImg,float& cast,float& da);
Point  centerOfGravity(Mat& input,int &x,int &y);
int  coordinate(Point First,Point Second,Point centre,Point2d Sensor_Size,int H);
int height=0,width=0;
float cast=0,da=0;
//#define DEBUG
#ifdef DEBUG
#define dbg(de)        printf(de)
#else
#define dbg()    
#endif

#define focalLength 4.75  //相机焦距的物理尺寸，单位mm
Point2d centrePoint;  
Point2d SensorSize;
int realH=420;

 int main( int argc, char** argv )  
 {  
	 centrePoint.x=316.835;
	 centrePoint.y=247.917;
	 
	 SensorSize.x=3.2;
	 SensorSize.y=2.4;
	int x=0,y=0;
    VideoCapture cap(1); //capture the video from web cam  
	width=cap.get(CV_CAP_PROP_FRAME_HEIGHT );
	height=cap.get(CV_CAP_PROP_FRAME_HEIGHT );

    if ( !cap.isOpened() )  // if not success, exit program  
    {  
         cout << "Cannot open the web cam" << endl;  
         return -1;  
    }  
  
  namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"  
  
  Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = 4.377083949726417e+02;
    cameraMatrix.at<double>(0, 1) = 0.021564701076255;
    cameraMatrix.at<double>(0, 2) = 3.260491311579127e+02;
    cameraMatrix.at<double>(1, 1) = 4.375146185726212e+02;
    cameraMatrix.at<double>(1, 2) = 2.436940467669944e+02;

    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = -0.385541673306882;
    distCoeffs.at<double>(1, 0) = 0.145556723650446;
    distCoeffs.at<double>(2, 0) = 6.057456843815020e-04;
    distCoeffs.at<double>(3, 0) = -0.001158039562780;
    distCoeffs.at<double>(4, 0) = 0;

    Mat view, rview, map1, map2;
    Size imageSize;
	Mat imgOriginal;  
    cap.read(imgOriginal); // read a new frame from video  

    imageSize = imgOriginal.size();
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
        getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
        imageSize, CV_16SC2, map1, map2);


  int exposure=6;
	//exposure = cap.get(CV_CAP_PROP_EXPOSURE);
	//cout<<exposure<<endl;
	exposure*=(-1);
  int GLowH = 60;  
  int GHighH = 71;  
  
  int RLowH = 0;  
  int RHighH = 15; 

  int iLowS = 147;   
  int iHighS = 255;  
  
  int iLowV = 200;  
  int iHighV = 255;  
  //  int iLowH = 75;  
  //int iHighH = 130;  
  //
  //int iLowS = 0;   
  //int iHighS = 255;  
  //
  //int iLowV = 0;  
  //int iHighV = 142; 

  ////Create trackbars in "Control" window  
  cvCreateTrackbar("曝光值：", "Control", &exposure, 20); //Hue (0 - 179)  

  cvCreateTrackbar("LowH", "Control", &GLowH, 179); //Hue (0 - 179)  
  cvCreateTrackbar("HighH", "Control",&GHighH, 179);  
  
  cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)  
  cvCreateTrackbar("HighS", "Control", &iHighS, 255);  
  
  cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)  
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);  

  /*try
	{   */                                     
    while (true)  //haimeidakai
    {  
		
		dbg("1");
		cap.set(CV_CAP_PROP_EXPOSURE,exposure);

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video  
		remap(imgOriginal, imgOriginal, map1, map2, INTER_LINEAR);
		flip(imgOriginal, imgOriginal, 1); // flip by x axis  
		brightnessException (imgOriginal,cast, da);
		/*if(da>=64)
		{
			exposure++;
		}else if(da<=1)
		{
			exposure--;
		}*/
         if (!bSuccess) //if not success, break loop  
        {  
             cout << "Cannot read a frame from video stream" << endl;  
             break;  
        }  
   	 dbg("2");
   Mat imgHSV;  
   vector<Mat> hsvSplit;  
   cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV  
   //hsvSplit.resize(3);
   //因为我们读取的是彩色图，直方图均衡化需要在HSV空间做  
   split(imgHSV, hsvSplit);  
 /*  imshow("hsvSplit[2]前", hsvSplit[2]);*/
   equalizeHist(hsvSplit[2],hsvSplit[2]);  //直方图均衡化，该函数能归一化图像亮度和增强对比度
    //imshow("hsvSplit[2]后", hsvSplit[2]);
   merge(hsvSplit,imgHSV);  
   	dbg("3");
   Mat imgThresholded;  
   //提取红灯的信息
   inRange(imgHSV, Scalar(RLowH, iLowS, iLowV), Scalar(RHighH, iHighS, iHighV), imgThresholded); //Threshold the image  
   //开操作 (去除一些噪点)  
   Mat element = getStructuringElement(MORPH_RECT, Size(2, 2));  
   morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);  
   //闭操作 (连接一些连通域)  
   Mat elementB = getStructuringElement(MORPH_RECT, Size(5, 5));  
   morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, elementB);  
   Mat imgBinary;
   Point redPoint;//红点
   dbg("4");
   threshold(imgThresholded, imgBinary, 128, 1, THRESH_BINARY);//  二值化
   dbg("5");
    bwLabel(imgBinary,imgBinary);
	dbg("6");
	redPoint=centerOfGravity(imgBinary,redPoint.x,redPoint.y);
	dbg("7");
	 imshow("RImage", imgThresholded); //show the thresholded image  
	
	//提取绿灯的信息
   inRange(imgHSV, Scalar(GLowH, iLowS, iLowV), Scalar(GHighH, iHighS, iHighV), imgThresholded); //Threshold the image  
   //开操作 (去除一些噪点)  
   morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);  
   //闭操作 (连接一些连通域)  
   morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, elementB);  
   Point greenPoint;//绿点
   dbg("8");
   threshold(imgThresholded, imgBinary, 128, 1, THRESH_BINARY);//  二值化
    bwLabel(imgBinary,imgBinary);
	greenPoint=centerOfGravity(imgBinary,greenPoint.x,greenPoint.y);
	
	cout<<coordinate(redPoint,greenPoint,centrePoint,SensorSize,realH)<<endl;
	
	imshow("GImage", imgThresholded); //show the thresholded image  
	
	//printf("%d,%d      %d,%d\r\n",redPoint.x,redPoint.y,greenPoint.x,greenPoint.y);
    
   imshow("Original", imgOriginal); //show the original image  

  waitKey(1);

    }  
	/*}catch(...){}*/
   return 0;  
  
}  
/////////////////////////////////////////////////////////////////
//求相机到线段的距离,注意相机平面必须与线段平面平行
//所用到的主要公式f/d=h/H
/////////////////////////////////////////////////////////////////
int  coordinate(Point First,Point Second,Point centre,Point2d Sensor_Size,int H)
{
	//先求出主光轴到两点连线的垂足的距离（实际尺寸）
	double h=0;//h为两点的距离
		//求垂足的坐标
	int  Distance_To_Centre = 0;//返回值
	Point retVal;  
    double dx = abs(First.x - Second.x);  
    double dy = abs(First.y - Second.y);  

		 double u = (centre.x - First.x)*(First.x - Second.x) +  
        (centre.y - First.y)*(First.y - Second.y);  
    u = u/((dx*dx)+(dy*dy));  
  
    retVal.x = abs(First.x + u*dx);  
    retVal.y = abs(First.y + u*dy);  
  
	double centreToretVal=0;//主轴到垂足在图像上的距离
	centreToretVal=sqrt((centre.x-retVal.x)*Sensor_Size.x/width*(centre.x-retVal.x)*Sensor_Size.x/width+\
		(centre.y-retVal.y)*Sensor_Size.y/height*(centre.y-retVal.y)*Sensor_Size.y/height);

	int d=0;//与线段所在平面的距离
	h=sqrt((First.x-Second.x)*Sensor_Size.x/width*(First.x-Second.x)*Sensor_Size.x/width+\
		(First.y-Second.y)*Sensor_Size.y/height*(First.y-Second.y)*Sensor_Size.y/height);
	d=focalLength*H/h;

	double arctan=0;//垂足与主光轴的倾斜角度
	arctan=atan((centreToretVal/focalLength));
	
	
	if(acos(arctan)!=0)
	Distance_To_Centre=d/cos(arctan);//根据倾斜角度求出实际距离


	return Distance_To_Centre;
}

//////////////////////////////////////////
//标定图像的轮廓
///////////////////////////////////////////
void bwLabel(const Mat& imgBw, Mat & imgOut)
{
    // 对图像周围扩充一格
     Mat imgClone = Mat(imgBw.rows + 1, imgBw.cols + 1, imgBw.type(), Scalar(0));
     imgBw.copyTo(imgClone(Rect(1, 1, imgBw.cols, imgBw.rows)));
	 Mat imgLabeled;
     imgLabeled.create(imgClone.size(), imgClone.type());
     imgLabeled.setTo(Scalar::all(0));
      vector<vector<Point>> contours;
     vector<Vec4i> hierarchy;
     findContours(imgClone, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
 
     vector<int> contoursLabel(contours.size(), 0);
     int numlab = 1;
     // 标记外围轮廓
     for (vector<vector<Point>>::size_type i = 0; i < contours.size(); i++)
     {
         if (hierarchy[i][3] >= 0) // 有父轮廓
         {
             continue;
         }
         for (vector<Point>::size_type k = 0; k != contours[i].size(); k++)
        {
             imgLabeled.at<uchar>(contours[i][k].y, contours[i][k].x) = numlab;
         }
        contoursLabel[i] = numlab++;
     }
     // 标记内轮廓
     for (vector<vector<Point>>::size_type i = 0; i < contours.size(); i++)    
	 {
         if (hierarchy[i][3] < 0)
         {
             continue;
         }
         for (vector<Point>::size_type k = 0; k != contours[i].size(); k++)
         {
             imgLabeled.at<uchar>(contours[i][k].y, contours[i][k].x) = contoursLabel[hierarchy[i][3]];
         }
     }
	
		 // 非轮廓像素的标记
		for (int i = 0; i < imgLabeled.rows; i++)
		 {
			 for (int j = 0; j < imgLabeled.cols; j++)
			 {
				 if (imgClone.at<uchar>(i, j) != 0 && imgLabeled.at<uchar>(i, j) == 0)
				 {
					 imgLabeled.at<uchar>(i, j) = imgLabeled.at<uchar>(i, j - 1);
				}
			}
		 }
		 imgOut = imgLabeled(Rect(1, 1, imgBw.cols, imgBw.rows)).clone(); // 将边界裁剪掉1像素
		 contours.clear();

 }

////////////////////////////////
//求最大区域的质心
////////////////////////////////
Point centerOfGravity(Mat& input,int &x,int &y)
{

      int height = input.rows; //行数
      int width = input.cols ; //列数
	  int k=0;  //模块数量
	  int maxNumOfmodual=0;  //面积最大的模块编号
	  int num[5000][2];  //第0位记录的是像素数，第二位记录的是编号
	  Point COG;//质心
	  for(int i=0;i<50;i++)
	  {
		num[i][0]=0;
		num[i][1]=i;
	  }  
	  
      if (input.isContinuous())  
      {
		 uchar* dta=(uchar*)input.data;
          //将模块的像素数放入 num数组中
		  for (int i = 0;i < height;i++)  
			{ 
				  for (int j = 0;j < width;j++)  
				 {
					 num[dta[i * width + j]][0]++;
				 } //单行处理结束                    
			}
		num[0][0]=1;
		   //找到num数组中最大的数的编号num[0](即面积最大的模块)
		    while(num[k][0]!=0)
		  { k++; }
			k-=1;
		  for(int i=k;i!=0;i--)
		  {
			 if(num[i][0]>num[i-1][0])
			 {
				 num[i-1][0]=num[i][0];
				 num[i-1][1]=num[i][1];
				 maxNumOfmodual=num[i-1][1];
			 }
              
		  }
		    //求出最大模块的质心
		  int x=0,y=0,n=0;
		  
		   for (int i = 2;i < height - 2;i++)  
			{ 
				  for (int j = 2;j < width - 2;j++)  
				 {
					 if(dta[i * width + j]==maxNumOfmodual)
					 {
						 dta[i * width + j]=50;
						x+=j;
						y+=i;
						n++;
					 }
				 }                    
			}
		   if(n!=0)
		   {
			 x/=n;
			 y/=n;
		   }
		   //printf("%d   %d\r\n",x,y);
		   //cout<<x<<","<<y<<endl;
		   
		   COG.x=x;
		   COG.y=y;
	
       }  

	  return COG;

}

 void brightnessException (Mat InputImg,float& cast,float& da) 
{ 
    Mat GRAYimg; 
    cvtColor(InputImg,GRAYimg,CV_BGR2GRAY); 
    float a=0; 
    int Hist[256]; 
    for(int i=0;i<256;i++) 
    Hist[i]=0; 
	try
	{
		  if (GRAYimg.isContinuous())  
		  {
			  uchar* dta=(uchar*)GRAYimg.data;
			   for (int i = 2;i < height - 2;i++)  
				{ 
					  for (int j = 2;j < width - 2;j++)  
						 {
				            a+=(dta[i*width+j]);
							int x=dta[i*width+j]; 
							Hist[x]++; 
						 } //单行处理结束                    
				}
		   }   
	}
	catch(...){
		printf("异常");
				}
    
    da=a/float(GRAYimg.rows*GRAYimg.cols); //带+-号的平均值
    //float D =abs(da);   //带+-号的平均值
    //float Ma=0; 
    //for(int i=0;i<256;i++) 
    //{ 
    //    Ma+=abs(i-128-da)*Hist[i]; 
    //} 
    //Ma/=float((GRAYimg.rows*GRAYimg.cols)); 
    //float M=abs(Ma); 
    //float K=D/M; 
    //cast = K;
  //printf("亮度指数： %f\n",da);
    /*if(cast>1)
    {
	   printf("亮度：");
	   if(da>0)
	   printf("过亮\n");
	   else
	   printf("过暗\n");
	 }
	else
	 printf("亮度：正常\n");*/
    return; 
}

//Point  centerOfMass(Mat& inputImage) 
//{
//	int weight=inputImage.cols;
//	int height=inputImage.rows;
//	
//}
