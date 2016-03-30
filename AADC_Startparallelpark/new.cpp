#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <sstream>


using namespace cv;
using namespace std;

Point point1,point3, point2, match_old, match; /* vertical points of the bounding box */
int drag = 0;
Rect rect; /* bounding box */
Mat img, roiImg; /* roiImg - the part of the image in the bounding box */
int select_flag = 0;
bool go_fast = false;
  Mat dst, dst_norm, dst_norm_scaled,img_object,img_scene;
Mat mytemplate,img1;
bool frame_noise = false;
int count1 = 0;
bool start = true;
int current_stage = 1;
bool stagefinish = false;
bool spot = false;
bool cont = false;
int dist = 0; // accel
int total_count = 0;
bool s1 = false;
bool s2 = false;
bool s3 = false;
bool s4 = false;
bool s5 = false;
bool s6 = false;
int angle = 0;  // steer
bool accel = false;
bool foundother;
bool found1;
bool steer = false;
int total_count_old = 0;
///------- template matching -----------------------------------------------------------------------------------------------
/**
 * Code for thinning a binary image using Zhang-Suen algorithm.
 *
 * Author:  Nash (nash [at] opencv-code [dot] com) 
 * Website: http://opencv-code.com
 */
#include <opencv2/opencv.hpp>

/**
 * Perform one thinning iteration.
 * Normally you wouldn't call this function directly from your code.
 *
 * Parameters:
 * 		im    Binary image with range = [0,1]
 * 		iter  0=even, 1=odd
 */
void thinningIteration(cv::Mat& img, int iter)
{
    CV_Assert(img.channels() == 1);
    CV_Assert(img.depth() != sizeof(uchar));
    CV_Assert(img.rows > 3 && img.cols > 3);

    cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);

    int nRows = img.rows;
    int nCols = img.cols;

    if (img.isContinuous()) {
        nCols *= nRows;
        nRows = 1;
    }

    int x, y;
    uchar *pAbove;
    uchar *pCurr;
    uchar *pBelow;
    uchar *nw, *no, *ne;    // north (pAbove)
    uchar *we, *me, *ea;
    uchar *sw, *so, *se;    // south (pBelow)

    uchar *pDst;

    // initialize row pointers
    pAbove = NULL;
    pCurr  = img.ptr<uchar>(0);
    pBelow = img.ptr<uchar>(1);

    for (y = 1; y < img.rows-1; ++y) {
        // shift the rows up by one
        pAbove = pCurr;
        pCurr  = pBelow;
        pBelow = img.ptr<uchar>(y+1);

        pDst = marker.ptr<uchar>(y);

        // initialize col pointers
        no = &(pAbove[0]);
        ne = &(pAbove[1]);
        me = &(pCurr[0]);
        ea = &(pCurr[1]);
        so = &(pBelow[0]);
        se = &(pBelow[1]);

        for (x = 1; x < img.cols-1; ++x) {
            // shift col pointers left by one (scan left to right)
            nw = no;
            no = ne;
            ne = &(pAbove[x+1]);
            we = me;
            me = ea;
            ea = &(pCurr[x+1]);
            sw = so;
            so = se;
            se = &(pBelow[x+1]);

            int A  = (*no == 0 && *ne == 1) + (*ne == 0 && *ea == 1) + 
                     (*ea == 0 && *se == 1) + (*se == 0 && *so == 1) + 
                     (*so == 0 && *sw == 1) + (*sw == 0 && *we == 1) +
                     (*we == 0 && *nw == 1) + (*nw == 0 && *no == 1);
            int B  = *no + *ne + *ea + *se + *so + *sw + *we + *nw;
            int m1 = iter == 0 ? (*no * *ea * *so) : (*no * *ea * *we);
            int m2 = iter == 0 ? (*ea * *so * *we) : (*no * *so * *we);

            if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                pDst[x] = 1;
        }
    }

    img &= ~marker;
}

/**
 * Function for thinning the given binary image
 *
 * Parameters:
 * 		src  The source image, binary with range = [0,255]
 * 		dst  The destination image
 */
void thinning(const cv::Mat& src, cv::Mat& dst)
{
    dst = src.clone();
    dst /= 255;         // convert to binary image

    cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
    cv::Mat diff;

    do {
        thinningIteration(dst, 0);
        thinningIteration(dst, 1);
        cv::absdiff(dst, prev, diff);
        dst.copyTo(prev);
    } 
    while (cv::countNonZero(diff) > 0);

    dst *= 255;
}
Mat TplMatch( Mat &img, Mat &mytemplate )
{
  Mat result;

  matchTemplate( img, mytemplate, result, CV_TM_SQDIFF_NORMED );
  // cout << "match: " << result << endl;
  normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

  return result;
}


///------- Localizing the best match with minMaxLoc ------------------------------------------------------------------------

Point minmax( Mat &result )
{
  double minVal, maxVal;
  Point  minLoc, maxLoc, matchLoc;

  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
  matchLoc = minLoc;
  //cout << "match: " << minVal << endl;

  return matchLoc;
}
void stage()
{
	switch (current_stage)
	{
	case 1:
	{
		dist++;
		accel = true;
		if(dist == 10)
		{
			current_stage = 2;
			s1 = true;
			accel = false;
			dist = 0;
			return;
		}
		cout <<dist << " dist " << accel << " accel " << steer << " steer " << angle << "  steering angle " << current_stage << " stageval " << s1  << " s1" <<endl; 

		break;
	}
	
	//else if((current_stage == 2) && (s1))
	case 2:
	{
			dist++;
			accel = true;
			cout <<dist << " dist " << accel << " accel " << steer << " steer " << angle << "  steering angle " << current_stage << " stageval " << s1  << " s1" <<endl; 
			if((foundother) && (!spot))
			{
				current_stage = 1;
				s1 = false;
				dist = 0;
				foundother = false;
				//start = false;
				return;
			}
			if((dist == 20) && (spot))
			{
				current_stage = 3;
				s2 = true;
				accel = false;
			    dist = 0;
			    return;
			}
			break;
			
	}
	//else if((current_stage = 3) && (s2))
	case 3:
	{
		dist++;
		angle++;
		cout <<dist << " dist " << accel << " accel " << steer << " steer " << angle << "  steering angle " << current_stage << " stageval " << s1  << " s1" <<endl;
		accel = true;
		steer = true;
		if((dist == 5) && (angle == 5))
		{
				current_stage = 4;
				s3 = true;
				accel = false;
				steer = false;
			    dist = 0;
			    return;
		}
		break;
		
	}
	case 4:
	//else if((current_stage = 4) && (s3 ))
	{
			dist++;
			angle--;
			cout <<dist << " dist " << accel << " accel " << steer << " steer " << angle << "  steering angle " << current_stage << " stageval " << s1  << " s1" <<endl;

			accel = true;
			steer = true;
			if((dist == 5) && (angle == 0))
			{
					s4 = true;
					accel = false;
					steer = false;
					dist = 0;
					stagefinish = true;
					return;
			}
			break;
		
		}
	default:
	{
		return;
		break;	
		
	}
}
}

void action()
{
	if((found1) && (start) && (!stagefinish))
	{
		cout <<  current_stage << " cs "<< endl;
		stage();
	}
}


///------- tracking --------------------------------------------------------------------------------------------------------

void track()
{
    if (select_flag)
    {
        //roiImg.copyTo(mytemplate);
//         select_flag = false;
        go_fast = true;
    }
   // cout << count1 << start << stagefinish << " here" << endl;
    action();

//     imshow( "mytemplate", mytemplate ); waitKey(0);
    	 //std::cout << "match_old: " << match_old << endl;
         Mat result  =  TplMatch( img, mytemplate );
    match =  minmax( result ); 
	//std::cout << "match: " << match << endl;
	imshow("result",result);
    if((match.x == 0) || (match.y ==0))
    {
	frame_noise = true;
    } 
    else
	frame_noise = false;
    if(!frame_noise)
	{
   	rectangle( img, match, Point( match.x + mytemplate.cols , match.y + mytemplate.rows ), CV_RGB(255, 255, 255), 0.5 );

   	
         
    	/// latest match is the new template
    	Rect ROI = cv::Rect( match.x, match.y, mytemplate.cols, mytemplate.rows );
    	if((abs(match.x - match_old.x) > 60) || (abs(match.y - match_old.y) > 60))
		{
			total_count = total_count +1;
			count1 = count1 +1;
			//cout << (total_count_old - total_count )<< "tco" << total_count << " tc " << endl;
		    if((abs(total_count_old-total_count) == 1) && total_count == 1)
				found1 = true;
			else
				foundother = true;
				
				
		}
		
    
	cout <<"count  " <<count1 << endl;
    	//roiImg = img( ROI );
    	//roiImg.copyTo(mytemplate);
   	 imshow( "roiImg", roiImg ); //waitKey(0);
   	 match_old = match;
     total_count_old = total_count;
   	 //std::cout << "diffx " << abs(match.x - match_old.x)<< "diffy "<<abs(match.y - match_old.y) << endl;
    }
}




///------- MouseCallback function ------------------------------------------------------------------------------------------


void mouseHandler(int event, int x, int y, int flags, void *param)
{
    if (event == CV_EVENT_LBUTTONDOWN && !drag)
    {
        /// left button clicked. ROI selection begins
        point1 = Point(x, y);
        drag = 1;
    }

    if (event == CV_EVENT_MOUSEMOVE && drag)
    {
        /// mouse dragged. ROI being selected
        Mat img1 = img.clone();
        point2 = Point(x, y);
        rectangle(img1, point1, point2, CV_RGB(255, 0, 0), 3, 8, 0);
        imshow("image", img1);
    }

    if (event == CV_EVENT_LBUTTONUP && drag)
    {
        point2 = Point(x, y);
        rect = Rect(point1.x, point1.y, x - point1.x, y - point1.y);
        drag = 0;
        roiImg = img(rect);
	thinning(roiImg, roiImg);
        roiImg.copyTo(mytemplate);
//  imshow("MOUSE roiImg", roiImg); waitKey(0);
    }

    if (event == CV_EVENT_LBUTTONUP)
    {
        /// ROI selected
        select_flag = 1;
        drag = 0;
    }

}



///------- Main() ----------------------------------------------------------------------------------------------------------

int main()
{
    int k;
     int k1 = 0;
/*    
///open webcam
    VideoCapture cap(0);
    if (!cap.isOpened())
      return 1;*/

    ///open video file
    Mat temp, diff;
    VideoCapture cap;
    cap.open( "/home/venkat/Desktop/new5.mp4" );
    if ( !cap.isOpened() )
    {   cout << "Unable to open video file" << endl;    return -1;    }
/*    
    /// Set video to 320x240
     cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
     cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);*/

    cap >> img;
cap >> img1;

    cv::cvtColor(img, img, CV_BGR2GRAY);
    img = img(Rect(150,320,300,300));
    GaussianBlur( img, img, Size(7,7), 3.0 );
    cv::threshold(img, img, 0, 255, CV_THRESH_BINARY | THRESH_OTSU) ;
    cv::threshold(img, img, 150, 255, CV_THRESH_BINARY);  
    //adaptiveThreshold( img, img, 255,ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,13, 1 );
    Mat element1 = getStructuringElement( MORPH_RECT,Size(3,10),Point( 0, 0 ) );
    Mat element2 = getStructuringElement( MORPH_RECT,Size(10,3),Point( 0, 0 ) );
    //dilate( low1, dst_low, element );
    //dilate( img, img, element1 );
    //erode( img, img, element2 );
    diff = img;
    temp = img;
    //GaussianBlur( img, img, Size(7,7), 3.0 );
    //Canny(img, img, 75, 150, 3);
    imshow( "image", img );
    while (1)
    {
        //cv::cvtColor(img, img, CV_BGR2GRAY);
        cap >> img;
    img = img(Rect(150,320,300,200));
         //img = img(Rect(0,img.rows/2,img.cols/2,img.rows/2));
        cv::cvtColor(img, img, CV_BGR2GRAY);
            GaussianBlur( img, img, Size(7,7), 3.0 );
        cv::threshold(img, img, 0, 255, CV_THRESH_BINARY | THRESH_OTSU) ;  
        //adaptiveThreshold( img, img, 255,ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,13, 1 );
        //Mat element = getStructuringElement( MORPH_RECT,Size(3,10),Point( 0, 0 ) );
        //dilate( low1, dst_low, element );
    	//dilate( img, img, element1 );
    	//erode( img, img, element2 );
		thinning(img, img);
        if ( img.empty() )
            break;
        k1++;
        //cout << k1 << "" << endl;
        
     /*   if(k1 == 5){
		k1 = 0;
   	for(int x=0;x<=(img.cols -1);x++) 
    	{ 
      		//total_val[x] = 0;
      		for (int y=0;y< img.rows -1;y++)
      		{
		diff.at<uchar>(y,x) = 0;
      		diff.at<uchar>(y,x) = temp.at<uchar>(y,x)- img.at<uchar>(y,x);
      		//if (gray2.at<uchar>(y,x) < 195)
           	//gray2.at<uchar>(y,x)=0;
                   
      		}  
	}*/
    //go_fast = true;
    // Flip the frame horizontally and add blur
    //cv::flip( img, img, 1 );
    //GaussianBlur( img, img, Size(7,7), 3.0 );
    //Canny(img,img, 50, 150, 3);

        if ( rect.width == 0 && rect.height == 0 )
            cvSetMouseCallback( "image", mouseHandler, NULL );
        else
            track();


      temp = img;
	//imshow("imag", img1);
        imshow("image", img);
//  waitKey(100);   k = waitKey(75);
    k = waitKey(go_fast ? 300 : 10000);
        if (k == 27)
            break;
    }
 //}
    return 0;
}
