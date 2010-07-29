#include <opencv/cv.h>

IplImage* cvFilterHS(IplImage* src,int Hmin,int Hmax, int Smin, int Smax, int mode)
{
  IplImage* timg = cvCreateImage( cvGetSize(src), 8, 3 );
  IplImage* timg2 = cvCreateImage( cvGetSize(src), 8, 3 );
  cvCvtColor( src, timg2, CV_BGR2HSV );
  timg->widthStep=src->widthStep;
  cvResize( timg2,timg, CV_INTER_LINEAR );


  IplImage* mask;
  if(mode)
    mask = cvCloneImage(src);
  else
    mask = cvCreateImage(cvSize(src->width, src->height),IPL_DEPTH_8U, 1);

  uchar* temp;
  uchar* temp2;
  for(int i = 0; i < timg->width; i++)
  {
     for(int j = 0; j < timg->height; j++)
     {
       temp = &((uchar*)(timg->imageData + timg->widthStep*j))[i*3];
       if(mode)
	 temp2= &((uchar*)(mask->imageData + mask->widthStep*j))[i*3];
       if(temp[0] >= Hmin && temp[0] <= Hmax && temp[1] >= Smin && temp[1]<= Smax ){
	 
	 if(!mode)
	   ((uchar*)(mask->imageData + mask->widthStep*j))[i] = 255;
       }
       else{
	 
	 if(mode){
	   temp2[0] = 0;
	   temp2[1] = 0;
	   temp2[2] = 0;
	 }
	 else
	   ((uchar*)(mask->imageData + mask->widthStep*j))[i] = 0;
	 
       }
     }
  }
  
  cvReleaseImage(&timg);
  cvReleaseImage(&timg2);
  return mask;
}

void cvFlipBinaryImg(IplImage* src)
{
  for(int i = 0; i < src->width; i++)
    {
      for(int j = 0; j < src->height; j++)
	{
	  if(((uchar*)(src->imageData + src->widthStep*j))[i] == 255)	    
	    ((uchar*)(src->imageData + src->widthStep*j))[i] = 0;
	  else
	    ((uchar*)(src->imageData + src->widthStep*j))[i] = 255;
	}

    }

}

IplImage* cvFilterRGB(IplImage* src,int Rmin,int Rmax, int Gmin, int Gmax,int Bmin, int Bmax, int mode)
{

IplImage* mask;
  if(mode)
    mask = cvCreateImage(cvSize(src->width, src->height),IPL_DEPTH_8U, 3);
  else
    mask = cvCreateImage(cvSize(src->width, src->height),IPL_DEPTH_8U, 1);

  uchar* temp;
  uchar* temp2;
  for(int i = 0; i < src->width; i++)
  {
     for(int j = 0; j < src->height; j++)
     {
        temp = &((uchar*)(src->imageData + src->widthStep*j))[i*3];
        if(mode)
          temp2= &((uchar*)(mask->imageData + mask->widthStep*j))[i*3];
        if(temp[0] >= Rmin && temp[0] <= Rmax && temp[1] >= Gmin && temp[1]<= Gmax && temp[2] >= Bmin && temp[2]<= Bmax){

             if(mode){
               temp2[0] = temp[0];
               temp2[1]= temp[1];
               temp2[2] = temp[2];
             }
             else
               ((uchar*)(mask->imageData + mask->widthStep*j))[i] = 255;
        }
        else{

             if(mode){
               temp2[0] = 0;
               temp2[1] = 0;
               temp2[2] = 0;
             }
             else
               ((uchar*)(mask->imageData + mask->widthStep*j))[i] = 0;

        }
     }
  }


  return mask;
}
