/*
 * anneal_buoy.cpp
 *
 *  Created on: Apr 23, 2011
 *      Author: gerow
 */

#include "anneal.h"
#include "highgui.h"
#include "cv.h"
#include <vector>
#include <iostream>
#include <stdio.h>

IplImage* g_img;
CvPoint* g_poly_points;
CvPoint* g_new_points;
int g_num_points;

IplImage* doPyrDown( IplImage* in, int filter = IPL_GAUSSIAN_5x5 )
{
	assert(in->width % 2 == 0 && in->height % 2 == 0);
	IplImage* out = cvCreateImage(cvSize(in->width/2, in->height/2), in->depth, in->nChannels);
	cvPyrDown(in, out);
	return (out);
}

int loadPoints(char* filename)
{
	FILE *f;
	if (!(f = fopen(filename, "r")))
		cout << "Error loading points from file";
	fseek(f, 0, SEEK_END);
	int g_num_points = ftell(f);
	fseek(f, 0, SEEK_SET);
	g_poly_points = (CvPoint *)malloc(g_num_points * sizeof(CvPoint));
	char buffer[1024];
	int i = 0;
	while (!feof(f))
	{
		fscanf(f, "%s", buffer);
		int xval = atoi(buffer);
		fscanf(f, "%s", buffer);
		int yval = atoi(buffer);
		g_poly_points[i].x = xval - 543;
		g_poly_points[i].y = yval - 507;
		++i;
	}

}

int makeTemplatePoints(int x, int y, double scale, CvPoint* target)
{
	CvPoint* new_points;
	new_points = (CvPoint*)malloc(g_num_points * sizeof(CvPoint));

	for (int i = 0; i < g_num_points; ++i)
	{
		double theta = atan((double)g_poly_points[i].y/(double)g_poly_points[i].x);
		double r = sqrt(pow(g_poly_points[i].x, 2) + pow(g_poly_points[i].y, x));
		double rprime = r * sqrt(scale);
		double xprime = rprime * cos(theta);
		double yprime = rprime * sin(theta);
		new_points[i].x = floor(xprime);
		new_points[i].y = floor(yprime);
	}

	g_new_points = new_points;
	return g_num_points;
}

int getIntersectionScore(int x, int y, double scale, IplImage* img)
{
	IplImage* template_image = cvCreateImage(cvGetSize(g_img), IPL_DEPTH_8U, 1);
	cvSetZero(template_image);
	int imageT = cvCountNonZero(img);
	CvPoint* points;
	int npoints;
	npoints = makeTemplatePoints(x, y, scale, points);
	CvScalar color;
	color.val[0] = 255;
	for (int i = 1; i < 4; ++i)
		color.val[i] = 0;
	cvFillConvexPoly(template_image, g_new_points, npoints, color);
	free(g_new_points);
	int templateT = cvCountNonZero(template_image);
	cvAnd(img, template_image, template_image);
	int intersection = cvCountNonZero(template_image);
	if (imageT >= templateT)
		return intersection / imageT;
	else
		return intersection / templateT;
}

class BuoyAnneal : public Anneal
{
public:
	BuoyAnneal() { }
	virtual double score(AnnealResult &candidate)
	{
		int x = floor(candidate.results[0]);
		int y = floor(candidate.results[1]);
		double scale = candidate.results[2];
		double score = getIntersectionScore(x, y, scale, g_img);
		candidate.score = score;
		return score;
	}
};

int main(int argc, char **argv)
{
	loadPoints("points");
	if (argc != 3)
	{
		cout << "Usage: " << argv[0] << " file_name num_of_iterations" << endl;
		return 0;
		//comment
	}
	cvNamedWindow("Image Before");
	g_img = cvLoadImage(argv[1]);
	cout << "Reading image with " << g_img->nChannels << " channels." << endl;
	IplImage* grayscale = cvCreateImage(cvGetSize(g_img), IPL_DEPTH_8U, 1);
	if (g_img->nChannels == 3)
		cvCvtColor(g_img, grayscale, CV_RGB2GRAY);
	g_img = grayscale;
	g_img = doPyrDown(g_img);
	cvShowImage("Image Before", g_img);
	cvWaitKey(0);
	cvDestroyWindow("Image Before");
	int iterations = atoi(argv[2]);
	double x, y, scale;
	x = 4;
	y = 4;
	scale = 1;
	AnnealVar xVar(0, g_img->width, 4);
	AnnealVar yVar(0, g_img->height, 4);
	AnnealVar scaleVar(0, 10, 1);
	BuoyAnneal b;
	b.addVar(xVar);
	b.addVar(yVar);
	b.addVar(scaleVar);
	AnnealResult r = b.simulate(iterations, Anneal::LINEAR);
	cout << "x = " << r.results[0] << endl;
	cout << "y = " << r.results[1] << endl;
	cout << "scale = " << r.results[2] << endl;
	IplImage* output_image;
	output_image = cvCreateImage(cvGetSize(g_img), IPL_DEPTH_8U, 1);
	cvCopy(g_img, output_image);
	CvScalar s;
	s.val[0] = 100;
	s.val[1] = 0;
	s.val[2] = 0;
	s.val[3] = 0;
	CvPoint* points;
	int npoints;
	npoints = makeTemplatePoints(floor(r.results[0]), floor(r.results[1]), floor(r.results[2]), points);
	cvPolyLine(output_image, &g_new_points, &npoints, 1, 1, s);
	cvNamedWindow("Anneal Result");
	cvShowImage("Anneal Result", output_image);
	cvWaitKey(0);

	return 0;
}
