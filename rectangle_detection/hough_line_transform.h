#ifndef HOUGH_LINE_TRANSFORM_H
#define HOUGH_LINE_TRANSFORM_H

#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h> 
#include <iostream>
#include <math.h>
#include "segment.h"
#include "intersect.h"

using namespace std;

class HoughLineTransform 
{
	private:
		typedef vector<cv::Vec2f> LinesContainer;
		typedef vector<Intersect> IntersectsContainer;
		typedef vector<Segment> SegmentsContainer;
		typedef cv::Mat Mat;
		typedef cv::Point Point;
		typedef cv::Scalar Scalar;
		
		Mat image_src_, image_dst_, image_dst_color_;
		LinesContainer lines_;
		IntersectsContainer intersects_;
		SegmentsContainer segments_;
		float rho_, theta_;
		Point point_1_, point_2_, point_A1_, point_A2_, point_B1_, point_B2_, intersect_;
		double a_, b_, x0_, y0_, theta_error_;
	
	public:
		const IntersectsContainer getIntersects() const;
		Intersect getIntersects(int) const;
		int getIntersectsSize() const;
		Mat getImageSrc() const;
		Mat getImageDst() const;
		Mat getImageDstColor() const;
		Segment * findMatchingSegment(Point, Point, SegmentsContainer &);
		int calculateDenominator(Point, Point, Point, Point) const;
		Point calculateIntersect(Point, Point, Point, Point, int) const;
		void calculateTheta(Intersect &);
		bool storeIntersects(Segment &, Segment &, Point);
		void calculateSegments(int);
		void drawDetectedLines(int);
		void calculateIntersections(int);
		void applyHoughLineTransform();
		HoughLineTransform(Mat);
		HoughLineTransform();
		~HoughLineTransform();
};

const HoughLineTransform::IntersectsContainer HoughLineTransform::getIntersects() const 
{
	return intersects_;
}

Intersect HoughLineTransform::getIntersects(int index) const 
{
	return intersects_[index];
}

int HoughLineTransform::getIntersectsSize() const
{
	return intersects_.size();
}

HoughLineTransform::Mat HoughLineTransform::getImageSrc() const
{
	return image_src_;
}

HoughLineTransform::Mat HoughLineTransform::getImageDst() const
{
	return image_dst_;
}


HoughLineTransform::Mat HoughLineTransform::getImageDstColor() const
{
	return image_dst_color_;
}

Segment * HoughLineTransform::findMatchingSegment(Point p1, Point p2, SegmentsContainer &ss)
{
	for(SegmentsContainer::iterator it = ss.begin(); it != ss.end(); ++it)
	{
		if((it->getEndpoint(1) == p1) && (it->getEndpoint(2) == p2))
		{
			return &ss[it-ss.begin()];
		}
	} 
	return NULL;
} 


int HoughLineTransform::calculateDenominator(Point pA1, Point pA2, Point pB1, Point pB2) const
{
	int d = ((pA1.x -pA2.x)*(pB1.y - pB2.y)) - 
			((pA1.y - pA2.y)*(pB1.x - pB2.x));
			
	return d;
}

HoughLineTransform::Point HoughLineTransform::calculateIntersect(Point pA1, Point pA2, Point pB1, Point pB2, int d) const
{
	Point intersect;
	intersect.x = ((((pA1.x * pA2.y) - (pA1.y * pA2.x)) * 
				  (pB1.x - pB2.x)) - ((pA1.x - pA2.x) * 
				  ((pB1.x * pB2.y) - (pB1.y * pB2.x))))
				  /d;
			
	intersect.y = ((((pA1.x * pA2.y) - (pA1.y * pA2.x)) * 
				  (pB1.y - pB2.y)) - ((pA1.y - pA2.y) * 
				  ((pB1.x * pB2.y) - (pB1.y * pB2.x))))
				  /d;
				  
	return intersect;
}

void HoughLineTransform::calculateTheta(Intersect &p)
{
	Point A = p.getLine(1).getEndpoint(1);
	Point B = p.getLine(2).getEndpoint(1);
	Point intersect = p.getIntersect();
		
	double distanceAI = sqrt(pow((A.x - intersect.x), 2.0) + 
							 pow((A.y - intersect.y), 2.0));
	double distanceBI = sqrt(pow((B.x - intersect.x), 2.0) + 
							 pow((B.y - intersect.y), 2.0));				
	double distanceAB = sqrt(pow((B.x - A.x), 2.0) + pow((B.y - A.y), 2.0));
			   
	//printf("A: %d, %d B: %d, %d I: %d, %d \n", A.x, A.y, B.x, B.y, intersect.x, intersect.y); 
	double theta = 57.2957795 * acos((pow(distanceAI, 2.0) + 
									  pow(distanceBI, 2.0) - 
									  pow(distanceAB, 2.0))/
									  (2 * distanceAI * distanceBI));
	p.setTheta(theta);
	//printf("Theta: %f \n", theta);
}
			
bool HoughLineTransform::storeIntersects(Segment &s1, Segment &s2, Point q)
{	
	Intersect intersect(q, s1, s2);
	
	bool match = false;		
	for(IntersectsContainer::iterator it = intersects_.begin(); it != intersects_.end(); ++it)
	{
		if(*it == intersect)
		{
			match = true;
			break;
		}
	}
	if(!match)
	{	
		calculateTheta(intersect);
		intersects_.push_back(intersect);
	}
	
	return match;
}	

void HoughLineTransform::calculateSegments(int i)
{
	rho_ = lines_[i][0];
	theta_ = lines_[i][1];

	a_ = cos(theta_);
	b_ = sin(theta_);
	x0_ = a_*rho_; 
	y0_ = b_*rho_;
		
	point_1_.x = cvRound(x0_ + 1000*(-b_));
	point_1_.y = cvRound(y0_ + 1000*(a_));

	point_2_.x = cvRound(x0_ - 1000*(-b_));
	point_2_.y = cvRound(y0_ - 1000*(a_));		
		
	clipLine(image_dst_color_.size(), point_1_, point_2_); 
	Segment s(point_1_, point_2_);
	segments_.push_back(s);
}

void HoughLineTransform::drawDetectedLines(int i)
{		
	line(image_dst_color_, segments_[i].getEndpoint(1), segments_[i].getEndpoint(2), Scalar(0, 0, 255), 3, CV_AA);
	circle(image_dst_color_, segments_[i].getEndpoint(1), 3, Scalar(255,0,0), -1, 8, 0);
	circle(image_dst_color_, segments_[i].getEndpoint(2), 3, Scalar(255,0,0), -1, 8, 0);
}

void HoughLineTransform::calculateIntersections(int i)
{
	point_A1_ = segments_[i].getEndpoint(1);
	point_A2_ = segments_[i].getEndpoint(2);

	for(SegmentsContainer::iterator it = segments_.begin(); it != segments_.end(); ++it)
	{
		if(i == (it - segments_.begin())) continue;
		
		point_B1_ = it->getEndpoint(1);
		point_B2_ = it->getEndpoint(2);
		
				
		int denominator = calculateDenominator(point_A1_, point_A2_, point_B1_, point_B2_); 
		if(denominator == 0) continue;
				
		intersect_ = calculateIntersect(point_A1_, point_A2_, point_B1_, point_B2_, denominator);
		if((intersect_.x >= 0) && (intersect_.x <= image_dst_color_.cols))
		{
			if((intersect_.y >= 0) && (intersect_.y <= image_dst_color_.rows))
			{
				bool match = storeIntersects(segments_[i], *it, intersect_);
				
				if(!match){
					// Draw intersection
					circle(image_dst_color_, intersects_.back().getIntersect(), 3, Scalar(0,255,0), -1, 8, 0);		
				}
			}
		}	
	}
}

void HoughLineTransform::applyHoughLineTransform(){
	// Apply Canny 
	Canny(image_src_, image_dst_, 50, 200, 3);
	
	// Convert to color
	cvtColor(image_dst_, image_dst_color_, CV_GRAY2BGR);
	
	// Apply Hough line transform (standard)
	HoughLines(image_dst_, lines_, 1, CV_PI/180, 100, 0, 0);	
	printf("Lines: %d \n", lines_.size());
	
	for(LinesContainer::iterator it = lines_.begin(); it != lines_.end(); ++it)
	{
		calculateSegments(it - lines_.begin());
		drawDetectedLines(it - lines_.begin());
	}

	for(SegmentsContainer::iterator it = segments_.begin(); it != segments_.end(); ++it)
	{
		calculateIntersections(it - segments_.begin());
	}

	for(IntersectsContainer::iterator it = intersects_.begin(); it != intersects_.end(); ++it)
	{
		it->print("");
	}
}

HoughLineTransform::HoughLineTransform(Mat src)
{
	image_src_ = src;
}

HoughLineTransform::HoughLineTransform(){}

HoughLineTransform::~HoughLineTransform(){}

#endif
