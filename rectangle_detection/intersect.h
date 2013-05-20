#ifndef INTERSECT_H
#define INTERSECT_H

#include "cv.h" 
#include "highgui.h"
#include <vector>
#include "segment.h"

using namespace std;

class Intersect
{
	private:
		typedef cv::Point Point;
		
		Point intersect_;
		Segment line_1_, line_2_;
		double theta_;
	
	public:		
		Point getIntersect() const;
		Segment getLine(int) const;
		double getTheta() const;
		void setTheta(double);
		void print(string) const;
		double differenceFromAngle(double) const;
		bool operator== (const Intersect &) const;
		Intersect(Point, Segment, Segment);
		Intersect();
		~Intersect();
};

Intersect::Point Intersect::getIntersect() const
{
	return intersect_;
}

Segment Intersect::getLine(int num) const
{
	if(num == 1) return line_1_;

	else return line_2_;
}

double Intersect::getTheta() const
{
	return theta_;
}

void Intersect::setTheta(double t)
{
	theta_ = t;
}

void Intersect::print(string input) const
{
	printf("%sIntersect: %d, %d ", input.c_str(), intersect_.x, intersect_.y);
	printf("Line1: P1: %d, %d, P2: %d, %d ", line_1_.getEndpoint(1).x, line_1_.getEndpoint(1).y, 
											 line_1_.getEndpoint(2).x, line_1_.getEndpoint(2).y);
	printf("Line2: P1: %d, %d, P2: %d, %d \n", line_2_.getEndpoint(1).x, line_2_.getEndpoint(1).y, 
											   line_2_.getEndpoint(2).x, line_2_.getEndpoint(2).y);
}

double Intersect::differenceFromAngle(double angle) const
{
	return abs(angle - theta_);
}

bool Intersect::operator== (const Intersect &right) const
{
    return ((intersect_ == right.getIntersect()) && 
		  	((line_1_ == right.getLine(1))  ||
		  	 (line_1_ == right.getLine(2))) && 
		  	((line_2_ == right.getLine(1))  ||
		  	 (line_2_ == right.getLine(2))));
}

Intersect::Intersect(Point p, Segment s1, Segment s2)
{
	intersect_ = p;
	line_1_ = s1;
	line_2_ = s2;
}

Intersect::Intersect(){}

Intersect::~Intersect(){}

#endif
