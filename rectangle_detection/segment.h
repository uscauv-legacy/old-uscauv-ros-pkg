#ifndef SEGMENT_H
#define SEGMENT_H

#include <vector>

using namespace std;

class Segment 
{
	private:
		cv::Point endpoint_1_, endpoint_2_;
	
	public:	
		bool operator== (const Segment &);
		cv::Point getEndpoint(int) const;
		Segment(cv::Point, cv::Point);
		Segment();
		~Segment();
};

bool Segment::operator== (const Segment &right)
{
    return (((endpoint_1_ == right.getEndpoint(1))  || 
    		 (endpoint_1_ == right.getEndpoint(2))) &&
            ((endpoint_2_ == right.getEndpoint(1))  ||
           	 (endpoint_2_ == right.getEndpoint(2))));
}

cv::Point Segment::getEndpoint(int num) const
{
	if(num == 1) return endpoint_1_;
	
	else return endpoint_2_;
}

Segment::Segment(cv::Point p1, cv::Point p2)
{
	endpoint_1_ = p1;
	endpoint_2_ = p2;
}

Segment::Segment(){}

Segment::~Segment(){}

#endif
