#ifndef SEARCH_NODE_H
#define SEARCH_NODE_H

#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h> 
#include <iostream>
#include <math.h>
#include "segment.h"
#include "intersect.h"

using namespace std;

class SearchNode 
{
	private:
		Intersect intersect_;
		vector<Intersect> corners_;
		
	public:
		Intersect getIntersect() const;
		const vector<Intersect> & getCorners() const;
		Intersect getCorners(int) const;
		int getCornersSize() const;
		void addToCorners(Intersect);
		bool operator== (const SearchNode &);
		void printCorners();
		double differenceFromAngle(double);
		bool isRectangle(double, double);
		bool matchCorners(vector<Intersect>);
		vector<Intersect> findValidIntersects(vector<Intersect>);
		SearchNode(Intersect,vector<Intersect>);
		SearchNode();		
		~SearchNode();
};

Intersect SearchNode::getIntersect() const
{
	return intersect_;
}

const vector<Intersect> & SearchNode::getCorners() const
{
	return corners_;
}

Intersect SearchNode::getCorners(int index) const
{
	return corners_[index];
}

int SearchNode::getCornersSize() const
{
	return corners_.size();
}

void SearchNode::addToCorners(Intersect i)
{
	corners_.push_back(i);
}

bool SearchNode::operator== (const SearchNode &right)
{
    return ((intersect_ == right.getIntersect()) && 
    		(matchCorners(right.corners_)));
}

void SearchNode::printCorners()
{
	for(vector<Intersect>::iterator it = corners_.begin(); it != corners_.end(); ++it)
		printf("Corner: %d, %d", it->getIntersect().x, it->getIntersect().y);
}

double SearchNode::differenceFromAngle(double angle)
{
	return abs(angle - intersect_.getTheta());
}

bool SearchNode::isRectangle(double error, double angle)
{
	if(corners_.size() == 4)
	{
		for(vector<Intersect>::iterator it = corners_.begin(); it != corners_.end(); ++it)
		{
			if(it->differenceFromAngle(angle) > error) return false;
		}
		// TODO: calculate area
		return true;
	}
	else return false;
}

bool SearchNode::matchCorners(vector<Intersect> c)
{
	if(corners_.size() != c.size()) return false;
	
	for(vector<Intersect>::iterator it = corners_.begin(); it != corners_.end(); ++it)
	{
		if(it->getIntersect() != c[it-corners_.begin()].getIntersect())
		{
			return false;
		}
	}
	
	return true;
}

vector<Intersect> SearchNode::findValidIntersects(vector<Intersect> intersects)
{	
	vector<Intersect> valid_intersects;
	printf("Number of intersects for current node: %d \n", intersects.size());
	for(vector<Intersect>::iterator it = intersects.begin(); it != intersects.end(); ++it)
	{
		if((it->getLine(1) == intersect_.getLine(1)) || (it->getLine(1) == intersect_.getLine(2)) ||
		   (it->getLine(2) == intersect_.getLine(1)) || (it->getLine(2) == intersect_.getLine(2)))
		{
			valid_intersects.push_back(intersects[it - intersects.begin()]);
			printf("Intersect added to valid_intersects.\n");
		}
	}

	return valid_intersects;
}

SearchNode::SearchNode(Intersect i ,vector<Intersect> c)
{
	intersect_ = i;
	corners_ = c;	
}

SearchNode::SearchNode(){}

SearchNode::~SearchNode(){}

#endif
