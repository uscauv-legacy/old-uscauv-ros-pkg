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
		typedef vector<Intersect> CornersContainer;
		typedef vector<Intersect> IntersectsContainer;
		
		Intersect intersect_;
		CornersContainer corners_;
		
	public:
		Intersect getIntersect() const;
		const CornersContainer & getCorners() const;
		Intersect getCorners(int) const;
		int getCornersSize() const;
		void addToCorners(Intersect);
		bool operator== (const SearchNode &) const;
		void printCorners() const;
		double differenceFromAngle(double) const;
		bool isRectangle(double, double) const;
		bool matchCorners(const CornersContainer &) const;
		IntersectsContainer findValidIntersects(const IntersectsContainer &) const;
		SearchNode(Intersect, CornersContainer);
		SearchNode();		
		~SearchNode();
};

Intersect SearchNode::getIntersect() const
{
	return intersect_;
}

const SearchNode::CornersContainer & SearchNode::getCorners() const
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

bool SearchNode::operator== (const SearchNode &right) const
{
    return ((intersect_ == right.getIntersect()) && 
    		(matchCorners(right.corners_)));
}

void SearchNode::printCorners() const
{
	for(CornersContainer::const_iterator it = corners_.begin(); it != corners_.end(); ++it)
		printf("Corner: %d, %d", it->getIntersect().x, it->getIntersect().y);
}

double SearchNode::differenceFromAngle(double angle) const
{
	return abs(angle - intersect_.getTheta());
}

bool SearchNode::isRectangle(double error, double angle) const
{
	if(corners_.size() == 4)
	{
		for(CornersContainer::const_iterator it = corners_.begin(); it != corners_.end(); ++it)
		{
			if(it->differenceFromAngle(angle) > error) return false;
		}
		// TODO: calculate area
		return true;
	}
	else return false;
}

bool SearchNode::matchCorners(const CornersContainer &c) const
{
	if(corners_.size() != c.size()) return false;
	
	for(CornersContainer::const_iterator it = corners_.begin(); it != corners_.end(); ++it)
	{
		if(it->getIntersect() != c[it-corners_.begin()].getIntersect())
		{
			return false;
		}
	}
	
	return true;
}

SearchNode::IntersectsContainer SearchNode::findValidIntersects(const IntersectsContainer &intersects) const
{	
	IntersectsContainer valid_intersects;
	printf("Number of intersects for current node: %d \n", intersects.size());
	for(IntersectsContainer::const_iterator it = intersects.begin(); it != intersects.end(); ++it)
	{
		if((it->getLine(1) == intersect_.getLine(1)) || (it->getLine(1) == intersect_.getLine(2)) ||
		   (it->getLine(2) == intersect_.getLine(1)) || (it->getLine(2) == intersect_.getLine(2)))
		{
			valid_intersects.push_back(*it);
			printf("Intersect added to valid_intersects.\n");
		}
	}

	return valid_intersects;
}

SearchNode::SearchNode(Intersect i , CornersContainer c)
{
	intersect_ = i;
	corners_ = c;	
}

SearchNode::SearchNode(){}

SearchNode::~SearchNode(){}

#endif
