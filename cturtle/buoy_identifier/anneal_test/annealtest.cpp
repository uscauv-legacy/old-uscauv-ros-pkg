/*
 * annealtest.cpp
 *
 *  Created on: Apr 13, 2011
 *      Author: mike
 */

#include <math.h>
#include <iostream>
#include "anneal.h"

using namespace std;

class MyAnneal : public Anneal
{
private:
	double t;
public:
	MyAnneal()
	{
		t = 1;
	}
	virtual double score()
	{
		double x = this->current_result.results[0];
		cout << "Using x = " << x << endl;
		cout << "Score: " << x * sin((3 * x)) << endl;
		return x * sin(3 * x);
	}
	virtual double temp()
	{
		double ret = t;
		t = t - 0.001;
		return ret;
	}
};

int main(int argc, char ** argv)
{
	MyAnneal a;
	AnnealVar v(0, 10, 3);
	a.addVar(v);
	AnnealResult res;
	res = a.simulate();
	cout << "The maximum is at " << res.results[0] << " with a score of " << res.score << endl;

	return 0;
}
