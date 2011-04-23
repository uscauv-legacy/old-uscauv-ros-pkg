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

double testFxn(double x)
{
	return pow(x, 3) * sin(4 * x);
}

class MyAnneal : public Anneal
{
private:
	double t;
public:
	MyAnneal()
	{
		t = 1;
	}
	virtual double score(AnnealResult &candidate)
	{
		double x = candidate.results[0];
		cout << "Using x = " << x << endl;
		candidate.score = testFxn(x);
		cout << "Score: " << candidate.score << endl;
		return candidate.score;
	}
	virtual double temp()
	{
		double ret = t;
		t = t - ((double)1 / (double)iterations);
		return ret;
	}
};

double randDouble()
{
	return (double)rand() / (double)RAND_MAX;
}

void bruteForce(int iterations)
{
	double maxval;
	double maxx;
	double tempval;
	double tempx;
	maxx = 5;
	maxval = testFxn(maxx);
	for (int i = 0; i < iterations; ++i)
	{
		tempx = randDouble() * 10;
		tempval = testFxn(tempx);
		if (tempval > maxval)
		{
			maxx = tempx;
			maxval = tempval;
		}
	}
	cout << "Result: x = " << maxx << " y = " << maxval << endl;
}

int main(int argc, char ** argv)
{
	if (argc != 2)
		cout << "Requires argument to determine number of iterations to run" << endl;
	int iterations = atoi(argv[1]);
	MyAnneal a;
	AnnealVar v(0, 10, 3);
	a.addVar(v);
	AnnealResult res;
	res = a.simulate(iterations);
	cout << "The maximum is at " << res.results[0] << " with a score of " << res.score << endl;
	bruteForce(iterations);

	return 0;
}
