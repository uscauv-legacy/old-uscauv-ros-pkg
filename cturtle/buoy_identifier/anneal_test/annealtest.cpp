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
	return x * sin(x);
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
	//virtual double temp()
	//{
	//	double ret = t;
	//	t = t - ((double)1 / (double)iterations);
	//	return ret;
	//}
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
		tempx = randDouble() * 50;
		tempval = testFxn(tempx);
		if (tempval > maxval)
		{
			maxx = tempx;
			maxval = tempval;
		}
	}
	cout << "Brute force result: x = " << maxx << " y = " << maxval << endl;
}

int main(int argc, char ** argv)
{
	if (argc != 2)
	{
		cout << "Usage: " << argv[0] << " num_of_iterations" << endl;
		return 0;
	}
	int iterations = atoi(argv[1]);
	MyAnneal a;
	AnnealVar v(0, 50, 3);
	a.addVar(v);
	MyAnneal b;
	MyAnneal sqrt;
	MyAnneal inv;
	b = a;
	sqrt = a;
	inv = a;
	AnnealResult res;
	AnnealResult negRes;
	AnnealResult sqrtRes;
	AnnealResult invRes;
	cout << "BEGIN LINEAR" << endl;
	res = a.simulate(iterations, Anneal::LINEAR);
	cout << "BEGIN NEG_PARAB" << endl;
	negRes = b.simulate(iterations, Anneal::NEG_PARAB);
	cout << "BEGIN SQRT" << endl;
	sqrtRes = sqrt.simulate(iterations, Anneal::SQRT);
	//cout << "BEGIN INVERSE" << endl;
	//invRes = inv.simulate(iterations, Anneal::INVERSE);
	cout << "The linear temp:    x = " << res.results[0] << " y = " << res.score << endl;
	cout << "The parabolic temp: x = " << negRes.results[0] << " y = " << negRes.score << endl;
	cout << "The sqrt temp:      x = " << sqrtRes.results[0] << " y = " << sqrtRes.score << endl;
	//cout << "The inverse temp found max at " << invRes.results[0] << " with a score of " << invRes.score << endl;
	bruteForce(iterations);

	return 0;
}
