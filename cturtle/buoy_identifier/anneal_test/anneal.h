#ifndef _ANNEAL_H
#define _ANNEAL_H

#include <vector>
#include <stdlib.h>
#include <iostream>

using namespace std;

/**
 * A class to pass to the Anneal method in order to define its
 * variables.
 */
class AnnealVar
{
public:
	double min;
	double max;
	double value;

	AnnealVar(double min, double max, double value=0);
	void setValue(double value);
};

/**
 * The return value of the Anneal::simulate() method
 */
class AnnealResult
{
public:
	std::vector<double> results;
	double score;
	bool operator==(AnnealResult &other);
	bool operator>(AnnealResult &other);
	bool operator<(AnnealResult &other);
};

/**
 * Anneal some shit.  (Find global minima/maxima/whateverthehellyouwant)
 */
class Anneal
{
public:
	std::vector<AnnealVar> vars;
	virtual double score(AnnealResult &candidate) = 0;
	enum tempTypes {LINEAR, NEG_PARAB, SQRT, INVERSE};
	virtual double temp(int type=Anneal::LINEAR);
	bool randBin();
	double randDouble();
	//double best_score;
	AnnealResult best_result;
	AnnealResult current_result;
	int iterations;
	int currentIteration;
public:
	AnnealResult simulate(int iterations, int tempCurve=Anneal::LINEAR);
	void addVar(AnnealVar &var);
};

AnnealVar::AnnealVar(double min, double max, double value) : min(min), max(max), value(value) { }
bool AnnealResult::operator==(AnnealResult &other)
{
	return this->score == other.score;
}

bool AnnealResult::operator<(AnnealResult &other)
{
	return this->score < other.score;
}

bool AnnealResult::operator>(AnnealResult &other)
{
	return this->score > other.score;
}

void AnnealVar::setValue(double value)
{
	this->value = value;
}

bool Anneal::randBin()
{
	return rand() % 2;
}

double Anneal::randDouble()
{
	return (double)rand() / (double)RAND_MAX;
}

double Anneal::temp(int tempCurve)
{
	double temp;
	switch (tempCurve)
	{
		case Anneal::LINEAR:
			temp = 1 -  ((double)this->currentIteration)/((double)this->iterations);
			break;
		case Anneal::NEG_PARAB:
			temp = 1 - ((double)pow(this->currentIteration, 2) / ((double)pow(this->iterations, 2)));
			break;
		case Anneal::SQRT:
			temp =  1 - sqrt((double)this->currentIteration)/sqrt((double)this->iterations);
			break;
		case Anneal::INVERSE:
			temp = 1 - ((double)1 / (((double)this->currentIteration * (double)this->iterations) + 1));
			break;
	}
	cout << "Temp is " << temp << endl;
	return temp;
}

AnnealResult Anneal::simulate(int iterations, int tempCurve)
{
	this->iterations = iterations;
	AnnealResult candidate;
	double temperature;
	this->currentIteration = 0;
	for (unsigned int i = 0; i < this->vars.size(); ++i)
	{
		this->best_result.results.push_back(this->vars[i].value);
	}
	this->current_result = best_result;
	this->score(this->best_result);
	candidate = this->current_result = this->best_result;
	temperature = this->temp(tempCurve);
	while (temperature > 0)
	{
		for (unsigned int i = 0; i < this->vars.size(); ++i)
		{
			double newVar;
			do
			{
				bool positive = this->randBin();
				double range = this->vars[i].max - this->vars[i].min;
				//range
				double mod = range * this->randDouble() * temperature;
				if (positive)
				{
					newVar = current_result.results[i] + mod;
				}
				else
				{
					newVar = current_result.results[i] - mod;
				}
				if (newVar > vars[i].min && newVar < vars[i].max)
				{
					cout << newVar << " is in the range " << vars[i].min << " and " << vars[i].max << endl;
				}
			} while (newVar < vars[i].min || newVar > vars[i].max);
			candidate.results[i] = newVar;
		}
		this->score(candidate);
		if (candidate > this->best_result)
			best_result = candidate;
		if (candidate > this->current_result)
			current_result = candidate;
		else if (this->randDouble() < temperature)
			current_result = candidate;
		++this->currentIteration;
		temperature = this->temp(tempCurve);
	}
	return best_result;
}

void Anneal::addVar(AnnealVar &var)
{
	this->vars.push_back(var);
}

#endif
