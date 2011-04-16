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
protected:
	std::vector<AnnealVar> vars;
	virtual double score() = 0;
	virtual double temp() = 0;
	bool randBin();
	double randDouble();
	//double best_score;
	AnnealResult best_result;
	AnnealResult current_result;
public:
	AnnealResult simulate();
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

AnnealResult Anneal::simulate()
{
	AnnealResult candidate;
	double temperature;
	for (unsigned int i = 0; i < this->vars.size(); ++i)
	{
		this->best_result.results.push_back(this->vars[i].value);
	}
	this->current_result = best_result;
	this->best_result.score = this->score();
	candidate = this->current_result = this->best_result;
	while ((temperature = this->temp()) > 0)
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
		candidate.score = this->score();
		if (candidate > this->best_result)
			best_result = candidate;
		if (candidate > this->current_result)
			current_result = candidate;
		else if (this->randDouble() < temperature)
			current_result = candidate;
	}
	return best_result;
}

void Anneal::addVar(AnnealVar &var)
{
	this->vars.push_back(var);
}

#endif
