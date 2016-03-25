#pragma once

#include "GATypes.h"
#include "IBSFitEval.h"
#include <vector>
#include <random>


class GA {
public:
    GA(GAOptions options, EndCondition endCondition, IBSFitEval fitEval) 
      : m_options(options), m_endCondition(endCondition), m_fitEval(fitEval) {};
    virtual ~GA() {};

    virtual double run();
private:
    virtual bool validateNewIndividual(Individual newIndividual);
    virtual std::pair<double,std::shared_ptr<IBS>> fitness(int indIndex);
    virtual double grade();
    virtual void createPopulation(int count, Individual min, Individual max);
    virtual std::vector<Individual> evolve();
    static double randomDouble(double min, double max);
    static std::vector<double> randomDoubleVector(int amount, double min, double max);
    static int randomInt(int min, int max);
    static std::vector<int> randomIntVector(int amount, int min, int max);

    GAOptions m_options;
    std::vector<Individual> m_population;
    int m_generation;
    //std::vector<double> m_fitnessValues;
    std::vector<std::pair<int,std::pair<double,std::shared_ptr<IBS>>>> m_fitnessCache;
    int m_fitnessGeneration;
    EndCondition m_endCondition;
    IBSFitEval m_fitEval;
};
