#pragma once

#include "GAOptions.h"
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
    virtual int tournamentSelection();
    virtual std::vector<Individual> evolve();
    virtual Individual mutate(Individual ind);
    /* virtual Individual crossover(Individual male, Individual female); */
    virtual Individual spatialCrossover(Individual male, Individual female);
    static double uniformDouble(double min, double max);
    static double normalDouble(double mean, double variance);
    static std::vector<double> uniformDoubleVector(int amount, double min, double max);
    static int uniformInt(int min, int max);
    static std::vector<int> uniformIntVector(int amount, int min, int max);

    GAOptions m_options;
    std::vector<Individual> m_population;
    int m_generation;
    //std::vector<double> m_fitnessValues;
    std::vector<std::pair<int,std::pair<double,std::shared_ptr<IBS>>>> m_fitnessCache;
    int m_fitnessGeneration;
    EndCondition m_endCondition;
    IBSFitEval m_fitEval;
};
