#include "input/GAConfiguration.h"

void GAOptions::setPopulationSize(int newPopulationSize) 
{
    m_populationSize = newPopulationSize;
}

void GAOptions::setMinMaxIndividual(std::pair<Individual,Individual> newMinMaxIndividual) 
{
    m_minMaxIndividual = newMinMaxIndividual;
}

void GAOptions::setMutationRate(double newMutationRate) 
{
    m_mutationRate = newMutationRate;
}

void GAOptions::setTournamentSize(int newTournamentSize) 
{
    m_tournamentSize = newTournamentSize;
}

void GAOptions::setCrossoverRate(double newCrossoverRate) 
{
    m_crossoverRate = newCrossoverRate;
}

void GAOptions::setCrossoverRadius(double newCrossoverRadius) 
{
    m_crossoverRadius = newCrossoverRadius;
}

void GAOptions::setElitismAmount(int newElitismAmount) 
{
    m_elitismAmount = newElitismAmount;
}

void GAOptions::setOutputPath(std::string newOutputPath) 
{
    m_outputPath = newOutputPath;
}

void GAOptions::setLogFitness(bool newLogFitness) 
{
    m_logFitness = newLogFitness;
}

std::string std::to_string(std::pair<Individual,Individual> &val)
{
    std::ostringstream oss;
    Individual ind1 = val.first;
    Individual ind2 = val.second;
    oss << ind1.vals[0] << "," << ind1.vals[1] << "," << ind1.vals[2] << ",";
    oss << ind2.vals[0] << "," << ind2.vals[1] << "," << ind2.vals[2];
    return oss.str();
}
