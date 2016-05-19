#pragma once

#include <string>
#include <sstream>
#include <utility>
#include <algorithm>

#define DEFAULT_MUTATIONRATE 0.015
#define DEFAULT_TOURNAMENTSIZE 5
#define DEFAULT_CROSSOVERRATE 0.5
#define DEFAULT_CROSSOVERRADIUS 0.1
#define DEFAULT_ELITISM 0
#define DEFAULT_LOGFITNESS false

typedef struct Individual {
    double vals[3];
    std::string name;
    double similarity;

} Individual;

typedef bool (*EndCondition)(int,double);

class GAOptions
{
public:

    //GETTERS
    int getPopulationSize() {return m_populationSize;}
    std::pair<Individual,Individual> getMinMaxIndividual() {return m_minMaxIndividual;}
    double getMutationRate() {return m_mutationRate;}
    int getTournamentSize() {return m_tournamentSize;}
    double getCrossoverRate() {return m_crossoverRate;}
    double getCrossoverRadius() {return m_crossoverRadius;}
    int getElitismAmount() {return m_elitismAmount;}
    std::string getOutputPath() {return m_outputPath;}
    bool getLogFitness() {return m_logFitness;}

    //SETTERS
    void setPopulationSize(int newPopulationSize);
    void setMinMaxIndividual(std::pair<Individual,Individual> newMinMaxIndividual);
    void setMutationRate(double newMutationRate);
    void setTournamentSize(int newTournamentSize);
    void setCrossoverRate(double newCrossoverRate);
    void setCrossoverRadius(double newCrossoverRadius);
    void setElitismAmount(int newElitismAmount);
    void setLogFitness(bool newLogFitness);
    void setOutputPath(std::string newOutputPath);


private:
    GAOptions () {}

    bool m_logFitness;
    int m_populationSize;
    std::pair<Individual,Individual> m_minMaxIndividual;
    double m_mutationRate;
    int m_tournamentSize;
    double m_crossoverRate;
    double m_crossoverRadius;
    int m_elitismAmount;
    std::string m_outputPath;
};

namespace std
{
    std::string to_string(std::pair<Individual,Individual> &val);
}
