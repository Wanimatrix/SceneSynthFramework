#include "GAOptions.h"
#include "Configuration.h"


#define CONFNM_POPSIZE "GA_PopulationSize"
#define CONFNM_LOGFITNESS "GA_LogFitness"
#define CONFNM_MUTATIONRATE "GA_MutationRate"
#define CONFNM_TOURNAMENTSIZE "GA_TournamentSize"
#define CONFNM_CROSSOVERRATE "GA_CrossoverRate"
#define CONFNM_CROSSOVERRADIUS "GA_CrossoverRadius"
#define CONFNM_ELITISMAMOUNT "GA_ElitismAmount"
#define CONFNM_OUTPUTPATH "GA_OutputPath"
#define CONFNM_MINMAXIND "GA_MinMaxIndividual"


/* typedef struct GAOptions { */
/*   bool logFitness; */
/*   int populationSize; */
/*   std::pair<Individual,Individual> minMaxIndividual; */
/*   double retain = 0.2; */
/*   double random_select = 0.05; */
/*   double mutate = 0.015; */
/*   int tournamentSize = 5; */
/*   double crossover = 0.5; */
/*   double crossoverRadius = 0.1; */
/*   int elitism = 0; */
/*   std::string outputPath; */
/* } GAOptions; */

GAOptions GAOptions::loadFromConfig()
{
    Configuration& config = Configuration::getInstance();
    GAOptions options;
    options.setPopulationSize(std::stoi(config.get(CONFNM_POPSIZE)));
    options.setOutputPath(config.get("ExperimentRunPath"));
    if (config.exists(CONFNM_LOGFITNESS)) 
        options.setLogFitness((config.get(CONFNM_LOGFITNESS) == "1"));
    else
        options.setLogFitness(DEFAULT_LOGFITNESS);
    if (config.exists(CONFNM_MUTATIONRATE)) 
        options.setMutationRate(stod(config.get(CONFNM_MUTATIONRATE)));
    else
        options.setMutationRate(DEFAULT_MUTATIONRATE);
    if (config.exists(CONFNM_TOURNAMENTSIZE)) 
        options.setTournamentSize(stod(config.get(CONFNM_TOURNAMENTSIZE)));
    else
        options.setTournamentSize(DEFAULT_TOURNAMENTSIZE);
    if (config.exists(CONFNM_CROSSOVERRATE)) 
        options.setCrossoverRate(stod(config.get(CONFNM_CROSSOVERRATE)));
    else
        options.setCrossoverRate(DEFAULT_CROSSOVERRATE);
    if (config.exists(CONFNM_CROSSOVERRADIUS)) 
        options.setCrossoverRadius(stod(config.get(CONFNM_CROSSOVERRADIUS)));
    else
        options.setCrossoverRadius(DEFAULT_CROSSOVERRADIUS);
    if (config.exists(CONFNM_ELITISMAMOUNT)) 
        options.setElitismAmount(stod(config.get(CONFNM_ELITISMAMOUNT)));
    else
        options.setElitismAmount(DEFAULT_ELITISM);
    return options;
}

void GAOptions::setPopulationSize(int newPopulationSize) 
{
    m_populationSize = newPopulationSize;
    saveToConfig(CONFNM_POPSIZE,std::to_string(newPopulationSize));
}

void GAOptions::setMinMaxIndividual(std::pair<Individual,Individual> newMinMaxIndividual) 
{
    m_minMaxIndividual = newMinMaxIndividual;
    saveToConfig(CONFNM_MINMAXIND,std::to_string(m_minMaxIndividual));
}

void GAOptions::setMutationRate(double newMutationRate) 
{
    m_mutationRate = newMutationRate;
    saveToConfig(CONFNM_MUTATIONRATE,std::to_string(newMutationRate));
}

void GAOptions::setTournamentSize(int newTournamentSize) 
{
    m_tournamentSize = newTournamentSize;
    saveToConfig(CONFNM_TOURNAMENTSIZE,std::to_string(newTournamentSize));
}

void GAOptions::setCrossoverRate(double newCrossoverRate) 
{
    m_crossoverRate = newCrossoverRate;
    saveToConfig(CONFNM_CROSSOVERRATE,std::to_string(newCrossoverRate));
}

void GAOptions::setCrossoverRadius(double newCrossoverRadius) 
{
    m_crossoverRadius = newCrossoverRadius;
    saveToConfig(CONFNM_CROSSOVERRADIUS,std::to_string(newCrossoverRadius));
}

void GAOptions::setElitismAmount(int newElitismAmount) 
{
    m_elitismAmount = newElitismAmount;
    saveToConfig(CONFNM_ELITISMAMOUNT,std::to_string(newElitismAmount));
}

void GAOptions::setOutputPath(std::string newOutputPath) 
{
    m_outputPath = newOutputPath;
    saveToConfig(CONFNM_OUTPUTPATH,newOutputPath);
}

void GAOptions::setLogFitness(bool newLogFitness) 
{
    m_logFitness = newLogFitness;
    saveToConfig(CONFNM_LOGFITNESS,std::to_string(newLogFitness));
}

void GAOptions::saveToConfig(std::string configName, std::string configValue)
{
    Configuration::getInstance().set(configName,configValue);
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
