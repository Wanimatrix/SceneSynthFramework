/* #pragma once */
/* #include <utility> */
/* #include <string> */
/* #include <iostream> */
/* #include <cmath> */

/* #define DEFAULT_MUTATE 0.015 */
/* #define DEFAULT_TOURNAMENTSIZE 5 */
/* #define DEFAULT_CROSSOVER 0.5 */
/* #define DEFAULT_CROSSOVERRADIUS 0.1 */
/* #define DEFAULT_ELITISM 0 */
/* #define DEFAULT_LOGFITNESS */
/* // #define NB_GENES 3 */
/* // #define GENE_LENGTH 32 */
/* // #define EXP_LEN 8 */
/* // #define FRAC_LEN 32 */

/* typedef struct Individual { */
/*     /1* int chromosome[NB_GENES * GENE_LENGTH]; *1/ */
/*     double vals[3]; */
/*     std::string name; */
/*     double similarity; */

/*     /1* int binToInt(int *bin, int bits, bool sign) *1/ */
/*     /1* { *1/ */
/*     /1*     int result = 0; *1/ */
/*     /1*     int binary[bits-(sign?1:0)]; *1/ */
/*     /1*     bool isSigned = (bin[0] == 1); *1/ */
/*     /1*     //isSigned = true; *1/ */
/*     /1*     std::cout << "isSigned? " << isSigned << ";Bin[0] = " << bin[0] << std::endl; *1/ */
/*     /1*     if(sign) *1/ */
/*     /1*         for(int i = 1; i < bits; i++) binary[i-1] = ((isSigned == (bin[i] == 1)) ? 0 : 1); *1/ */
/*     /1*     else *1/ */
/*     /1*         for(int i = 0; i < bits; i++) binary[i] = bin[i]; *1/ */
/*     /1*     int multiplier = 1; *1/ */
/*     /1*     for(int j = bits-(sign?2:1); j >= 0; j--) *1/ */
/*     /1*     { *1/ */
/*     /1*         result += binary[j] * multiplier; *1/ */
/*     /1*         multiplier *= 2; *1/ */
/*     /1*     } *1/ */
/*     /1*     return (sign && isSigned) ? -result-1 : result; *1/ */
/*     /1* } *1/ */

/*     /1* double binToDouble(int *bin, int expBits, int fracBits) *1/ */
/*     /1* { *1/ */
/*     /1*     int expBin[expBits]; *1/ */
/*     /1*     for(int k = 0; k < expBits; k++) *1/ */
/*     /1*     { *1/ */
/*     /1*         expBin[k] = bin[1 + k]; *1/ */
/*     /1*         std::cout << expBin[k] << ","; *1/ */
/*     /1*     } *1/ */
/*     /1*     std::cout << std::endl; *1/ */

/*     /1*     int fracBin[fracBits]; *1/ */
/*     /1*     for(int k = 0; k < fracBits; k++) *1/ */
/*     /1*     { *1/ */
/*     /1*         fracBin[k] = bin[1 + expBits + k]; *1/ */
/*     /1*         std::cout << fracBin[k] << ","; *1/ */
/*     /1*     } *1/ */
/*     /1*     std::cout << std::endl; *1/ */

/*     /1*     int sign = 1; *1/ */
/*     /1*     if(bin[0] == 1) sign = -1; *1/ */

/*     /1*     int expInt = binToInt(expBin, expBits, false); *1/ */
/*     /1*     double fracDouble = 1; *1/ */
/*     /1*     for(int k = 0; k < fracBits; k++) *1/ */ 
/*     /1*     { *1/ */
/*     /1*         std::cout << "POW: " << fracBin[k]*std::pow(2,-(k+1)) << std::endl; *1/ */
/*     /1*         fracDouble += fracBin[k]*std::pow(2,-(k+1)); *1/ */
/*     /1*     } *1/ */

/*     /1*     std::cout << "FRAC: " << fracDouble << ", EXP: " << expInt << std::endl; *1/ */

/*     /1*     return sign * fracDouble * std::pow(2,expInt-127); *1/ */
/*     /1* } *1/ */

/*     /1* double *getDoubleChromosome() *1/ */
/*     /1* { *1/ */
/*     /1*     double result[NB_GENES]; *1/ */
/*     /1*     for(int i = 0; i < NB_GENES; i++) *1/ */
/*     /1*     { *1/ */
/*     /1*         int gene[GENE_LENGTH]; *1/ */
/*     /1*         for(int k = 0; k < GENE_LENGTH; k++) gene[k] = chromosome[i*GENE_LENGTH + k]; *1/ */

/*     /1*         result[i] = binToDouble(gene, EXP_LEN, FRAC_LEN); *1/ */
/*     /1*     } *1/ */
/*     /1*     return result; *1/ */
/*     /1* } *1/ */
/* } Individual; */

/* typedef bool (*EndCondition)(int,double); */

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

/* // TODO: Add else clauses to set default options in configurations. */
/* GAOptions& ExpGAIBS::loadOptions() */
/* { */
/*     Configuration& config = Configuration::getInstance(); */
/*     GAOptions options; */
/*     options.populationSize = stoi(config.get("GA_PopulationSize")); */
/*     options.outputPath = config.get("GA_ExperimentRunPath"); */
/*     if (config.exists("GA_LogFitness")) */ 
/*         options.logFitness = config.get("GA_LogFitness"); */
/*     else */
/*         options.logFitness = DEFAULT_LOGFITNESS */
/*     /1* if (config.exists("GA_PopulationSize")) *1/ */ 
/*     /1* if (config.exists("GA_MinMaxIndividual")) *1/ */ 
/*     /1* { *1/ */
/*     /1*     std::string minMaxIndStr = config.get("GA_MinMaxIndividual"); *1/ */
/*     /1*     std::pair<Individual,Individual> minMax; *1/ */
/*     /1*     std::vector<std::string> splitted; *1/ */
/*     /1*     boost::split(splitted, minMaxIndStr, boost::is_any_of(",")); *1/ */
/*     /1*     assert(splitted.size() == 2*(sizeof(minMax.first.vals)/sizeof(double))); *1/ */
/*     /1*     for(int i = 0; i < splitted.size();i++) *1/ */
/*     /1*     { *1/ */
/*     /1*         if(i < splitted.size()/2) *1/ */
/*     /1*             minMax.first.vals[i] = stod(splitted[i]); *1/ */
/*     /1*         else *1/ */
/*     /1*             minMax.second.vals[i-(splitted.size()/2)] = stod(splitted[i-(splitted.size()/2)]); *1/ */
/*     /1*     } *1/ */
/*     /1*     options.minMaxIndividual = minMax; *1/ */
/*     /1* } *1/ */
/*     if (config.exists("GA_Mutate")) */ 
/*         options.mutate = stod(config.get("GA_Mutate")); */
/*     else */
/*         options.mutate = DEFAULT_MUTATE; */
/*     if (config.exists("GA_TournamentSize")) */ 
/*         options.tournamentSize = stod(config.get("GA_TournamentSize")); */
/*     else */
/*         options.tournamentSize = DEFAULT_TOURNAMENTSIZE; */
/*     if (config.exists("GA_Crossover")) */ 
/*         options.crossover = stod(config.get("GA_Crossover")); */
/*     else */
/*         options.crossover = DEFAULT_CROSSOVER; */
/*     if (config.exists("GA_CrossoverRadius")) */ 
/*         options.crossoverRadius = stod(config.get("GA_CrossoverRadius")); */
/*     else */
/*         options.crossoverRadius = DEFAULT_CROSSOVERRADIUS; */
/*     if (config.exists("GA_Elitism")) */ 
/*         options.elitism = stod(config.get("GA_Elitism")); */
/*     else */
/*         options.elitism = DEFAULT_ELITISM; */
/* } */

/* GAOptions& addGAOption(int &option, ) */
/* { */
/* } */
