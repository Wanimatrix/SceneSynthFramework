#include "GA.h"
#include "../Debug/DebugTools.h"
#include <random>
#include <algorithm>

double GA::randomDouble(double min, double max) {
    std::random_device m_rseed;
    std::mt19937 rgen(m_rseed());
    std::uniform_real_distribution<double> ddist(min,max);
    return ddist(rgen);
}

std::vector<double> GA::randomDoubleVector(int amount, double min, double max) {
    std::random_device m_rseed;
    std::vector<double> result;
    result.reserve(amount);
    std::mt19937 rgen(m_rseed());
    std::uniform_real_distribution<double> ddist(min,max);
    while(result.size() < amount) {
        result.push_back(ddist(rgen));
    }
    return result;
}

int GA::randomInt(int min, int max) {
    std::random_device m_rseed;
    std::mt19937 rgen(m_rseed());
    std::uniform_int_distribution<int> ddist(min,max);
    return ddist(rgen);
}

std::vector<int> GA::randomIntVector(int amount, int min, int max) {
    std::random_device m_rseed;
    std::vector<int> result;
    result.reserve(amount);
    std::mt19937 rgen(m_rseed());
    std::uniform_int_distribution<int> ddist(min,max);
    while(result.size() < amount) {
        result.push_back(ddist(rgen));
    }
    return result;
}

void GA::createPopulation(int count, Individual min, Individual max) {
    m_population.reserve(count);
    m_fitnessCache.reserve(count);
    while(m_population.size() < count) {
        DebugLogger::ss << "Population: " << m_population.size() << "/" << count;
        DebugLogger::log();
        Individual ind;
        for(int i = 0; i < sizeof(ind.vals)/sizeof(ind.vals[0]); i++) {
            ind.vals[i] = GA::randomDouble(min.vals[i], max.vals[i]);
        }
        std::ostringstream oss;
        oss << "Chair_" << m_population.size();
        ind.name = oss.str();
        m_population.push_back(ind);
        m_fitnessCache.push_back(std::pair<int,std::pair<double,std::shared_ptr<IBS>>>(-1,std::pair<double,std::shared_ptr<IBS>>(0,std::shared_ptr<IBS>(nullptr))));
    }
    m_population.shrink_to_fit();
}

std::pair<double,std::shared_ptr<IBS>> GA::fitness(int indIndex) {
    // TODO: When accessing for the second time during a generation get it from the saved vector
    if(m_fitnessCache[indIndex].first == m_generation)
    {
        DebugLogger::ss << "Getting fitness from cache...";
        DebugLogger::log();
        return m_fitnessCache[indIndex].second;
    }
    //if(m_fitnessGeneration == m_generation) {
        //return m_fitnessValues[indIndex];
    //}
    std::pair<double,std::shared_ptr<IBS>> result;
    result = m_fitEval.eval(m_population[indIndex]);
    m_fitnessCache[indIndex] = std::pair<int,std::pair<double,std::shared_ptr<IBS>>>(m_generation,result);
    return result;
}

double GA::grade() {
    DebugLogger::ss << "Grading started ...";
    DebugLogger::log();
    std::vector<std::shared_ptr<Object>> ibsObjs;
    std::pair<double,std::shared_ptr<IBS>> indFit = fitness(0);
    if(indFit.second)
        ibsObjs.push_back(indFit.second->ibsObj);
    double popFitness = indFit.first;
    double bestFitness = popFitness;
    double bestFitIdx = 0;
    for(int i = 1; i < m_population.size(); i++) {
        std::pair<double,std::shared_ptr<IBS>> indFit = fitness(i);
        assert(indFit.second || indFit.first == std::numeric_limits<double>::infinity());
        if(indFit.second)
            ibsObjs.push_back(indFit.second->ibsObj);
        popFitness += indFit.first;
        if(indFit.first < bestFitness)
        {
            bestFitness = indFit.first;
            bestFitIdx = i;
        }
    }
    std::ostringstream genNb;
    genNb << m_generation;
    m_fitEval.displayIndividualsInScene(m_population, m_options.outputPath+"generationResult_"+genNb.str()+".blend", ibsObjs);
    DebugLogger::ss << "Grading done ..." << std::endl;
    DebugLogger::ss << "Best fitness was " << bestFitness;
    DebugLogger::log();
    return popFitness / m_population.size();
}

std::vector<Individual> GA::evolve() {
    // Sort individuals on fitness
    std::vector<std::pair<double,Individual>> fitnessPairs;
    for(int i = 0; i < m_population.size(); i++) {
        DebugLogger::ss << "Evolving...";
        DebugLogger::log();
        fitnessPairs.push_back(std::make_pair(fitness(i).first,m_population[i]));
    }
    std::sort(fitnessPairs.begin(), fitnessPairs.end(), [](std::pair<double,Individual> a,std::pair<double,Individual> b) {
        return a.first < b.first;
    });

    // Keep best individuals as parents for the next gen
    int retain_length = m_options.retain*fitnessPairs.size();
    std::vector<Individual> parents;
    parents.reserve(m_population.size());


    DebugLogger::ss << "Selecting parents ...";
    DebugLogger::log();
    for(int i = 0; i < retain_length; i++) {
        parents.push_back(fitnessPairs[i].second);
        std::ostringstream oss;
        oss << "Chair_" << i;
        parents[i].name = oss.str();
    }
    //std::tranform(  fitnessPairs.begin(),
                    //fitnessPairs.begin()+retain_length,
                    //parents.end(),
                    //[](std::pair<double,Individual> a) {return a.second;});
    assert(parents.size()==retain_length);

    // Take random some lesser individuals as parent
    DebugLogger::ss << "Selecting extra parents ...";
    DebugLogger::log();
    double randomSelect = m_options.random_select;
    for(int i = retain_length; i < fitnessPairs.size(); i++) {
        if( randomSelect > GA::randomDouble(0,1) )
            parents.push_back(fitnessPairs[i].second);
    }
    //std::copy_if(   fitnessPairs.begin()+retain_length,
                    //fitnessPairs.end(),
                    //std::back_inserter(parents),
                    //[randomSelect](std::pair<double,Individual> a) {
                        //return randomSelect > GA::randomDouble(0,1);
                    //});

    // Mutate some of the parents
    DebugLogger::ss << "Mutating parents ...";
    DebugLogger::log();
    for(Individual i : parents) {
        if(m_options.mutate > GA::randomDouble(0,1)) {
            int position = GA::randomInt(0,sizeof(i.vals)/sizeof(i.vals[0])-1);
            i.vals[position] = GA::randomDouble(m_options.minMaxIndividual.first.vals[position],
                m_options.minMaxIndividual.second.vals[position]);
        }
    }

    // Crossover parent to generate next gen
    DebugLogger::ss << "Crossover parents ...";
    DebugLogger::log();
    int parentsSize = parents.size();
    int childrenSize = m_population.size()-parentsSize;
    std::vector<Individual> children;
    children.reserve(childrenSize);
    while(children.size() < childrenSize) {
        std::vector<int> maleFemale = GA::randomIntVector(2,0,parentsSize-1);
        if(maleFemale[0] != maleFemale[1]) {
            DebugLogger::ss << "Different parents" << std::endl;
            DebugLogger::ss << "Parent 1: " << "(" << parents[maleFemale[0]].vals[0] << 
                                               "," << parents[maleFemale[0]].vals[1] <<
                                               "," << parents[maleFemale[0]].vals[2] << "); name = " << parents[maleFemale[0]].name << std::endl;
            DebugLogger::ss << "Parent 2: " << "(" << parents[maleFemale[1]].vals[0] << 
                                               "," << parents[maleFemale[1]].vals[1] <<
                                               "," << parents[maleFemale[1]].vals[2] << "); name = " << parents[maleFemale[1]].name << std::endl;
            DebugLogger::log();
            Individual male = parents[maleFemale[0]];
            Individual female = parents[maleFemale[1]];
            int half = (sizeof(male.vals) / sizeof(male.vals[0])) / 2;
            DebugLogger::ss << "Child being calculated ...";
            DebugLogger::log();
            Individual child;
            auto childIt = std::copy(std::begin(male.vals),std::begin(male.vals)+half,std::begin(child.vals));
            std::copy(std::begin(female.vals)+half,std::end(female.vals)+half,childIt);
            std::ostringstream oss;
            oss << "Chair_" << parents.size()+children.size();
            child.name = oss.str();
            children.push_back(child);
        }
    }

    // Append next generation to parents and return as the next gen
    DebugLogger::ss << "Creating next generation.";
    DebugLogger::log();
    parents.insert(parents.end(),children.begin(),children.end());
    return parents;
}

double GA::run() {
    DebugLogger::ss << "Starting genetic algorithm.";
    DebugLogger::log();
    createPopulation(m_options.populationSize, m_options.minMaxIndividual.first, m_options.minMaxIndividual.second);
    DebugLogger::ss << "Population with size " << m_population.size() << " created";
    DebugLogger::log();
    m_generation = 0;
    double popFitness = grade();
    DebugLogger::ss << "Current population fitness: " << popFitness;
    DebugLogger::log();
    while(!m_endCondition(m_generation,popFitness)) 
    {
        m_generation++;
        //DebugLogger::on();
        DebugLogger::ss << "New generation: " << m_generation;
        DebugLogger::log();
        //DebugLogger::off();
        m_population = evolve();
        popFitness = grade();
        if(m_options.logFitness) 
        {
            //DebugLogger::on();
            DebugLogger::ss << "Current fitness: " << popFitness;
            DebugLogger::log();
            //DebugLogger::off();
        }
    }
    //evolve();
}
