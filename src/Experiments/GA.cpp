#include "GA.h"
#include "../Debug/DebugTools.h"
#include <algorithm>
#include "../Utilities/Utilities.h"


bool GA::validateNewIndividual(Individual newIndividual)
{
    return m_fitEval.validate(newIndividual);
}

void GA::createPopulation(int count, Individual min, Individual max) 
{
    m_population.reserve(count);
    m_fitnessCache.reserve(count);
    while(m_population.size() < count) {
        DebugLogger::ss << "Population: " << m_population.size() << "/" << count;
        DebugLogger::log();
        Individual ind;
        do
        {
            for(int i = 0; i < sizeof(ind.vals)/sizeof(ind.vals[0]); i++) {
                ind.vals[i] = utilities::uniformDouble(min.vals[i], max.vals[i]);
            }
        } while (!validateNewIndividual(ind));
        std::ostringstream oss;
        oss << "Chair_" << m_population.size();
        ind.name = oss.str();
        m_population.push_back(ind);
        m_fitnessCache.push_back(std::pair<int,std::pair<double,std::shared_ptr<IBS>>>(-1,std::pair<double,std::shared_ptr<IBS>>(0,std::shared_ptr<IBS>(nullptr))));
    }
    m_population.shrink_to_fit();
}

std::pair<double,std::shared_ptr<IBS>> GA::fitness(int indIndex) 
{
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

double GA::grade() 
{
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
        assert(indFit.second || indFit.first == -std::numeric_limits<double>::infinity());
        if(indFit.second)
            ibsObjs.push_back(indFit.second->ibsObj);
        popFitness += indFit.first;
        if(indFit.first > bestFitness)
        {
            bestFitness = indFit.first;
            bestFitIdx = i;
        }
    }
    std::ostringstream genNb;
    genNb << m_generation;
    m_fitEval.displayIndividualsInScene(m_population, m_options.getOutputPath()+"generationResult_"+genNb.str()+".blend", ibsObjs);
    DebugLogger::ss << "Grading done ..." << std::endl;
    DebugLogger::ss << "Best fitness was " << bestFitness;
    DebugLogger::log();
    return popFitness / m_population.size();
}

int GA::tournamentSelection()
{
    int winnerIdx = utilities::uniformInt(0,m_population.size()-1);
    double winnerFitness = GA::fitness(winnerIdx).first;
    for(int i = 1; i < m_options.getTournamentSize(); i++)
    {
        int newIdx = utilities::uniformInt(0,m_population.size()-1);
        double newFitness = GA::fitness(newIdx).first;
        if(newFitness > winnerFitness)
        {
            winnerIdx = newIdx;
            winnerFitness = newFitness;
        }
    }

    return winnerIdx;
}

/* Individual GA::crossover(Individual male, Individual female) */
/* { */
/*     Individual newIndividual; */

/*     for(int g = 0; g < sizeof(newIndividual.vals)/sizeof(*newIndividual.vals); g++) */
/*     { */
/*         if(GA::uniformDouble(0,1) <= m_options.getCrossoverRate()) */
/*             newIndividual.vals[g] = male.vals[g]; */
/*         else */
/*             newIndividual.vals[g] = female.vals[g]; */
/*     } */

/*     return newIndividual; */
/* } */

Individual GA::spatialCrossover(Individual male, Individual female)
{
    double t = utilities::uniformDouble(0,1);
    double parentDiff[] = {std::abs(male.vals[0]-female.vals[0]),std::abs(male.vals[1]-female.vals[1]),std::abs(male.vals[2]-female.vals[2])};
    double sphereCenter[] = {male.vals[0]+t*parentDiff[0],male.vals[1]+t*parentDiff[1],male.vals[2]+t*parentDiff[2]};
    double X1 = utilities::normalDouble(0,1);
    double X2 = utilities::normalDouble(0,1);
    double X3 = utilities::normalDouble(0,1);
    double radius = m_options.getCrossoverRadius();
    double distance = m_options.getCrossoverRadius() * std::pow(utilities::uniformDouble(0,1),1.0/3.0);
    double factor = distance/std::sqrt(X1*X1 + X2*X2 + X3*X3);
    Individual newIndividual;
    newIndividual.vals[0] = sphereCenter[0]+(factor*X1);
    newIndividual.vals[1] = sphereCenter[1];//+(factor*X2);
    newIndividual.vals[2] = sphereCenter[2]+(factor*X3);

    return newIndividual;
}

Individual GA::mutate(Individual ind)
{
    Individual mutated = ind;
    for(int g = 0; g < sizeof(ind.vals)/sizeof(*ind.vals); g++)
    {
        if(utilities::uniformDouble(0,1) <= m_options.getMutationRate())
        {
            double gene = utilities::uniformDouble(m_options.getMinMaxIndividual().first.vals[g],m_options.getMinMaxIndividual().second.vals[g]);
            ind.vals[g] = gene;
        }
    }
    return mutated;
}

std::vector<Individual> GA::evolve() 
{
    DebugLogger::ss << "Evolving...";
    DebugLogger::log();

    // Sort individuals on fitness
    /* std::vector<std::pair<double,Individual>> fitnessPairs; */
    /* for(int i = 0; i < m_population.size(); i++) { */
    /*     fitnessPairs.push_back(std::make_pair(fitness(i).first,m_population[i])); */
    /* } */
    /* std::sort(fitnessPairs.begin(), fitnessPairs.end(), [](std::pair<double,Individual> a,std::pair<double,Individual> b) { */
    /*     return a.first > b.first; */
    /* }); */

    // Keep best individuals as parents for the next gen
    /* int retain_length = m_options.retain*fitnessPairs.size(); */
    /* std::vector<Individual> parents; */
    /* parents.reserve(m_population.size()); */


    /* DebugLogger::ss << "Selecting parents ..."; */
    /* DebugLogger::log(); */
    /* for(int i = 0; i < retain_length; i++) { */
    /*     parents.push_back(fitnessPairs[i].second); */
    /*     std::ostringstream oss; */
    /*     oss << "Chair_" << i; */
    /*     parents[i].name = oss.str(); */
    /* } */
    /* assert(parents.size()==retain_length); */

    // Take random some lesser individuals as parent
    /* DebugLogger::ss << "Selecting extra parents ..."; */
    /* DebugLogger::log(); */
    /* double randomSelect = m_options.random_select; */
    /* for(int i = retain_length; i < fitnessPairs.size(); i++) { */
    /*     if( randomSelect > GA::randomDouble(0,1) ) */
    /*     { */
    /*         parents.push_back(fitnessPairs[i].second); */
    /*         std::ostringstream oss; */
    /*         oss << "Chair_" << parents.size()-1; */
    /*         parents[parents.size()-1].name = oss.str(); */
    /*     } */
    /* } */

    // Mutate some of the parents
    /* DebugLogger::ss << "Mutating parents ..."; */
    /* DebugLogger::log(); */
    /* for(Individual i : parents) { */
    /*     if(m_options.mutate > GA::randomDouble(0,1)) { */
    /*         int position = GA::randomInt(0,sizeof(i.vals)/sizeof(i.vals[0])-1); */
    /*         i.vals[position] = GA::randomDouble(m_options.minMaxIndividual.first.vals[position], */
    /*             m_options.minMaxIndividual.second.vals[position]); */
    /*     } */
    /* } */

    // Crossover parent to generate next gen
    /* DebugLogger::ss << "Crossover parents ..."; */
    /* DebugLogger::log(); */
    /* int parentsSize = parents.size(); */
    /* int childrenSize = m_population.size()-parentsSize; */
    /* std::vector<Individual> children; */
    /* children.reserve(childrenSize); */
    /* while(children.size() < childrenSize) { */
    /*     std::vector<int> maleFemale = GA::randomIntVector(2,0,parentsSize-1); */
    /*     if(maleFemale[0] != maleFemale[1]) { */
    /*         Individual male = parents[maleFemale[0]]; */
    /*         Individual female = parents[maleFemale[1]]; */
    /*         int crossoverPosition = GA::randomInt(1,2); */
    /*         /1* int half = (sizeof(male.vals) / sizeof(male.vals[0])) / 2; *1/ */
    /*         Individual child; */
    /*         auto childIt = std::copy(std::begin(male.vals),std::begin(male.vals)+crossoverPosition,std::begin(child.vals)); */
    /*         std::copy(std::begin(female.vals)+crossoverPosition,std::end(female.vals),childIt); */
    /*         std::ostringstream oss; */
    /*         oss << "Chair_" << parents.size()+children.size(); */
    /*         child.name = oss.str(); */
    /*         children.push_back(child); */
    /*     } */
    /* } */

    // Append next generation to parents and return as the next gen
    /* DebugLogger::ss << "Creating next generation."; */
    /* DebugLogger::log(); */
    /* parents.insert(parents.end(),children.begin(),children.end()); */
    /* DebugLogger::ss << "******************" << std::endl; */
    /* DebugLogger::ss << "* NEW GENERATION *" << std::endl; */
    /* DebugLogger::ss << "******************" << std::endl; */
    /* DebugLogger::ss << "                  " << std::endl; */
    /* for(Individual i : parents) */
    /* { */
    /*     DebugLogger::ss << i.name << std::endl; */
    /* } */
    /* DebugLogger::log(); */
    /* return parents; */
    
    std::vector<Individual> newPopulation;

    if (m_options.getElitismAmount() > 0 && m_options.getElitismAmount() < m_population.size())
    {
        // Sort individuals on fitness
        std::vector<std::pair<double,Individual>> fitnessPairs;
        for(int i = 0; i < m_population.size(); i++) {
            fitnessPairs.push_back(std::make_pair(fitness(i).first,m_population[i]));
        }
        std::sort(fitnessPairs.begin(), fitnessPairs.end(), [](std::pair<double,Individual> a,std::pair<double,Individual> b) {
            return a.first > b.first;
        });

        for(int i = 0; i < m_options.getElitismAmount(); i++)
        {
            newPopulation.push_back(fitnessPairs[i].second);
        }
    }

    for(int i = m_options.getElitismAmount(); i < m_population.size(); i++)
    {
        Individual child;
        do
        {
            Individual male = m_population[tournamentSelection()];
            Individual female = m_population[tournamentSelection()];
            child = spatialCrossover(male,female);
        } while (!validateNewIndividual(child));
        std::ostringstream oss;
        oss << "Chair_" << i;
        child.name = oss.str();
        newPopulation.push_back(child);
    }

    for(int i = m_options.getElitismAmount(); i < m_population.size(); i++)
    {
        Individual tmpIndividual;
        do
        {
            tmpIndividual = mutate(newPopulation[i]);
        } while (!validateNewIndividual(tmpIndividual));
        newPopulation[i] = tmpIndividual;
    }
    /* DebugLogger::ss << "******************" << std::endl; */
    /* DebugLogger::ss << "* NEW GENERATION *" << std::endl; */
    /* DebugLogger::ss << "******************" << std::endl; */
    /* DebugLogger::ss << "                  " << std::endl; */
    /* for(Individual i : newPopulation) */
    /* { */
    /*     DebugLogger::ss << i.name << "= (" << i.vals[0] << ", " << i.vals[1] << ", " << i.vals[2] << ")" << std::endl; */
    /* } */
    /* DebugLogger::log(); */

    return newPopulation;
}

double GA::run() 
{
    DebugLogger::ss << "Starting genetic algorithm.";
    DebugLogger::log();
    createPopulation(m_options.getPopulationSize(), m_options.getMinMaxIndividual().first, m_options.getMinMaxIndividual().second);
    DebugLogger::ss << "Population with size " << m_population.size() << " created";
    DebugLogger::log();
    m_generation = 0;
    double popFitness = grade();
    DebugLogger::ss << "Current population fitness: " << popFitness;
    DebugLogger::log();
    while(!m_endCondition(m_generation,popFitness)) 
    {
        //DebugLogger::on();
        DebugLogger::ss << "New generation: " << m_generation;
        DebugLogger::log();
        //DebugLogger::off();
        m_population = evolve();
        m_generation++;
        popFitness = grade();
        if(m_options.getLogFitness()) 
        {
            //DebugLogger::on();
            DebugLogger::ss << "Current fitness: " << popFitness;
            DebugLogger::log();
            //DebugLogger::off();
        }
    }
    //evolve();
}
