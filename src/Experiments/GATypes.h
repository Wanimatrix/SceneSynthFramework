#pragma once
#include <utility>
#include <string>

typedef struct Individual {
  double vals[3];
  std::string name;
} Individual;

typedef bool (*EndCondition)(int,double);
typedef struct GAOptions {
  bool logFitness;
  int populationSize;
  std::pair<Individual,Individual> minMaxIndividual;
  double retain = 0.2;
  double random_select = 0.05;
  double mutate = 0.01;
  std::string outputPath;
} GAOptions;
