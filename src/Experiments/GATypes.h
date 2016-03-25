#pragma once
#include <utility>
#include <string>
#include <iostream>
#include <cmath>

// #define NB_GENES 3
// #define GENE_LENGTH 32
// #define EXP_LEN 8
// #define FRAC_LEN 32

typedef struct Individual {
    /* int chromosome[NB_GENES * GENE_LENGTH]; */
    double vals[3];
    std::string name;

    /* int binToInt(int *bin, int bits, bool sign) */
    /* { */
    /*     int result = 0; */
    /*     int binary[bits-(sign?1:0)]; */
    /*     bool isSigned = (bin[0] == 1); */
    /*     //isSigned = true; */
    /*     std::cout << "isSigned? " << isSigned << ";Bin[0] = " << bin[0] << std::endl; */
    /*     if(sign) */
    /*         for(int i = 1; i < bits; i++) binary[i-1] = ((isSigned == (bin[i] == 1)) ? 0 : 1); */
    /*     else */
    /*         for(int i = 0; i < bits; i++) binary[i] = bin[i]; */
    /*     int multiplier = 1; */
    /*     for(int j = bits-(sign?2:1); j >= 0; j--) */
    /*     { */
    /*         result += binary[j] * multiplier; */
    /*         multiplier *= 2; */
    /*     } */
    /*     return (sign && isSigned) ? -result-1 : result; */
    /* } */

    /* double binToDouble(int *bin, int expBits, int fracBits) */
    /* { */
    /*     int expBin[expBits]; */
    /*     for(int k = 0; k < expBits; k++) */
    /*     { */
    /*         expBin[k] = bin[1 + k]; */
    /*         std::cout << expBin[k] << ","; */
    /*     } */
    /*     std::cout << std::endl; */

    /*     int fracBin[fracBits]; */
    /*     for(int k = 0; k < fracBits; k++) */
    /*     { */
    /*         fracBin[k] = bin[1 + expBits + k]; */
    /*         std::cout << fracBin[k] << ","; */
    /*     } */
    /*     std::cout << std::endl; */

    /*     int sign = 1; */
    /*     if(bin[0] == 1) sign = -1; */

    /*     int expInt = binToInt(expBin, expBits, false); */
    /*     double fracDouble = 1; */
    /*     for(int k = 0; k < fracBits; k++) */ 
    /*     { */
    /*         std::cout << "POW: " << fracBin[k]*std::pow(2,-(k+1)) << std::endl; */
    /*         fracDouble += fracBin[k]*std::pow(2,-(k+1)); */
    /*     } */

    /*     std::cout << "FRAC: " << fracDouble << ", EXP: " << expInt << std::endl; */

    /*     return sign * fracDouble * std::pow(2,expInt-127); */
    /* } */

    /* double *getDoubleChromosome() */
    /* { */
    /*     double result[NB_GENES]; */
    /*     for(int i = 0; i < NB_GENES; i++) */
    /*     { */
    /*         int gene[GENE_LENGTH]; */
    /*         for(int k = 0; k < GENE_LENGTH; k++) gene[k] = chromosome[i*GENE_LENGTH + k]; */

    /*         result[i] = binToDouble(gene, EXP_LEN, FRAC_LEN); */
    /*     } */
    /*     return result; */
    /* } */
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
