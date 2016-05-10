/**
This code comes from this paper:

Hu, Ruizhen, et al. "Interaction Context (ICON): Towards a Geometric Functionality Descriptor." ACM Transactions on Graphics 34.
*/

#pragma once

#include "../Object.h"
#include "../types.h"
#include <iostream>
#include <fstream>
#include <string>

class write
{
public:
	static void writeObj(const Object &obj, const std::string &filepath);
    static void writeSamples(const Object &obj, const std::string &filepath);
    static void writePointsToFile(std::vector<Point3d>& points, const std::string &fp);
private:
    static void writeSamplesToFile(std::vector<SamplePoint> &samples, const std::string &fp);
};
