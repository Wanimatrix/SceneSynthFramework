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

class writeOBJ
{
public:
	static void write(const Object &obj, const std::string &filepath);
};