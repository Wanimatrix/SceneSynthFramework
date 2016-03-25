/* #include <iostream> */
/* #include <cmath> */

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

/* int main () */
/* { */

/*     int bin[32] = {0,1,0,0,0,0,0,1,1,1,0,0,1,0,1,1,1,0,1,1,1,0,0,0,1,0,0,1,0,0,0,1}; */
/*     double result = binToDouble(bin, 8,23); */
/*     std::cout << "00100111001010110 to decimal: " << result << std::endl; */
/* } */
