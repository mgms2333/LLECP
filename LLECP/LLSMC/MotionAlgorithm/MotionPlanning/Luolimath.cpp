#include"Luolimath.h"

double Prescription(double input,uint Degree)
{
    double dDegree = (double)Degree;
    if (input >= 0)
        return std::pow(input, 1.0 / dDegree);
    else
        return -std::pow(-input, 1.0 / dDegree);
}