#ifndef UNIFORM_H
#define UNIFORM_H

#include <vector>
#include <complex>

struct UniformArray {
    std::vector<std::complex<double>> weights;
    double beamwidth;
};

class ArrayUtils {
public:
    /**
     * Calculates uniform array weights and beamwidth.
     * @param d    Element spacing in units of lambda
     * @param ph0  Beam angle in degrees
     * @param N    Number of array elements
     * @return     A struct containing the weights vector and dphi
     */
    static UniformArray uniform(double d, double ph0, int N);
};

#endif