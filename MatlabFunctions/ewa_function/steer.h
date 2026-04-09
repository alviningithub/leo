#ifndef STEER_H
#define STEER_H

#include <vector>
#include <complex>

class ArrayUtils {
public:
    // Existing scan function from previous step
    static std::vector<std::complex<double>> scan(const std::vector<std::complex<double>>& a, double ps0);

    /**
     * Steers array towards a given angle.
     * @param d    Element spacing in units of lambda
     * @param a    Array weights (complex vector)
     * @param ph0  Steering angle in degrees (90 is broadside)
     * @return     Steered weights
     */
    static std::vector<std::complex<double>> steer(double d, const std::vector<std::complex<double>>& a, double ph0);
};

#endif