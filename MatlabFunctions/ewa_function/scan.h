#ifndef SCAN_H
#define SCAN_H

#include <vector>
#include <complex>

class ArrayUtils {
public:
    /**
     * Scans array weights with a given scanning phase.
     * @param a    Array weights to be scanned (vector of complex numbers)
     * @param ps0  Scanning phase in radians
     * @return     Scanned weights as a complex vector
     */
    static std::vector<std::complex<double>> scan(const std::vector<std::complex<double>>& a, 
                                                 double ps0);
};

#endif