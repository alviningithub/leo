#ifndef DTFT_H
#define DTFT_H

#include <vector>
#include <complex>

class SignalUtils {
public:
    /**
     * Computes the Discrete-Time Fourier Transform.
     * @param x  Input signal vector (length L)
     * @param w  Frequency vector in rads/sample (length N)
     * @return   Vector of complex DTFT values (length N)
     */
    static std::vector<std::complex<double>> dtft(const std::vector<std::complex<double>>& x, 
                                                 const std::vector<double>& w);
};

#endif // DTFT_H