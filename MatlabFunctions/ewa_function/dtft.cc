#include "dtft.h"
#include <cmath>

std::vector<std::complex<double>> SignalUtils::dtft(const std::vector<double>& x, 
                                                   const std::vector<double>& w) {
    size_t L = x.size();
    size_t N = w.size();
    std::vector<std::complex<double>> X(N, 0.0);
    
    // Imaginary unit 'j'
    const std::complex<double> j(0.0, 1.0);

    for (size_t k = 0; k < N; ++k) {
        // z = exp(-j * w)
        std::complex<double> z = std::exp(-j * w[k]);
        
        std::complex<double> val(0.0, 0.0);
        
        // Horner's Rule: evaluate polynomial from back to front
        // MATLAB: for n = L-1:-1:0, X = x(n+1) + z .* X
        for (int n = static_cast<int>(L) - 1; n >= 0; --n) {
            val = x[n] + z * val;
        }
        
        X[k] = val;
    }

    return X;
}