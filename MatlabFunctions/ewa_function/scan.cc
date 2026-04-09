#include "scan.h"
#include <cmath>

std::vector<std::complex<double>> ArrayUtils::scan(const std::vector<std::complex<double>>& a, 
                                                  double ps0) {
    size_t N = a.size();
    std::vector<std::complex<double>> ascan(N);
    
    const std::complex<double> j(0.0, 1.0);
    double center = (static_cast<double>(N) - 1.0) / 2.0;

    for (size_t n = 0; n < N; ++n) {
        // m = n - (N-1)/2
        double m = static_cast<double>(n) - center;
        
        // Calculate the progressive phase: exp(-j * m * ps0)
        std::complex<double> phase_factor = std::exp(-j * (m * ps0));
        
        // Apply to weights: ascan(n) = a(n) * phase_factor
        ascan[n] = a[n] * phase_factor;
    }

    return ascan;
}