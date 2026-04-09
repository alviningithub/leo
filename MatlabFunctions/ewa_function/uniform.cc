#include "uniform.h"
#include "steer.h"  // Assuming steer() is in this header
#include "bwidth.h" // Assuming bwidth() is in this header
#include <cmath>

UniformArray ArrayUtils::uniform(double d, double ph0, int N) {
    const double PI = 3.14159265358979323846;
    
    // 1. Initialize uniform weights: a = ones(1, N)
    // We use complex doubles because steer() will apply phase shifts
    std::vector<std::complex<double>> a(N, std::complex<double>(1.0, 0.0));

    // 2. Steer weights: a = steer(d, a, ph0)
    std::vector<std::complex<double>> steered_weights = ArrayUtils::steer(d, a, ph0);

    // 3. Calculate psi-space beamwidth: dps = 0.886 * 2 * pi / N
    // This constant (0.886) is the 3-dB width for a uniform (rectangular) window
    double dpsi = 0.886 * 2.0 * PI / static_cast<double>(N);

    // 4. Calculate angle-space beamwidth: dph = bwidth(d, ph0, dps)
    double dphi = BeamUtils::bwidth(d, ph0, dpsi);

    // Return both results in the struct
    return {steered_weights, dphi};
}