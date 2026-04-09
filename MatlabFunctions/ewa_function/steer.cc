#include "steer.h"
#include <cmath>

std::vector<std::complex<double>> ArrayUtils::steer(double d, const std::vector<std::complex<double>>& a, double ph0) {
    // 1. Handle the broadside case to avoid floating-point cosine errors
    if (std::abs(ph0 - 90.0) < 1e-12) {
        return a;
    }

    const double PI = 3.14159265358979323846;

    // 2. Convert steering angle from degrees to radians
    double ph0_rad = ph0 * PI / 180.0;

    // 3. Calculate scanning phase: ps0 = 2 * pi * d * cos(ph0)
    double ps0 = 2.0 * PI * d * std::cos(ph0_rad);

    // 4. Reuse the scan function logic
    return ArrayUtils::scan(a, ps0);
}