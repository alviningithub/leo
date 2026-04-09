/* * bwidth.cc - implementation of beamwidth mapping
 * Sophocles J. Orfanidis - 1997-2008
 */

#include "bwidth.h"
#include <cmath>
#include <limits>

double BeamUtils::bwidth(double d, double ph0, double dpsi) {
    // MATLAB: ph0 = ph0 * pi / 180;
    const double PI = 3.14159265358979323846;
    double ph0_rad = ph0 * PI / 180.0;
    
    double dphi;
    
    // MATLAB: if abs(sin(ph0)) < eps
    // Using machine epsilon for double precision
    if (std::abs(std::sin(ph0_rad)) < std::numeric_limits<double>::epsilon()) {
        // endfire 
        dphi = std::sqrt(2.0 * dpsi / (PI * d));
    } else {
        // broadside
        dphi = dpsi / (2.0 * PI * d * std::sin(ph0_rad));
    }

    // MATLAB: dphi = dphi * 180 / pi;
    return dphi * 180.0 / PI;
}