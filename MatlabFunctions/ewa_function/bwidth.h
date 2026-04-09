/* * bwidth.h - beamwidth mapping from psi-space to phi-space
 * * Sophocles J. Orfanidis - 1997-2008 - www.ece.rutgers.edu/~orfanidi/ewa
 */

#ifndef BWIDTH_H
#define BWIDTH_H

/**
 * Usage: dphi = BeamUtils::bwidth(d, ph0, dpsi)
 * * @param d    = array spacing in units of lambda
 * @param ph0  = beam steering angle in degrees
 * @param dpsi = beamwidth in psi-space (in radians)
 * * @return dphi = beamwidth in degrees
 * * Notes: uses the transformation dpsi=2*pi*d*sin(ph0)*dphi, which follows 
 * from differentiating psi=2*pi*d*(cos(phi) - cos(phi0)),
 * usually, dpsi input is calculated from dpsi=b*0.886*2*pi/N
 * where b is the broadening factor depending on the window design method
 */
class BeamUtils {
public:
    static double bwidth(double d, double ph0, double dpsi);
};

#endif // BWIDTH_H