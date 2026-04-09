#ifndef ANALOG_BEAMFORMING_H
#define ANALOG_BEAMFORMING_H

struct Receiver {
    double direction_deg;
    double distance;
};

struct BeamResult {
    double best_angle;
    double capacity;
    double snr;
};

#endif