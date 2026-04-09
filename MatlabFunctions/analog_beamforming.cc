#include <iostream>
#include <vector>
#include <cmath>
#include <complex>
#include <algorithm>
#include <random>
#include <iomanip>

#include "ewa_function/uniform.h" // Includes ArrayUtils
#include "ewa_function/dtft.h"    // Includes SignalUtils

int main() {
    // 1. Simulation Parameters
    const int antenna_number = 4;
    const int resolution = 400;
    const std::vector<double> beam_directions = {0, 30, 60, 90, 120, 150, 180};
    const double d = 0.5;
    const double f = 24e9; 
    const double c = 3e8;
    const double lambda = c / f;
    const double B = 20e6;
    const double tx_power = 18.0;          // dBm
    const double noise_dBm = -95.0;
    const double noise_mWatt = std::pow(10.0, noise_dBm / 10.0);
    const double PI = 3.14159265358979323846;

    // 2. Setup Random Receiver (rx)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dir_dist(1, 180);
    std::uniform_real_distribution<> dist_dist(10.0, 100.0);

    double rx_direction = dir_dist(gen);
    double rx_distance = dist_dist(gen);

    // 3. Pre-calculate phi and psi vectors
    std::vector<double> phi(resolution + 1);
    std::vector<double> neg_psi(resolution + 1);
    for (int i = 0; i <= resolution; ++i) {
        phi[i] = static_cast<double>(i) * PI / resolution;
        neg_psi[i] = -(2.0 * PI * d * std::cos(phi[i])); // -psi for dtft
    }

    // Find the index in phi closest to rx_direction
    double rx_rad = rx_direction * PI / 180.0;
    int closest_idx = 0;
    double min_diff = 1e9;
    for (int i = 0; i <= resolution; ++i) {
        if (std::abs(phi[i] - rx_rad) < min_diff) {
            min_diff = std::abs(phi[i] - rx_rad);
            closest_idx = i;
        }
    }

    // 4. Beam Search Loop
    double max_capacity = -1.0;
    double best_angle = 0.0;
    double best_snr = 0.0;

    for (double ph0 : beam_directions) {
        // Generate uniform weights steered to ph0
        UniformArray ua = ArrayUtils::uniform(d, ph0, antenna_number);
        
        // Calculate DTFT
        auto X = SignalUtils::dtft(ua.weights, neg_psi); 

        // Gain at the receiver's direction: g = |A|^2
        double g_rx = std::abs(X[closest_idx]);
        g_rx = g_rx * g_rx; // Square to get |z|^2

        // Path Loss & Power calculation
        double path_loss_dB = 20.0 * std::log10(lambda / (4.0 * PI * rx_distance));
        double rx_pwr_dBm = tx_power + 10.0 * std::log10(g_rx) + path_loss_dB;
        
        // SNR & Capacity
        double rx_pwr_mW = std::pow(10.0, rx_pwr_dBm / 10.0);
        double snr_linear = rx_pwr_mW / noise_mWatt;
        double snr_dB = 10.0 * std::log10(snr_linear);
        double capacity = B * std::log2(1.0 + snr_linear);

        if (capacity > max_capacity) {
            max_capacity = capacity;
            best_angle = ph0;
            best_snr = snr_dB;
        }
    }

    // 5. Output Results
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "rx_direction: " << rx_direction << std::endl;
    std::cout << "rx_distance: " << rx_distance << std::endl;
    std::cout << "------------------------------------" << std::endl;
    std::cout << "Best beam direction: " << best_angle << std::endl;
    std::cout << "Capacity: " << max_capacity << std::endl;
    std::cout << "SNR: " << best_snr << std::endl;

    return 0;
}