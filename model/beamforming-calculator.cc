#include "beamforming-calculator.h"
#include <cmath>
#include <complex>

BeamformingCalculator::BeamformingCalculator(int antenna_number, double d, double f)
    : m_antenna_number(antenna_number), m_d(d), m_f(f), 
      m_c(3e8), m_PI(3.14159265358979323846)
{
    m_lambda = m_c / m_f;
}

double BeamformingCalculator::GetGain(double beam_direction, double rx_direction)
{
    // 轉換為弧度
    double ph0_rad = beam_direction * m_PI / 180.0;
    double rx_rad = rx_direction * m_PI / 180.0;
    
    // 產生波束權重（uniform linear array）
    std::vector<std::complex<double>> weights(m_antenna_number);
    for (int n = 0; n < m_antenna_number; ++n) {
        double phase = n * 2.0 * m_PI * m_d * std::cos(ph0_rad);
        weights[n] = std::exp(std::complex<double>(0, phase));
    }
    
    // 計算 array factor（DTFT）在接收方向
    std::complex<double> array_factor = 0.0;
    for (int n = 0; n < m_antenna_number; ++n) {
        double phase = -2.0 * m_PI * m_d * std::cos(rx_rad) * n;
        array_factor += weights[n] * std::exp(std::complex<double>(0, phase));
    }
    
    // 計算增益 |array_factor|^2
    double gain = std::norm(array_factor);
    return gain;
}