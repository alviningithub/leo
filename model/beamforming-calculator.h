#ifndef BEAMFORMING_CALCULATOR_H
#define BEAMFORMING_CALCULATOR_H

#include <vector>
#include <complex>

class BeamformingCalculator {
public:
    /**
     * \param antenna_number 天線數量
     * \param d 天線間距（波長的倍數）
     * \param f 頻率（Hz）
     */
    BeamformingCalculator(int antenna_number, double d, double f);

    /**
     * 計算 beamforming 增益
     * \param beam_direction 波束指向方向（度）
     * \param rx_direction 接收端方向（度）
     * \return beamforming 增益（線性值）
     */
    double GetGain(double beam_direction, double rx_direction);

private:
    int m_antenna_number;
    double m_d;
    double m_f;
    double m_c;      // 光速
    double m_lambda; // 波長
    double m_PI;
};

#endif /* BEAMFORMING_CALCULATOR_H */