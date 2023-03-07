/**
 * @file thinpot.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

#include <Wire.h>
#include <array>
#include <cmath>

class Thinpot {
    private:
        enum Reg {
            ConversionResult=0, 
            AlertStatus,
            Configuration,
            CycleTimer, 
            DataLowCh1, 
            DataHighCh1, 
            HysteresisCh1,
            DataLowCh2, 
            DataHighCh2, 
            HysteresisCh2,
            DataLowCh3, 
            DataHighCh3, 
            HysteresisCh3,
            DataLowCh4, 
            DataHighCh4, 
            HysteresisCh4
        };
        uint8_t kHigh_[4] = {
            DataHighCh1,
            DataHighCh2,
            DataHighCh3,
            DataHighCh4
        };
        uint8_t kLow_[4] = {
            DataLowCh1,
            DataLowCh2,
            DataLowCh3,
            DataLowCh4
        };
        uint8_t kHysteresis_[4] = {
            HysteresisCh1,
            HysteresisCh2,
            HysteresisCh3,
            HysteresisCh4
        };
        const uint8_t kSequenceMode_ =0b11110000;
        const uint8_t kFLTR_ =        0b00001000;
        const uint8_t kAlertEnable_ = 0b00000100;
        const uint8_t kBusyEnable_ =  0b00000010;
        const uint8_t kResetAlert_ =  0b00000110;
        const uint8_t kPolarityHigh_ =0b00000001;
        
        const uint8_t kCH1_ = 0x00;
        const uint8_t kCH2_ = 0x01;
        const uint8_t kCH3_ = 0x02;
        const uint8_t kCH4_ = 0x03;

    private:
        uint8_t id_ = 0;
        TwoWire* const bus_;
        const uint8_t address_;
        uint16_t mm_vol_[11] = {0, 2424, 3031, 3242, 3411, 3509, 3606, 3680, 3725, 3767, 3803};
        int ver_ = 1;

        const std::array<std::array<float, 3>, 4> tangential_vec_ = 
        std::array<std::array<float, 3>, 4>({{
            { 1.0f/std::sqrt(2.0f),  1.0f/std::sqrt(2.0f), 0.0f}, 
            { 1.0f/std::sqrt(2.0f), -1.0f/std::sqrt(2.0f), 0.0f}, 
            {-1.0f/std::sqrt(2.0f), -1.0f/std::sqrt(2.0f), 0.0f}, 
            {-1.0f/std::sqrt(2.0f),  1.0f/std::sqrt(2.0f), 0.0f}
        }});
        const std::array<std::array<float, 3>, 4> normal_vec1_ =
        std::array<std::array<float, 3>, 4>({{
            { 1.0f/std::sqrt(2.0f), -1.0f/std::sqrt(2.0f), 0.0f}, 
            {-1.0f/std::sqrt(2.0f), -1.0f/std::sqrt(2.0f), 0.0f}, 
            {-1.0f/std::sqrt(2.0f),  1.0f/std::sqrt(2.0f), 0.0f}, 
            { 1.0f/std::sqrt(2.0f),  1.0f/std::sqrt(2.0f), 0.0f}
        }});

        const std::array<std::array<float, 3>, 4> normal_vec2_ =
        std::array<std::array<float, 3>, 4>({{
            { -1.0f,  0.0f, 0.0f}, 
            {  0.0f, -1.0f, 0.0f}, 
            {  1.0f,  0.0f, 0.0f}, 
            {  0.0f,  1.0f, 0.0f}
        }});
    
    public:
        const std::array<std::array<float, 3>, 4>* normal_vec_;
        float offset_ = 13.0f;

    public:
        bool collision_ = false;
        std::array<bool, 4> get_data_{{false, false, false, false}};
        std::array<float, 4> length_; // 接触位置の脚先からの距離
        uint8_t error_ = 0;
        unsigned long last_time_millis_ = 0;

    public:
        Thinpot();
        Thinpot(const uint8_t add, TwoWire& s, const uint8_t id = -1, const int v = 1);
        void restart();
        void writeReg16(const uint8_t reg, const uint16_t value);
        bool readReg8(const uint8_t reg, uint8_t* value);
        bool readReg16(const uint8_t reg, uint16_t* value);
        bool getConfiguration(uint8_t *value);
        void setInteruptLow(const uint16_t value);
        void getInteruptLow(uint16_t value[4]);
        void setInteruptHigh(const uint16_t value);
        void setHysteresis(const uint16_t value);
        bool checkAlert(uint8_t& alert);
        bool readADC(uint16_t dist[4]);
        void VolToLength(const uint16_t voltage[4], uint16_t length[4]);
        bool checkCollision();

        inline void setOffset(const float value) {offset_ = std::abs(value);}
};