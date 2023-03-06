/**
 * @file thinpot.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "thinpot.h"
Thinpot::Thinpot() : bus_(&Wire1), address_(0x20){}
Thinpot::Thinpot(const uint8_t add, TwoWire& s, const uint8_t id, const int v)
    : bus_(&s), address_(add), ver_(v) 
{
    bus_->begin();
    bus_->endTransmission();
    bus_->beginTransmission(address_);
    bus_->write(Configuration); // Access to Cinfiguration Register
    // bus_->write(kSequenceMode_ | kAlertEnable_ | kPolarityHigh_); //
    bus_->write(kSequenceMode_);
    bus_->endTransmission();
    id_ = id;
    if (ver_ == 1) normal_vec_ = &normal_vec1_;
    else normal_vec_ = &normal_vec2_;
}

void Thinpot::restart() {
    bus_->end();
    delay(1);
    bus_->begin();
}

void Thinpot::writeReg16(const uint8_t reg, const uint16_t value) {
    bus_->beginTransmission(address_);
    bus_->write(reg);
    bus_->write((value & 0xFF00) >> 8);
    bus_->write(value & 0x00FF);
    bus_->endTransmission();
}

bool Thinpot::readReg16(const uint8_t reg, uint16_t* value) {
    bus_->beginTransmission(address_);
    bus_->write(reg);
    bus_->endTransmission(false);
    bus_->requestFrom(address_, uint8_t(2));
    if (bus_->available()) {
        *value = uint16_t(bus_->read()) << 8;
        *value |= uint16_t(bus_->read());
        return true;
    }
    return false;
}

bool Thinpot::readReg8(const uint8_t reg, uint8_t *value){
    bus_->beginTransmission(address_);
    bus_->write(reg);
    bus_->endTransmission(false);
    bus_->requestFrom(address_, uint8_t(1));
    if (bus_->available()) {
        *value = bus_->read();
        return true;
    }
    else return false;
}

bool Thinpot::getConfiguration(uint8_t *value){
    return readReg8(Configuration, value);
}


void Thinpot::setInteruptLow(const uint16_t value){
    for (uint8_t i =  0; i < 4; ++i) {
        writeReg16(kLow_[i], value & 0x4FFF);
    }
}

void Thinpot::getInteruptLow(uint16_t value[4]){
    for (uint8_t i = 0; i < 4; ++i) {
        readReg16(kLow_[i], value+i);
    }
}

void Thinpot::setInteruptHigh(const uint16_t value){
    for (uint8_t i =  0; i < 4; ++i) {
        writeReg16(kHigh_[i], value & 0x4FFF);
    }
}
void Thinpot::setHysteresis(const uint16_t value){
    for (uint8_t i =  0; i < 4; ++i) {
        writeReg16(kHysteresis_[i], value & 0x4FFF);
    }
}

bool Thinpot::checkAlert(uint8_t& alert) {
    bus_->beginTransmission(address_);
    bus_->write(AlertStatus);
    bus_->endTransmission();

    bus_->requestFrom(address_, uint8_t(1)); // 1 byte
    if (bus_->available()) {
        alert = bus_->read();
        return true;
    }
    else return false;
}

bool Thinpot::readADC(uint16_t dist[4]){
    bus_->beginTransmission(address_);
    bus_->write(ConversionResult); // mode2
    bus_->write(kSequenceMode_); // sequence
    delayMicroseconds(50);
    error_ = bus_->endTransmission(false); // 読み取りの時にはfalseにする. trueにするとsequenceModeにならない
    if (error_) {
        restart();
        return false;
    }
    uint8_t num = 0;
    num = bus_->requestFrom(address_, (uint8_t)8, false);
    uint8_t data[8];
    uint8_t i = 0;
    while (bus_->available() && i < 8) {
        for (;i < num; ++i) data[i] = bus_->read();
        if (num < 8) num += bus_->requestFrom(address_, (uint8_t)8 - num, false);
    }

    if (num == 8) {
        for (uint8_t j = 0; j < 4; ++j) {
            dist[j] = (static_cast<uint16_t>(data[2*j]) << 8);
            dist[j] |= data[2*j + 1];
            dist[j] &= 0x0FFF;
        }
    }
    else return false;
    // uint8_t ch, alart;
    // uint16_t read_val;
    // read_val = static_cast<uint16_t>(bus_->read()) << 8;
    // read_val |= bus_->read();
    // dist[0] = () & 0x0FFF;
    // ch = (read_val & 0x3000) >> 12;
    // alart = (read_val & 0x8000) >> 15;
    delayMicroseconds(50);
    error_ = bus_->endTransmission(true);
    return true;
}

void Thinpot::VolToLength(const uint16_t voltage[4], uint16_t length[4]){
    for (uint8_t i = 0; i < 4; ++i) {
        for (uint8_t j = 1; j < 11; ++j) {
            if (voltage[i] < mm_vol_[j]) {
                length[i] = 5.0f / (mm_vol_[j] - mm_vol_[j-1]) * (voltage[i] - mm_vol_[j-1]) + 5.0f * (j - 1);
                break;
            }
            if (j == 10) length[i] = 4095;
        }
    }
}

bool Thinpot::checkCollision() {
    uint16_t voltage[4], len[4];
    readADC(voltage);
    VolToLength(voltage, len);
    last_time_millis_ = millis();
    
    collision_ = false;
    for (uint8_t i = 0; i < 4; ++i) {
        if (voltage[i] > 4000) {
            get_data_[i] = false; 
            continue;
        }
        else {
            get_data_[i] = true;
            length_[i] = len[i] + offset_; // 
            collision_ = true;
        }
    }
    return collision_;
}