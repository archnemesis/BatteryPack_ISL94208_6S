#include "ISL94208.h"

#include "Arduino.h"
#include "Wire.h"

ISL94208::ISL94208(uint8_t addr)
{
    _addr = addr;
}

void ISL94208::enableDFET(bool enable)
{
    uint8_t reg = readRegister(ISL94208_REG_FETCTRL);

    if (enable) {
        reg |= 0x01;
    }
    else {
        reg &= ~0x01;
    }
    
    writeRegister(ISL94208_REG_FETCTRL, reg);
}

void ISL94208::enableCFET(bool enable)
{
    uint8_t reg = readRegister(ISL94208_REG_FETCTRL);
    
    if (enable) {
        reg |= 0x02;
    }
    else {
        reg &= ~0x02;
    }

    writeRegister(ISL94208_REG_FETCTRL, reg);
}

uint8_t ISL94208::readRegister(uint8_t addr)
{
    Wire.beginTransmission(_addr);
    Wire.write(addr);
    Wire.endTransmission();

    Wire.requestFrom(_addr, (uint8_t)1);
    uint8_t reg = Wire.read();
    return reg;
}

void ISL94208::writeRegister(uint8_t addr, uint8_t value)
{
    Wire.beginTransmission(_addr);
    Wire.write(addr);
    Wire.write(value);
    Wire.endTransmission();
}

void ISL94208::enableCB(uint8_t cell, bool enable)
{
    if (cell < 1 || cell > 6) return;

    uint8_t reg = readRegister(ISL94208_REG_BAL);
    
    if (enable) {
        reg |= (1 << cell);
    }
    else {
        reg &= ~(1 << cell);
    }

    writeRegister(ISL94208_REG_BAL, reg);
}

void ISL94208::enableVMON(bool enable)
{
    uint8_t reg = readRegister(ISL94208_REG_FETCTRL);
    
    if (enable) {
        reg |= (1 << 6);
    }
    else {
        reg &= ~(1 << 6);
    }

    writeRegister(ISL94208_REG_FETCTRL, reg);
}

void ISL94208::enableFeatureSetWrites(bool enable)
{
    uint8_t reg = readRegister(ISL94208_REG_WRITEN);
    
    if (enable) {
        reg |= (1 << 7);
    }
    else {
        reg &= ~(1 << 7);
    }

    writeRegister(ISL94208_REG_WRITEN, reg);
}

void ISL94208::enableChargeSetWrites(bool enable)
{
    uint8_t reg = readRegister(ISL94208_REG_WRITEN);
    
    if (enable) {
        reg |= (1 << 6);
    }
    else {
        reg &= ~(1 << 6);
    }

    writeRegister(ISL94208_REG_WRITEN, reg);
}

void ISL94208::enableDischargeSetWrites(bool enable)
{
    uint8_t reg = readRegister(ISL94208_REG_WRITEN);
    
    if (enable) {
        reg |= (1 << 5);
    }
    else {
        reg &= ~(1 << 5);
    }

    writeRegister(ISL94208_REG_WRITEN, reg);
}

void ISL94208::selectAnalogOutput(uint8_t cell)
{
    uint8_t reg = readRegister(ISL94208_REG_ANALOG);
    reg &= 0xF0;
    reg |= (cell & 0x0F);
    writeRegister(ISL94208_REG_ANALOG, reg);
}

void ISL94208::sleep()
{
    uint8_t reg = readRegister(ISL94208_REG_FETCTRL);
    reg |= 0x80;
    writeRegister(ISL94208_REG_FETCTRL, reg);
}

uint8_t ISL94208::readCheckBit()
{
    uint8_t reg = readRegister(ISL94208_REG_CONFIG);
    reg = (reg & 0x20) >> 5;
    return reg;
}

void ISL94208::setOverCurrentChargeThreshold(uint8_t threshold)
{
    uint8_t reg = readRegister(ISL94208_REG_CHGSET);
    reg &= ~0x60;
    reg |= threshold << 5;
    writeRegister(ISL94208_REG_CHGSET, reg);
}

void ISL94208::setOverCurrentDischargeThreshold(uint8_t threshold)
{
    uint8_t reg = readRegister(ISL94208_REG_DISCHGSET);
    reg &= ~0x60;
    reg |= threshold << 5;
    writeRegister(ISL94208_REG_DISCHGSET, reg);
}

uint8_t ISL94208::readSleepFlag()
{
    uint8_t reg = readRegister(ISL94208_REG_FETCTRL);
    return ((reg & 0x80) == 0x80);
}

uint8_t ISL94208::readWkupPolarity()
{
    uint8_t reg = readRegister(ISL94208_REG_FEATSET);
    return ((reg & 0x01) == 0x01);
}