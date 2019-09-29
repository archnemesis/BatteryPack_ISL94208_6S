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

void ISL94208::enableTemp(bool enable)
{
    uint8_t reg = readRegister(ISL94208_REG_FEATSET);
    
    if (enable) {
        reg |= (1 << 5);
    }
    else {
        reg &= ~(1 << 5);
    }

    writeRegister(ISL94208_REG_FEATSET, reg);
}

void ISL94208::selectAnalogOutput(uint8_t cell)
{
    uint8_t reg = readRegister(ISL94208_REG_ANALOG);
    reg &= 0xF0;
    reg |= (cell & 0x0F);
    writeRegister(ISL94208_REG_ANALOG, reg);
}

void ISL94208::sleep(bool enable)
{
    uint8_t reg = readRegister(ISL94208_REG_FETCTRL);

    if (enable) {
        reg |= 0x80;
    }
    else {
        reg &= ~0x80;
    }
    
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

void ISL94208::setShortCircuitDischargeThreshold(uint8_t threshold)
{
    uint8_t reg = readRegister(ISL94208_REG_DISCHGSET);
    reg &= ~0x0C;
    reg |= threshold << 2;
    writeRegister(ISL94208_REG_DISCHGSET, reg);
}

void ISL94208::enableShortCircuitDischargeControl(bool enable)
{
    uint8_t reg = readRegister(ISL94208_REG_DISCHGSET);

    if (enable) {
        reg |= (1 << 4);
    }
    else {
        reg &= ~(1 << 4);
    }

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

uint8_t ISL94208::readWkupFlag()
{
    uint8_t reg = readRegister(ISL94208_REG_CONFIG);
    return ((reg & 0x10) == 0x10);
}

uint8_t ISL94208::readDischargeOCFlag()
{
    uint8_t reg = readRegister(ISL94208_REG_OPSTATUS);
    return ((reg & 0x02) == 0x02);
}

uint8_t ISL94208::readDischargeShortCircuitFlag()
{
    uint8_t reg = readRegister(ISL94208_REG_OPSTATUS);
    return ((reg & 0x03) == 0x03);
}

uint8_t ISL94208::readChargeOCFlag()
{
    uint8_t reg = readRegister(ISL94208_REG_OPSTATUS);
    return ((reg & 0x01) == 0x01);
}

uint8_t ISL94208::readLDFailFlag()
{
    uint8_t reg = readRegister(ISL94208_REG_OPSTATUS);
    return ((reg & 0x04) == 0x04);
}

uint8_t ISL94208::readUserFlag0()
{
    uint8_t reg = readRegister(ISL94208_REG_ANALOG);
    return ((reg & 0x40) == 0x40);
}

uint8_t ISL94208::readUserFlag1()
{
    uint8_t reg = readRegister(ISL94208_REG_ANALOG);
    return ((reg & 0x80) == 0x80);
}

uint8_t ISL94208::readUserFlag2()
{
    uint8_t reg = readRegister(ISL94208_REG_WRITEN);
    return ((reg & 0x08) == 0x08);
}

uint8_t ISL94208::readUserFlag3()
{
    uint8_t reg = readRegister(ISL94208_REG_WRITEN);
    return ((reg & 0x10) == 0x10);
}

void ISL94208::setUserFlag0(bool set)
{
    uint8_t reg = readRegister(ISL94208_REG_ANALOG);

    if (set) {
        reg |= 0x40;
    }
    else {
        reg &= ~0x40;
    }

    writeRegister(ISL94208_REG_ANALOG, reg);
}

void ISL94208::setUserFlag1(bool set)
{
    uint8_t reg = readRegister(ISL94208_REG_ANALOG);

    if (set) {
        reg |= 0x80;
    }
    else {
        reg &= ~0x80;
    }

    writeRegister(ISL94208_REG_ANALOG, reg);
}

void ISL94208::setUserFlag2(bool set)
{
    uint8_t reg = readRegister(ISL94208_REG_WRITEN);

    if (set) {
        reg |= 0x08;
    }
    else {
        reg &= ~0x08;
    }

    writeRegister(ISL94208_REG_WRITEN, reg);
}

void ISL94208::setUserFlag3(bool set)
{
    uint8_t reg = readRegister(ISL94208_REG_WRITEN);

    if (set) {
        reg |= 0x10;
    }
    else {
        reg &= ~0x10;
    }

    writeRegister(ISL94208_REG_WRITEN, reg);
}

void ISL94208::disableAllCB()
{
    uint8_t reg = readRegister(ISL94208_REG_BAL);
    reg &= ~(0x7E);
    writeRegister(ISL94208_REG_BAL, reg);
}