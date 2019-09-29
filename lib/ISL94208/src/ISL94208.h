#ifndef ISL94208_H
#define ISL94208_H

#include "Arduino.h"

#define ISL94208_REG_CONFIG     0x00
#define ISL94208_REG_OPSTATUS   0x01
#define ISL94208_REG_BAL        0x02
#define ISL94208_REG_ANALOG     0x03
#define ISL94208_REG_FETCTRL    0x04
#define ISL94208_REG_DISCHGSET  0x05
#define ISL94208_REG_CHGSET     0x06
#define ISL94208_REG_FEATSET    0x07
#define ISL94208_REG_WRITEN     0x08

#define ISL94208_OC_CHG_THRESH_100MV    0x00
#define ISL94208_OC_CHG_THRESH_120MV    0x01
#define ISL94208_OC_CHG_THRESH_140MV    0x02
#define ISL94208_OC_CHG_THRESH_160MV    0x03
#define ISL94208_SHORT_CIRCUIT_THRESH_200MV     0x00
#define ISL94208_SHORT_CIRCUIT_THRESH_350MV     0x01
#define ISL94208_SHORT_CIRCUIT_THRESH_650MV     0x02
#define ISL94208_SHORT_CIRCUIT_THRESH_1200MV    0x03

class ISL94208
{
public:
    ISL94208(uint8_t addr);
    void enableDFET(bool enable = true);
    void enableCFET(bool enable = true);
    void enableCB(uint8_t cell, bool enable = true);
    void disableAllCB();
    void enableVMON(bool enable = true);
    void enableFeatureSetWrites(bool enable = true);
    void enableChargeSetWrites(bool enable = true);
    void enableDischargeSetWrites(bool enable = true);
    void enableTemp(bool enable = true);
    void setUserFlag0(bool set = true);
    void setUserFlag1(bool set = true);
    void setUserFlag2(bool set = true);
    void setUserFlag3(bool set = true);
    void selectAnalogOutput(uint8_t cell);
    void sleep(bool enable = true);
    void setOverCurrentChargeThreshold(uint8_t threshold);
    void setOverCurrentDischargeThreshold(uint8_t threshold);
    void setShortCircuitDischargeThreshold(uint8_t threshold);
    void enableShortCircuitDischargeControl(bool enable = true);
    uint8_t readSleepFlag();
    uint8_t readWkupPolarity();
    uint8_t readWkupFlag();
    uint8_t readRegister(uint8_t addr);
    uint8_t readDischargeOCFlag();
    uint8_t readChargeOCFlag();
    uint8_t readDischargeShortCircuitFlag();
    uint8_t readLDFailFlag();
    uint8_t readUserFlag0();
    uint8_t readUserFlag1();
    uint8_t readUserFlag2();
    uint8_t readUserFlag3();
    void writeRegister(uint8_t addr, uint8_t value);
    uint8_t readCheckBit();
private:
    uint8_t _addr;
};

#endif