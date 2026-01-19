#pragma once
#include <Arduino.h>
#include <Wire.h>

class AS5600Mux
{
  public:
    static const uint8_t NUM_ENCODERS = 6;

    AS5600Mux(TwoWire &wire = Wire,
              uint8_t muxAddress = 0x70,
              uint8_t encoderAddr = 0x36);

    void begin(bool initWire = false, uint32_t clockHz = 400000UL);

    // --- I/O generico ---
    bool readRegisters(uint8_t encoderIndex,
                       uint8_t regAddress,
                       uint8_t *buffer,
                       size_t length);

    bool writeRegisters(uint8_t encoderIndex,
                        uint8_t regAddress,
                        const uint8_t *data,
                        size_t length);

    // --- Letture base (senza offset) ---
    bool readRawAngle(uint8_t encoderIndex, uint16_t &rawAngle);
    bool readAngle(uint8_t encoderIndex, uint16_t &angle);
    bool readAngleDegrees(uint8_t encoderIndex, float &degrees);

    // --- Gestione zero per-encoder ---
    void setZeroOffsetRaw(uint8_t encoderIndex, uint16_t offsetRaw);
    void setZeroOffsetDegrees(uint8_t encoderIndex, float offsetDeg);
    bool captureZeroOffset(uint8_t encoderIndex);
    bool readAngleZeroedRaw(uint8_t encoderIndex, uint16_t &rawAngleZeroed);
    bool readAngleZeroedDegrees(uint8_t encoderIndex, float &degreesZeroed);

    // --- Nuovo: lettura in [-180°, +180°] relativa allo zero ---
    bool readAngleZeroedDegreesSigned(uint8_t encoderIndex, float &degreesSigned);

  private:
    TwoWire  &m_wire;
    uint8_t   m_muxAddress;
    uint8_t   m_encoderAddress;
    int8_t    m_currentChannel;

    uint16_t  m_zeroOffsetRaw[NUM_ENCODERS];

    bool selectChannel(uint8_t encoderIndex);
    bool i2cWriteByte(uint8_t addr, uint8_t data);

    // helper per convertire [0,360) → [-180,180]
    static float wrapToMinus180To180(float deg);
};
