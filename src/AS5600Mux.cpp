#include "AS5600Mux.h"

// Registri AS5600 principali
static const uint8_t AS5600_REG_RAW_ANGLE_H = 0x0C;
static const uint8_t AS5600_REG_RAW_ANGLE_L = 0x0D;
static const uint8_t AS5600_REG_ANGLE_H     = 0x0E;
static const uint8_t AS5600_REG_ANGLE_L     = 0x0F;

// Costruttore
AS5600Mux::AS5600Mux(TwoWire &wire,
                     uint8_t muxAddress,
                     uint8_t encoderAddr)
: m_wire(wire),
  m_muxAddress(muxAddress),
  m_encoderAddress(encoderAddr),
  m_currentChannel(-1)
{
  for (uint8_t i = 0; i < NUM_ENCODERS; ++i) {
    m_zeroOffsetRaw[i] = 0;
  }
}

void AS5600Mux::begin(bool initWire, uint32_t clockHz)
{
  if (initWire) {
    m_wire.begin();
    m_wire.setClock(clockHz);
  }
  m_currentChannel = -1;
}

bool AS5600Mux::selectChannel(uint8_t encoderIndex)
{
  if (encoderIndex >= NUM_ENCODERS) {
    return false;
  }

  if (m_currentChannel == (int8_t)encoderIndex) {
    return true; // già attivo
  }

  uint8_t channelMask = (1 << encoderIndex);

  m_wire.beginTransmission(m_muxAddress);
  m_wire.write(channelMask);
  uint8_t err = m_wire.endTransmission();

  if (err != 0) {
    return false;
  }

  m_currentChannel = (int8_t)encoderIndex;
  return true;
}

bool AS5600Mux::i2cWriteByte(uint8_t addr, uint8_t data)
{
  m_wire.beginTransmission(addr);
  m_wire.write(data);
  uint8_t err = m_wire.endTransmission();
  return (err == 0);
}

bool AS5600Mux::readRegisters(uint8_t encoderIndex,
                              uint8_t regAddress,
                              uint8_t *buffer,
                              size_t length)
{
  if (!buffer || length == 0) {
    return false;
  }

  if (!selectChannel(encoderIndex)) {
    return false;
  }

  m_wire.beginTransmission(m_encoderAddress);
  m_wire.write(regAddress);
  uint8_t err = m_wire.endTransmission(false); // repeated start

  if (err != 0) {
    return false;
  }

  size_t readCount = m_wire.requestFrom((int)m_encoderAddress, (int)length);
  if (readCount != length) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    if (!m_wire.available()) {
      return false;
    }
    buffer[i] = m_wire.read();
  }

  return true;
}

bool AS5600Mux::writeRegisters(uint8_t encoderIndex,
                               uint8_t regAddress,
                               const uint8_t *data,
                               size_t length)
{
  if (!data || length == 0) {
    return false;
  }

  if (!selectChannel(encoderIndex)) {
    return false;
  }

  m_wire.beginTransmission(m_encoderAddress);
  m_wire.write(regAddress);
  for (size_t i = 0; i < length; ++i) {
    m_wire.write(data[i]);
  }
  uint8_t err = m_wire.endTransmission();
  return (err == 0);
}

bool AS5600Mux::readRawAngle(uint8_t encoderIndex, uint16_t &rawAngle)
{
  uint8_t buf[2];
  if (!readRegisters(encoderIndex, AS5600_REG_RAW_ANGLE_H, buf, 2)) {
    return false;
  }

  rawAngle = ((uint16_t)buf[0] << 8) | buf[1];
  rawAngle &= 0x0FFF;
  return true;
}

bool AS5600Mux::readAngle(uint8_t encoderIndex, uint16_t &angle)
{
  uint8_t buf[2];
  if (!readRegisters(encoderIndex, AS5600_REG_ANGLE_H, buf, 2)) {
    return false;
  }

  angle = ((uint16_t)buf[0] << 8) | buf[1];
  angle &= 0x0FFF;
  return true;
}

bool AS5600Mux::readAngleDegrees(uint8_t encoderIndex, float &degrees)
{
  uint16_t raw;
  if (!readRawAngle(encoderIndex, raw)) {
    return false;
  }

  degrees = (static_cast<float>(raw) * 360.0f) / 4096.0f;
  return true;
}

// --- Gestione zero ---

void AS5600Mux::setZeroOffsetRaw(uint8_t encoderIndex, uint16_t offsetRaw)
{
  if (encoderIndex >= NUM_ENCODERS) return;

  m_zeroOffsetRaw[encoderIndex] = (offsetRaw & 0x0FFF);
}

void AS5600Mux::setZeroOffsetDegrees(uint8_t encoderIndex, float offsetDeg)
{
  if (encoderIndex >= NUM_ENCODERS) return;

  // Normalizza 0..360
  while (offsetDeg < 0.0f)   offsetDeg += 360.0f;
  while (offsetDeg >= 360.0f) offsetDeg -= 360.0f;

  // 0..360 -> 0..4095
  uint16_t raw = (uint16_t)((offsetDeg * 4096.0f) / 360.0f);
  raw &= 0x0FFF;

  m_zeroOffsetRaw[encoderIndex] = raw;
}

bool AS5600Mux::captureZeroOffset(uint8_t encoderIndex)
{
  if (encoderIndex >= NUM_ENCODERS) return false;

  uint16_t raw = 0;
  if (!readRawAngle(encoderIndex, raw)) {
    return false;
  }

  m_zeroOffsetRaw[encoderIndex] = (raw & 0x0FFF);
  return true;
}

bool AS5600Mux::readAngleZeroedRaw(uint8_t encoderIndex, uint16_t &rawAngleZeroed)
{
  if (encoderIndex >= NUM_ENCODERS) return false;

  uint16_t raw;
  if (!readRawAngle(encoderIndex, raw)) {
    return false;
  }

  uint16_t offset = m_zeroOffsetRaw[encoderIndex] & 0x0FFF;

  // differenza modulare 12 bit
  uint16_t diff = (raw - offset) & 0x0FFF;

  rawAngleZeroed = diff;
  return true;
}

bool AS5600Mux::readAngleZeroedDegrees(uint8_t encoderIndex, float &degreesZeroed)
{
  uint16_t rawZ;
  if (!readAngleZeroedRaw(encoderIndex, rawZ)) {
    return false;
  }

  degreesZeroed = (static_cast<float>(rawZ) * 360.0f) / 4096.0f;
  return true;
}

// --- Nuovo helper: [0,360) → [-180,180] ---
float AS5600Mux::wrapToMinus180To180(float deg)
{
  // Normalizza prima in [0,360)
  while (deg < 0.0f)    deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;

  if (deg > 180.0f) {
    deg -= 360.0f;  // es. 270 → -90
  }
  // adesso deg è in [-180, 180]
  return deg;
}

// --- Nuovo: lettura signed [-180,180] relativa allo zero ---
bool AS5600Mux::readAngleZeroedDegreesSigned(uint8_t encoderIndex, float &degreesSigned)
{
  if (encoderIndex >= NUM_ENCODERS) {
    return false;
  }

  // 1) leggo il raw "nudo" dall'AS5600
  uint16_t raw;
  if (!readRawAngle(encoderIndex, raw)) {
    return false;
  }

  // 2) applico l'offset di zero memorizzato (zero meccanico)
  uint16_t offset = m_zeroOffsetRaw[encoderIndex] & 0x0FFF;

  // rawRel = 0 quando il giunto è in posizione "zero meccanico"
  uint16_t rawRel = (raw - offset) & 0x0FFF;  // modulo 4096

  // 3) ruoto il sistema in modo che lo zero meccanico cada a metà scala (2048)
  //    rawMid = 2048 quando il giunto è in zero meccanico
  uint16_t rawMid = (rawRel + 2048) & 0x0FFF;

  // 4) trasformo in differenza signed rispetto a metà scala
  //    rawMid ∈ [0..4095], 2048 = centro -> diff ∈ [-2048 .. +2047]
  int16_t diff = (int16_t)rawMid - 2048;

  // 5) mappo diff in gradi: 2048 tick ≈ 180°
  degreesSigned = (static_cast<float>(diff) * 180.0f) / 2048.0f;

  return true;
}

