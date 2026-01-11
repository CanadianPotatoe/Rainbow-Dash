// ----------------------------------------------------------------------------
// TMC429.cpp - Raspberry Pi Pico Port
//
// Original Authors:
// Peter Polidoro peter@polidoro.io
//
// Modified for Raspberry Pi Pico
// ----------------------------------------------------------------------------
#include "TMC429.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

// Near the top of TMC429.cpp, replace existing #ifndef blocks with:

#define SPI_CLOCK 1000000  // 1 MHz - adjust as needed for your application



// Updated to match your specific wiring:
#define PICO_DEFAULT_SPI_SCK_PIN 2  // SCK -> GPIO2
#define PICO_DEFAULT_SPI_TX_PIN 3   // SDI -> GPIO3 (MOSI)
#define PICO_DEFAULT_SPI_RX_PIN 4   // SDO -> GPIO4 (MISO)

// Validate that pins are defined
#if !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || !defined(PICO_DEFAULT_SPI_RX_PIN)
  #error "SPI pins must be defined. Please define PICO_DEFAULT_SPI_SCK_PIN, PICO_DEFAULT_SPI_TX_PIN, and PICO_DEFAULT_SPI_RX_PIN"
#endif

// ... rest of your code ...
void TMC429::setup(size_t chip_select_pin, uint8_t clock_frequency_mhz, spi_inst_t* spi_instance)
{
  chip_select_pin_ = chip_select_pin;
  spi_port_ = spi_instance;  // Use the provided instance

  gpio_init(chip_select_pin_);
  gpio_set_dir(chip_select_pin_, GPIO_OUT);
  gpio_put(chip_select_pin_, 1);

  specifyClockFrequencyInMHz(clock_frequency_mhz);

  for (uint8_t motor=0; motor<MOTOR_COUNT; ++motor)
  {
    pulse_div_[motor] = 0;
    ramp_div_[motor] = 0;
  }

  spiBegin();
  setStepDiv(STEP_DIV_MAX);
  stopAll();
  initialize();
}

bool TMC429::communicating()
{
  return (getVersion() == VERSION);
}

uint32_t TMC429::getVersion()
{
  return readRegister(SMDA_COMMON, ADDRESS_TYPE_VERSION_429);
}

void TMC429::setRampMode(size_t motor)
{
  setMode(motor, RAMP_MODE);
}

void TMC429::setSoftMode(size_t motor)
{
  setMode(motor, SOFT_MODE);
}

void TMC429::setHoldMode(size_t motor)
{
  setMode(motor, HOLD_MODE);
}

void TMC429::setVelocityMode(size_t motor)
{
  setMode(motor, VELOCITY_MODE);
}

void TMC429::setLimitsInHz(size_t motor, uint32_t velocity_min_hz, uint32_t velocity_max_hz, uint32_t acceleration_max_hz_per_s)
{
  if (motor >= MOTOR_COUNT) return;

  setOptimalStepDivHz(velocity_max_hz);
  setOptimalPulseDivHz(motor, velocity_max_hz);
  setVelocityMinInHz(motor, velocity_min_hz);
  setVelocityMaxInHz(motor, velocity_max_hz);
  setOptimalRampDivHz(motor, velocity_max_hz, acceleration_max_hz_per_s);
  uint32_t a_max = setAccelerationMaxInHzPerS(motor, velocity_max_hz, acceleration_max_hz_per_s);
  setOptimalPropFactor(motor, a_max);
}

uint32_t TMC429::getVelocityMaxUpperLimitInHz()
{
  return convertVelocityToHz(0, VELOCITY_REGISTER_MAX);
}

uint32_t TMC429::getVelocityMinInHz(size_t motor)
{
  if (motor >= MOTOR_COUNT) return 0;
  return convertVelocityToHz(pulse_div_[motor], getVelocityMin(motor));
}

uint32_t TMC429::getVelocityMaxInHz(size_t motor)
{
  if (motor >= MOTOR_COUNT) return 0;
  return convertVelocityToHz(pulse_div_[motor], getVelocityMax(motor));
}

int32_t TMC429::getTargetVelocityInHz(size_t motor)
{
  if (motor >= MOTOR_COUNT) return 0;
  return convertVelocityToHz(pulse_div_[motor], getTargetVelocity(motor));
}

void TMC429::setTargetVelocityInHz(size_t motor, int32_t velocity_hz)
{
  if (motor >= MOTOR_COUNT) return;
  setTargetVelocity(motor, convertVelocityFromHz(pulse_div_[motor], velocity_hz));
}

int16_t TMC429::getTargetVelocity(size_t motor)
{
  if (motor >= MOTOR_COUNT) return 0;
  uint32_t velocity_unsigned = readRegister(motor, ADDRESS_V_TARGET);
  return unsignedToSigned(velocity_unsigned, V_BIT_COUNT);
}

void TMC429::setTargetVelocity(size_t motor, int16_t velocity)
{
  if (motor >= MOTOR_COUNT) return;
  writeRegister(motor, ADDRESS_V_TARGET, velocity);
}

bool TMC429::atTargetVelocity(size_t motor)
{
  if (motor >= MOTOR_COUNT) return true;
  return (getActualVelocity(motor) == getTargetVelocity(motor));
}

int32_t TMC429::getActualVelocityInHz(size_t motor)
{
  if (motor >= MOTOR_COUNT) return 0;
  return convertVelocityToHz(pulse_div_[motor], getActualVelocity(motor));
}

int16_t TMC429::getActualVelocity(size_t motor)
{
  if (motor >= MOTOR_COUNT) return 0;
  uint32_t velocity_unsigned = readRegister(motor, ADDRESS_V_ACTUAL);
  return unsignedToSigned(velocity_unsigned, V_BIT_COUNT);
}

void TMC429::setHoldVelocityMaxInHz(size_t motor, uint32_t velocity_max_hz)
{
  if (motor >= MOTOR_COUNT) return;
  setOptimalStepDivHz(velocity_max_hz);
  setOptimalPulseDivHz(motor, velocity_max_hz);
  setVelocityMaxInHz(motor, velocity_max_hz);
}

void TMC429::setHoldVelocityInHz(size_t motor, int32_t velocity_hz)
{
  if (motor >= MOTOR_COUNT) return;
  setHoldVelocity(motor, convertVelocityFromHz(pulse_div_[motor], velocity_hz));
}

void TMC429::setHoldVelocity(size_t motor, int16_t velocity)
{
  if (motor >= MOTOR_COUNT) return;
  writeRegister(motor, ADDRESS_V_ACTUAL, velocity);
}

uint32_t TMC429::getAccelerationMaxUpperLimitInHzPerS(uint32_t velocity_max_hz)
{
  uint8_t pulse_div = findOptimalPulseDivHz(velocity_max_hz);
  uint8_t ramp_div = (pulse_div > 0) ? (pulse_div - 1) : RAMP_DIV_MIN;
  return getAccelerationMaxUpperLimitInHzPerS(pulse_div, ramp_div);
}

uint32_t TMC429::getAccelerationMaxLowerLimitInHzPerS(uint32_t velocity_max_hz)
{
  uint8_t pulse_div = findOptimalPulseDivHz(velocity_max_hz);
  return getAccelerationMaxLowerLimitInHzPerS(pulse_div, RAMP_DIV_MAX, velocity_max_hz);
}

uint32_t TMC429::getAccelerationMaxInHzPerS(size_t motor)
{
  if (motor >= MOTOR_COUNT) return 0;
  return convertAccelerationToHzPerS(pulse_div_[motor], ramp_div_[motor], getAccelerationMax(motor));
}

uint32_t TMC429::getActualAccelerationInHzPerS(size_t motor)
{
  if (motor >= MOTOR_COUNT) return 0;
  return convertAccelerationToHzPerS(pulse_div_[motor], ramp_div_[motor], getActualAcceleration(motor));
}

int32_t TMC429::getTargetPosition(size_t motor)
{
  if (motor >= MOTOR_COUNT) return 0;
  uint32_t position_unsigned = readRegister(motor, ADDRESS_X_TARGET);
  return unsignedToSigned(position_unsigned, X_BIT_COUNT);
}

void TMC429::setTargetPosition(size_t motor, int32_t position)
{
  if (motor >= MOTOR_COUNT) return;
  writeRegister(motor, ADDRESS_X_TARGET, position);
}

bool TMC429::atTargetPosition(size_t motor)
{
  if (motor >= MOTOR_COUNT) return true;
  return (getActualPosition(motor) == getTargetPosition(motor));
}

int32_t TMC429::getActualPosition(size_t motor)
{
  if (motor >= MOTOR_COUNT) return 0;
  uint32_t position_unsigned = readRegister(motor, ADDRESS_X_ACTUAL);
  return unsignedToSigned(position_unsigned, X_BIT_COUNT);
}

void TMC429::setActualPosition(size_t motor, int32_t position)
{
  if (motor >= MOTOR_COUNT) return;
  writeRegister(motor, ADDRESS_X_ACTUAL, position);
}

void TMC429::stop(size_t motor)
{
  if (motor >= MOTOR_COUNT) return;
  setMode(motor, VELOCITY_MODE);
  setTargetVelocity(motor, 0);
}

void TMC429::stopAll()
{
  for (uint8_t motor=0; motor<MOTOR_COUNT; ++motor)
  {
    stop(motor);
  }
}

void TMC429::enableInverseStepPolarity()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_stp = 1;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::disableInverseStepPolarity()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_stp = 0;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::enableInverseDirPolarity()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_dir = 1;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::disableInverseDirPolarity()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_dir = 0;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::setSwitchesActiveLow()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_ref = 1;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::setSwitchesActiveHigh()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.inv_ref = 0;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::enableLeftSwitchStop(size_t motor)
{
  if (motor >= MOTOR_COUNT) return;
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.disable_stop_l = 0;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429::disableLeftSwitchStop(size_t motor)
{
  if (motor >= MOTOR_COUNT) return;
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.disable_stop_l = 1;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

bool TMC429::leftSwitchStopEnabled(size_t motor)
{
  if (motor >= MOTOR_COUNT) return false;
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  return !ref_conf_mode.ref_conf.disable_stop_l;
}

bool TMC429::leftSwitchActive(size_t motor)
{
  if (motor >= MOTOR_COUNT) return false;
  SwitchState s = getSwitchState();
  if (motor == 0) return s.l0;
  if (motor == 1) return s.l1;
  if (motor == 2) return s.l2;
  return false;
}

void TMC429::enableRightSwitches()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.en_refr = 1;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

void TMC429::disableRightSwitches()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.en_refr = 0;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

bool TMC429::rightSwitchesEnabled()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  return if_conf.if_conf.en_refr;
}

void TMC429::enableRightSwitchStop(size_t motor)
{
  if (motor >= MOTOR_COUNT) return;
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.disable_stop_r = 0;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429::disableRightSwitchStop(size_t motor)
{
  if (motor >= MOTOR_COUNT) return;
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.disable_stop_r = 1;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

bool TMC429::rightSwitchStopEnabled(size_t motor)
{
  if (motor >= MOTOR_COUNT) return false;
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  return !ref_conf_mode.ref_conf.disable_stop_r;
}

bool TMC429::rightSwitchActive(size_t motor)
{
  if (motor >= MOTOR_COUNT) return false;
  SwitchState s = getSwitchState();
  if (motor == 0) return s.r0;
  if (motor == 1) return s.r1;
  if (motor == 2) return s.r2;
  return false;
}

void TMC429::enableSwitchSoftStop(size_t motor)
{
  if (motor >= MOTOR_COUNT) return;
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.soft_stop = 1;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429::disableSwitchSoftStop(size_t motor)
{
  if (motor >= MOTOR_COUNT) return;
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.soft_stop = 0;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

bool TMC429::switchSoftStopEnabled(size_t motor)
{
  if (motor >= MOTOR_COUNT) return false;
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  return ref_conf_mode.ref_conf.soft_stop;
}

void TMC429::setReferenceSwitchToLeft(size_t motor)
{
  if (motor >= MOTOR_COUNT) return;
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.ref_rnl = 0;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429::setReferenceSwitchToRight(size_t motor)
{
  if (motor >= MOTOR_COUNT) return;
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.ref_conf.ref_rnl = 1;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

void TMC429::startLatchPositionWaiting(size_t motor)
{
  if (motor >= MOTOR_COUNT) return;
  writeRegister(motor, ADDRESS_X_LATCHED, 0);
}

bool TMC429::latchPositionWaiting(size_t motor)
{
  RefConfMode ref_conf_mode;
  ref_conf_mode.lp = false;
  if (motor < MOTOR_COUNT)
  {
    ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  }
  return ref_conf_mode.lp;
}

int32_t TMC429::getLatchPosition(size_t motor)
{
  if (motor >= MOTOR_COUNT) return 0;
  uint32_t position_unsigned = readRegister(motor, ADDRESS_X_LATCHED);
  return unsignedToSigned(position_unsigned, X_BIT_COUNT);
}

void TMC429::setPositionCompareMotor(size_t motor)
{
  if (motor >= MOTOR_COUNT) return;
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.pos_comp_sel = motor;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

TMC429::Status TMC429::getStatus()
{
  getVersion();
  return status_;
}

void TMC429::initialize()
{
  setStepDirOutput();
}

void TMC429::setStepDirOutput()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  if_conf.if_conf.en_sd = 1;
  writeRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429, if_conf.bytes);
}

uint32_t TMC429::readRegister(uint8_t smda, uint8_t address)
{
  MosiDatagram mosi_datagram;
  mosi_datagram.rrs = RRS_REGISTER;
  mosi_datagram.address = address;
  mosi_datagram.smda = smda;
  mosi_datagram.rw = RW_READ;
  mosi_datagram.data = 0;
  return writeRead(mosi_datagram).data;
}

void TMC429::writeRegister(uint8_t smda, uint8_t address, uint32_t data)
{
  MosiDatagram mosi_datagram;
  mosi_datagram.rrs = RRS_REGISTER;
  mosi_datagram.address = address;
  mosi_datagram.smda = smda;
  mosi_datagram.rw = RW_WRITE;
  mosi_datagram.data = data;
  writeRead(mosi_datagram);
}

TMC429::MisoDatagram TMC429::writeRead(MosiDatagram mosi_datagram)
{
  MisoDatagram miso_datagram;
  miso_datagram.bytes = 0x0;
  beginTransaction();
  for (int i=(DATAGRAM_SIZE - 1); i>=0; --i)
  {
    uint8_t byte_write = (mosi_datagram.bytes >> (8*i)) & 0xff;
    uint8_t byte_read = spiTransfer(byte_write);
    miso_datagram.bytes |= ((uint32_t)byte_read) << (8*i);
  }
  endTransaction();
  status_ = miso_datagram.status;
  return miso_datagram;
}

int32_t TMC429::unsignedToSigned(uint32_t input_value, uint8_t num_bits)
{
  uint32_t mask = 1 << (num_bits - 1);
  return -(input_value & mask) + (input_value & ~mask);
}

void TMC429::specifyClockFrequencyInMHz(uint8_t clock_frequency)
{
  clock_frequency_ = (clock_frequency <= CLOCK_FREQUENCY_MAX) ? clock_frequency : CLOCK_FREQUENCY_MAX;
}

void TMC429::setOptimalStepDivHz(uint32_t velocity_max_hz)
{
  int step_div = getStepDiv();
  double step_time = stepDivToStepTime(step_div);
  uint32_t velocity_max_upper_limit = (double)MHZ_PER_HZ/(step_time*2);

  while ((velocity_max_upper_limit < velocity_max_hz) && (step_div >= 1))
  {
    --step_div;
    step_time = stepDivToStepTime(step_div);
    velocity_max_upper_limit = (double)MHZ_PER_HZ/(step_time*2);
  }
  setStepDiv(step_div);
}

uint8_t TMC429::getStepDiv()
{
  GlobalParameters gp;
  gp.bytes = readRegister(SMDA_COMMON, ADDRESS_GLOBAL_PARAMETERS);
  return gp.clk2_div & STEP_DIV_MASK;
}

void TMC429::setStepDiv(uint8_t step_div)
{
  GlobalParameters gp;
  gp.bytes = readRegister(SMDA_COMMON, ADDRESS_GLOBAL_PARAMETERS);
  gp.clk2_div = step_div & STEP_DIV_MASK;
  writeRegister(SMDA_COMMON, ADDRESS_GLOBAL_PARAMETERS, gp.bytes);
}

double TMC429::stepDivToStepTime(uint8_t step_div)
{
  return (double)(16*(1 + step_div))/(double)clock_frequency_;
}

int32_t TMC429::convertVelocityToHz(uint8_t pulse_div, int16_t velocity)
{
  double x = ((double)clock_frequency_*(double)MHZ_PER_HZ)/(double)VELOCITY_CONSTANT;
  return (x*(double)velocity)/((double)(1 << pulse_div));
}

int16_t TMC429::convertVelocityFromHz(uint8_t pulse_div, int32_t velocity)
{
  double x = ((double)velocity*(double)(1 << pulse_div))/((double)clock_frequency_*(double)MHZ_PER_HZ);
  return x*(double)VELOCITY_CONSTANT;
}

uint8_t TMC429::findOptimalPulseDivHz(uint32_t velocity_max_hz)
{
  uint8_t pulse_div = PULSE_DIV_MAX + 1;
  uint32_t velocity_max_upper_limit = 0;
  while ((velocity_max_upper_limit < velocity_max_hz) && (pulse_div >= 1))
  {
    --pulse_div;
    velocity_max_upper_limit = getVelocityMaxUpperLimitInHz(pulse_div);
  }
  return pulse_div;
}

void TMC429::setOptimalPulseDivHz(size_t motor, uint32_t velocity_max_hz)
{
  uint8_t pulse_div = findOptimalPulseDivHz(velocity_max_hz);
  ClkConfig clk_config;
  clk_config.bytes = readRegister(motor, ADDRESS_CLOCK_CONFIGURATION);
  clk_config.clk_config.pulse_div = pulse_div;
  writeRegister(motor, ADDRESS_CLOCK_CONFIGURATION, clk_config.bytes);
  pulse_div_[motor] = pulse_div;
}

TMC429::Mode TMC429::getMode(size_t motor)
{
  if (motor >= MOTOR_COUNT) return RAMP_MODE;
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  return (Mode)ref_conf_mode.mode;
}

void TMC429::setMode(size_t motor, Mode mode)
{
  if (motor >= MOTOR_COUNT) return;
  RefConfMode ref_conf_mode;
  ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  ref_conf_mode.mode = (uint8_t)mode;
  writeRegister(motor, ADDRESS_REF_CONF_MODE, ref_conf_mode.bytes);
}

TMC429::ReferenceConfiguration TMC429::getReferenceConfiguration(size_t motor)
{
  RefConfMode ref_conf_mode;
  if (motor < MOTOR_COUNT)
  {
    ref_conf_mode.bytes = readRegister(motor, ADDRESS_REF_CONF_MODE);
  }
  return ref_conf_mode.ref_conf;
}

TMC429::InterfaceConfiguration TMC429::getInterfaceConfiguration()
{
  IfConf if_conf;
  if_conf.bytes = readRegister(SMDA_COMMON, ADDRESS_IF_CONFIGURATION_429);
  return if_conf.if_conf;
}

TMC429::SwitchState TMC429::getSwitchState()
{
  SwState switch_state;
  switch_state.bytes = readRegister(SMDA_COMMON, ADDRESS_SWITCHES);
  return switch_state.switch_state;
}

TMC429::ClockConfiguration TMC429::getClockConfiguration(size_t motor)
{
  ClkConfig clk_config;
  if (motor < MOTOR_COUNT)
  {
    clk_config.bytes = readRegister(motor, ADDRESS_CLOCK_CONFIGURATION);
  }
  return clk_config.clk_config;
}

double TMC429::getProportionalityFactor(size_t motor)
{
  if (motor >= MOTOR_COUNT) return 0.0;
  PropFactor prop_factor;
  prop_factor.bytes = readRegister(motor, ADDRESS_PROP_FACTOR);
  return ((double)prop_factor.pmul) / ((double)(1 << (prop_factor.pdiv + 3)));
}

double TMC429::getStepTimeInMicroS()
{
  return stepDivToStepTime(getStepDiv());
}

uint16_t TMC429::getVelocityMin(size_t motor)
{
  return readRegister(motor, ADDRESS_V_MIN);
}

void TMC429::setVelocityMinInHz(size_t motor, uint32_t velocity_min_hz)
{
  uint32_t velocity_min = convertVelocityFromHz(pulse_div_[motor], velocity_min_hz);
  if (velocity_min < VELOCITY_MIN_MIN)
  {
    velocity_min = VELOCITY_MIN_MIN;
  }
  writeRegister(motor, ADDRESS_V_MIN, velocity_min);
}

uint16_t TMC429::getVelocityMax(size_t motor)
{
  return readRegister(motor, ADDRESS_V_MAX);
}

void TMC429::setVelocityMaxInHz(size_t motor, uint32_t velocity_max_hz)
{
  uint32_t velocity_max = convertVelocityFromHz(pulse_div_[motor], velocity_max_hz);
  uint32_t velocity_max_upper_limit = getVelocityMaxUpperLimitInHz();
  if (velocity_max > velocity_max_upper_limit)
  {
    velocity_max = velocity_max_upper_limit;
  }
  writeRegister(motor, ADDRESS_V_MAX, velocity_max);
}

uint32_t TMC429::convertAccelerationToHzPerS(uint8_t pulse_div, uint8_t ramp_div, uint32_t acceleration)
{
  double a = ((double)clock_frequency_*(double)MHZ_PER_HZ)/(double)ACCELERATION_CONSTANT;
  double b = a*(double)clock_frequency_*(double)MHZ_PER_HZ;
  double c = b/((double)(1 << pulse_div));
  double d = c/((double)(1 << ramp_div));
  return (uint32_t)round(d*(double)acceleration);
}

uint32_t TMC429::convertAccelerationFromHzPerS(uint8_t pulse_div, uint8_t ramp_div, uint32_t acceleration)
{
  double a = ((double)acceleration*(double)(1 << pulse_div))/((double)clock_frequency_*(double)MHZ_PER_HZ);
  double b = a*(double)ACCELERATION_CONSTANT;
  double c = b/((double)clock_frequency_*(double)MHZ_PER_HZ);
  uint32_t d = round(c*(1 << ramp_div));
  return d;
}

uint8_t TMC429::findOptimalRampDivHz(uint32_t velocity_max_hz, uint32_t acceleration_max_hz_per_s)
{
  uint8_t pulse_div = findOptimalPulseDivHz(velocity_max_hz);
  uint8_t ramp_div = RAMP_DIV_MAX;
  uint32_t acceleration_max_upper_limit = getAccelerationMaxUpperLimitInHzPerS(pulse_div, ramp_div);
  uint32_t acceleration_max_lower_limit = getAccelerationMaxLowerLimitInHzPerS(pulse_div, ramp_div, velocity_max_hz);

  while ((acceleration_max_upper_limit < acceleration_max_hz_per_s) &&
    (acceleration_max_lower_limit < acceleration_max_hz_per_s) &&
    (ramp_div >= 1) &&
    (ramp_div >= pulse_div))
  {
    --ramp_div;
    acceleration_max_upper_limit = getAccelerationMaxUpperLimitInHzPerS(pulse_div, ramp_div);
    acceleration_max_lower_limit = getAccelerationMaxLowerLimitInHzPerS(pulse_div, ramp_div, velocity_max_hz);
  }
  return ramp_div;
}

void TMC429::setOptimalRampDivHz(size_t motor, uint32_t velocity_max_hz, uint32_t acceleration_max_hz_per_s)
{
  uint8_t ramp_div = findOptimalRampDivHz(velocity_max_hz, acceleration_max_hz_per_s);
  ClkConfig clk_config;
  clk_config.bytes = readRegister(motor, ADDRESS_CLOCK_CONFIGURATION);
  clk_config.clk_config.ramp_div = ramp_div;
  writeRegister(motor, ADDRESS_CLOCK_CONFIGURATION, clk_config.bytes);
  ramp_div_[motor] = ramp_div;
}

uint32_t TMC429::getVelocityMaxUpperLimitInHz(uint8_t pulse_div)
{
  return convertVelocityToHz(pulse_div, VELOCITY_REGISTER_MAX);
}

uint32_t TMC429::getAccelerationMaxUpperLimitInHzPerS(uint8_t pulse_div, uint8_t ramp_div)
{
  uint32_t a_max_upper_limit;
  if (((int8_t)ramp_div - (int8_t)pulse_div + 1) >= 0)
  {
    a_max_upper_limit = ACCELERATION_REGISTER_MAX;
  }
  else if (((int8_t)ramp_div - (int8_t)pulse_div + 12) < 1)
  {
    a_max_upper_limit = ACCELERATION_REGISTER_MIN;
  }
  else
  {
    a_max_upper_limit = (1 << ((int8_t)ramp_div - (int8_t)pulse_div + 12)) - 1;
  }
  if (a_max_upper_limit > ACCELERATION_REGISTER_MAX)
  {
    a_max_upper_limit = ACCELERATION_REGISTER_MAX;
  }
  if (a_max_upper_limit < ACCELERATION_REGISTER_MIN)
  {
    a_max_upper_limit = ACCELERATION_REGISTER_MIN;
  }
  return convertAccelerationToHzPerS(pulse_div, ramp_div, a_max_upper_limit);
}

uint32_t TMC429::getAccelerationMaxLowerLimitInHzPerS(uint8_t pulse_div, uint8_t ramp_div, uint32_t velocity_max)
{
  uint32_t a_max_lower_limit;
  if (((int8_t)ramp_div - (int8_t)pulse_div - 1) <= 0)
  {
    a_max_lower_limit = ACCELERATION_REGISTER_MIN;
  }
  else
  {
    a_max_lower_limit = (1 << ((int8_t)ramp_div - (int8_t)pulse_div - 1));
    if (convertVelocityFromHz(pulse_div, velocity_max) <= (int16_t)VELOCITY_REGISTER_THRESHOLD)
    {
      a_max_lower_limit /= 2;
    }
  }
  if (a_max_lower_limit > ACCELERATION_REGISTER_MAX)
  {
    a_max_lower_limit = ACCELERATION_REGISTER_MAX;
  }
  if (a_max_lower_limit < ACCELERATION_REGISTER_MIN)
  {
    a_max_lower_limit = ACCELERATION_REGISTER_MIN;
  }
  return convertAccelerationToHzPerS(pulse_div, ramp_div, a_max_lower_limit);
}

uint32_t TMC429::getAccelerationMax(size_t motor)
{
  return readRegister(motor, ADDRESS_A_MAX);
}

uint32_t TMC429::setAccelerationMaxInHzPerS(size_t motor, uint32_t velocity_max_hz, uint32_t acceleration_max_hz_per_s)
{
  uint32_t acceleration_max_upper_limit = getAccelerationMaxUpperLimitInHzPerS(pulse_div_[motor], ramp_div_[motor]);
  uint32_t acceleration_max_lower_limit = getAccelerationMaxLowerLimitInHzPerS(pulse_div_[motor], ramp_div_[motor], velocity_max_hz);
  if (acceleration_max_hz_per_s > acceleration_max_upper_limit)
  {
    acceleration_max_hz_per_s = acceleration_max_upper_limit;
  }
  if (acceleration_max_hz_per_s < acceleration_max_lower_limit)
  {
    acceleration_max_hz_per_s = acceleration_max_lower_limit;
  }
  uint32_t acceleration_max = convertAccelerationFromHzPerS(pulse_div_[motor], ramp_div_[motor], acceleration_max_hz_per_s);
  if (acceleration_max > ACCELERATION_REGISTER_MAX)
  {
    acceleration_max = ACCELERATION_REGISTER_MAX;
  }
  if (acceleration_max < ACCELERATION_REGISTER_MIN)
  {
    acceleration_max = ACCELERATION_REGISTER_MIN;
  }
  writeRegister(motor, ADDRESS_A_MAX, acceleration_max);
  return acceleration_max;
}

int16_t TMC429::getActualAcceleration(size_t motor)
{
  uint32_t acceleration_unsigned = readRegister(motor, ADDRESS_A_ACTUAL);
  return unsignedToSigned(acceleration_unsigned, A_BIT_COUNT);
}

void TMC429::setOptimalPropFactor(size_t motor, uint32_t acceleration_max)
{
  int pdiv, pmul, pm, pd;
  double p_ideal, p_reduced;

  pm=-1; pd=-1;
  p_ideal = acceleration_max/(128.0*(1 << (ramp_div_[motor] - pulse_div_[motor])));
  p_reduced = p_ideal*0.99;
  for (pdiv=0; pdiv<=13; ++pdiv)
  {
    pmul = (int)(p_reduced*8.0*(1 << pdiv)) - 128;
    if ((0 <= pmul) && (pmul <= 127))
    {
      pm = pmul + 128;
      pd = pdiv;
    }
  }
  if ((pm == -1) || (pd == -1))
  {
    return;
  }
  PropFactor prop_factor;
  prop_factor.bytes = readRegister(motor, ADDRESS_PROP_FACTOR);
  prop_factor.pmul = pm;
  prop_factor.pdiv = pd;
  writeRegister(motor, ADDRESS_PROP_FACTOR, prop_factor.bytes);
}

void TMC429::enableChipSelect()
{
  gpio_put(chip_select_pin_, 0); // LOW
}

void TMC429::disableChipSelect()
{
  gpio_put(chip_select_pin_, 1); // HIGH
}

void TMC429::beginTransaction()
{
  // No SPISettings needed for Pico - configure in spiBegin
  sleep_us(1);
  enableChipSelect();
}

void TMC429::endTransaction()
{
  disableChipSelect();
  sleep_us(1);
}

void TMC429::spiBegin()
{
  spi_init(spi_port_, SPI_CLOCK);
  spi_set_format(spi_port_, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  
  // Set these to match your wires!
  gpio_set_function(2, GPIO_FUNC_SPI); // SCK
  gpio_set_function(3, GPIO_FUNC_SPI); // TX (SDI)
  gpio_set_function(4, GPIO_FUNC_SPI); // RX (SDO)
}

uint8_t TMC429::spiTransfer(uint8_t byte)
{
  uint8_t rx_byte;
  spi_write_read_blocking(spi_port_, &byte, &rx_byte, 1);  // Change spi_default to spi_port_
  return rx_byte;
}

