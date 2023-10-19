#include "reMCP23017.h"
#include "reI2C.h"
#include "rTypes.h"
#include "rLog.h"
#include "reEvents.h"

static const char* logTAG = "MCP23017";

#define REG_IODIRA       0x00   // IODIR: I/O DIRECTION REGISTER (ADDR 0x00)
#define REG_IODIRB       0x01   //        Controls the direction of the data I/O. 1 = Pin is configured as an input. 0 = Pin is configured as an output.
#define REG_IPOLA        0x02   // IPOL: INPUT POLARITY PORT REGISTER (ADDR 0x01)
#define REG_IPOLB        0x03   //        If a bit is set, the corresponding GPIO register bit will reflect the inverted value on the pin.
#define REG_GPINTENA     0x04   // GPINTEN: INTERRUPT-ON-CHANGE PINS (ADDR 0x02)
#define REG_GPINTENB     0x05   //        General purpose I/O interrupt-on-change bits. 1 = Enables GPIO input pin for interrupt-on-change event
#define REG_DEFVALA      0x06   // DEFVAL: DEFAULT VALUE REGISTER (ADDR 0x03)
#define REG_DEFVALB      0x07   //        Sets the compare value for pins configured for interrupt-on-change from defaults. If the associated pin level is the opposite from the register bit, an interrupt occurs
#define REG_INTCONA      0x08   // INTCON: INTERRUPT-ON-CHANGE CONTROL REGISTER (ADDR 0x04)
#define REG_INTCONB      0x09   //        Controls how the associated pin value is compared for interrupt-on-change. 1 = Pin value is compared against the associated bit in the DEFVAL register. Pin value is compared against the previous pin value
#define REG_IOCON        0x0A   // CONFIGURATION REGISTER
#define REG_GPPUA        0x0C   // GPPU: GPIO PULL-UP RESISTOR REGISTER (ADDR 0x06)
#define REG_GPPUB        0x0D   //        Controls the weak pull-up resistors on each pin (when configured as an input). 1 = Pull-up enabled. 0 = Pull-up disabled
#define REG_INTFA        0x0E   // INTF: INTERRUPT FLAG REGISTER (ADDR 0x07)
#define REG_INTFB        0x0F   //        Reflects the interrupt condition on the port. It reflects the change only if interrupts are enabled per GPINTEN. 1 = Pin caused interrupt. 0 = Interrupt not pending
#define REG_INTCAPA      0x10   // INTCAP: INTERRUPT CAPTURED VALUE FOR PORT REGISTER (ADDR 0x08)
#define REG_INTCAPB      0x11   //        Reflects the logic level on the port pins at the time of interrupt due to pin change. 1 = Logic-high. 0 = Logic-low
#define REG_GPIOA        0x12   // GPIO: GENERAL PURPOSE I/O PORT REGISTER (ADDR 0x09)
#define REG_GPIOB        0x13   //        Reflects the logic level on the pins. 1 = Logic-high. 0 = Logic-low
#define REG_OLATA        0x14   // OLAT: OUTPUT LATCH REGISTER 0 (ADDR 0x0A)
#define REG_OLATB        0x15   //        Reflects the logic level on the output latch. 1 = Logic-high. 0 = Logic-low

#define BIT_IOCON_INTPOL BIT1   // This bit sets the polarity of the INT output pin. 1 = Active-high. 0 = Active-low
#define BIT_IOCON_ODR    BIT2   // Configures the INT pin as an open-drain output. 1 = Open-drain output (overrides the INTPOL bit.). 0 = Active driver output (INTPOL bit sets the polarity.)
#define BIT_IOCON_HAEN   BIT3   // Hardware Address Enable bit (MCP23S17 only)
#define BIT_IOCON_DISSLW BIT4   // Slew Rate control bit for SDA output. 1 = Slew rate disabled. 0 = Slew rate enabled
#define BIT_IOCON_SEQOP  BIT5   // Sequential Operation mode bit. 1 = Sequential operation disabled, address pointer does not increment. 0 = Sequential operation enabled, address pointer increments
#define BIT_IOCON_MIRROR BIT6   // INT Pins Mirror bit. 1 = The INT pins are internally connected. 0 = The INT pins are not connected. INTA is associated with PORTA and INTB is associated with PORTB
#define BIT_IOCON_BANK   BIT7   // Controls how the registers are addressed. 1 = The registers associated with each port are separated into different banks. 0 = The registers are in the same bank (addresses are sequential).

#define MCP23017_TIMEOUT 1000

reMCP23017::reMCP23017(i2c_port_t numI2C, uint8_t addrI2C, cb_gpio_change_t callback)
{
  _numI2C = numI2C; 
  _addrI2C = addrI2C;
  _callback = callback;
}

reMCP23017::~reMCP23017()
{
}

void reMCP23017::setCallback(cb_gpio_change_t callback)
{
  _callback = callback;
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------- I2C ---------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool reMCP23017::read8(uint8_t reg, uint8_t* value)
{
  esp_err_t err = readI2C(_numI2C, _addrI2C, &reg, 1, value, 1, 0, MCP23017_TIMEOUT); 
  if (err == ESP_OK) {
    // rlog_d(logTAG, "Read from MCP23017 %d.0x%2x register 0x%2x: 0x%2x", _numI2C, _addrI2C, reg, *value);
    return true;
  };
  rlog_e(logTAG, "Failed to read MCP23017 %d.0x%2x register 0x%2x: %d (%s)", _numI2C, _addrI2C, reg, err, esp_err_to_name(err));
  return false;
}

bool reMCP23017::write8(uint8_t reg, uint8_t value)
{
  esp_err_t err = writeI2C(_numI2C, _addrI2C, &reg, 1, &value, 1, MCP23017_TIMEOUT);
  if (err == ESP_OK) {
    // rlog_d(logTAG, "Write to MCP23017 %d.0x%2x register 0x%2x: 0x%2x", _numI2C, _addrI2C, reg, value);
    return true;
  };
  rlog_e(logTAG, "Failed to write MCP23017 %d.0x%2x register 0x%2x: %d (%s)", _numI2C, _addrI2C, reg, err, esp_err_to_name(err));
  return false;
}

bool reMCP23017::read16(uint8_t reg, uint16_t* value)
{
  esp_err_t err = readI2C(_numI2C, _addrI2C, &reg, 1, (uint8_t*)value, 2, 0, MCP23017_TIMEOUT); 
  if (err == ESP_OK) {
    // rlog_d(logTAG, "Read from MCP23017 %d.0x%2x register 0x%2x: 0x%4x", _numI2C, _addrI2C, reg, *value);
    return true;
  };
  rlog_e(logTAG, "Failed to read MCP23017 %d.0x%2x register 0x%2x: %d (%s)", _numI2C, _addrI2C, reg, err, esp_err_to_name(err));
  return false;
}

bool reMCP23017::write16(uint8_t reg, uint16_t value)
{
  esp_err_t err = writeI2C(_numI2C, _addrI2C, &reg, 1, (uint8_t*)&value, 2, MCP23017_TIMEOUT);
  if (err == ESP_OK) {
    // rlog_d(logTAG, "Write to MCP23017 %d.0x%2x register 0x%2x: 0x%4x", _numI2C, _addrI2C, reg, value);
    return true;
  };
  rlog_e(logTAG, "Failed to write MCP23017 %d.0x%2x register 0x%2x: %d (%s)", _numI2C, _addrI2C, reg, err, esp_err_to_name(err));
  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Configure ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool reMCP23017::configGet(mcp23017_int_out_mode_t *intOutMode, bool *intOutMirror)
{
  uint8_t config;
  // Read current config
  if (read8(REG_IOCON, &config)) {
    // Get mode of the INT output pin
    if (config & BIT_IOCON_ODR) {
      *intOutMode = MCP23017_OPEN_DRAIN;
    } else {
      if (config & BIT_IOCON_INTPOL) {
        *intOutMode = MCP23017_ACTIVE_HIGH;
      } else {
        *intOutMode = MCP23017_ACTIVE_LOW;
      };
    };
    // Get INT pins are internally connected
    *intOutMirror = (config & BIT_IOCON_MIRROR);
    return true;
  };
  return false;
}

bool reMCP23017::configSet(mcp23017_int_out_mode_t intOutMode, bool intOutMirror)
{
  uint8_t config;
  // Read current config
  if (read8(REG_IOCON, &config)) {
    // Set 16-bit mode (BANK = 0)
    config &= ~BIT_IOCON_BANK;
    // Set mode of the INT output pin
    if (intOutMode == MCP23017_ACTIVE_HIGH) {
      config &= ~BIT_IOCON_ODR;
      config |= BIT_IOCON_INTPOL;
    } else if (intOutMode == MCP23017_ACTIVE_LOW) {
      config &= ~BIT_IOCON_ODR;
      config &= ~BIT_IOCON_INTPOL;
    } else {
      config |= BIT_IOCON_ODR;
      config &= ~BIT_IOCON_INTPOL;
    };
    // Set INT pins are internally connected
    intOutMirror ? config |= BIT_IOCON_MIRROR : config &= ~BIT_IOCON_MIRROR;
    // Write config
    return write8(REG_IOCON, config);
  };
  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------- Inputs / outputs ---------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool reMCP23017::portGetMode(uint16_t * value)
{
  return read16(REG_IODIRA, value);
}

bool reMCP23017::portSetMode(uint16_t value)
{
  return write16(REG_IODIRA, value);
}

bool reMCP23017::pinGetMode(uint8_t pin, mcp23017_gpio_mode_t *mode)
{
  uint16_t buf;
  if (read16(REG_IODIRA, &buf)) {
    if ((buf & (1 << pin)) > 0) {
      *mode = MCP23017_GPIO_INPUT;
    } else {
      *mode = MCP23017_GPIO_OUTPUT;
    };
    return true;
  };
  return false;
}

bool reMCP23017::pinSetMode(uint8_t pin, mcp23017_gpio_mode_t mode)
{
  uint16_t buf;
  if (read16(REG_IODIRA, &buf)) {
    (mode == MCP23017_GPIO_INPUT) ? buf |= (1 << pin) : buf &= ~(1 << pin);
    return write16(REG_IODIRA, buf);
  };
  return false;
}

bool reMCP23017::portGetInputPolarity(uint16_t * value)
{
  return read16(REG_IPOLA, value);
}

bool reMCP23017::portSetInputPolarity(uint16_t value)
{
  return write16(REG_IPOLA, value);
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- Internal pullup ---------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool reMCP23017::portGetPullup(uint16_t * value)
{
  return read16(REG_GPPUA, value);
}

bool reMCP23017::portSetPullup(uint16_t value)
{
  return write16(REG_GPPUA, value);
}

bool reMCP23017::pinGetPullup(uint8_t pin, bool *enable)
{
  uint16_t buf;
  if (read16(REG_GPPUA, &buf)) {
    *enable = (buf & (1 << pin)) > 0;
    return true;
  };
  return false;
}

bool reMCP23017::pinSetPullup(uint8_t pin, bool enable)
{
  uint16_t buf;
  if (read16(REG_GPPUA, &buf)) {
    enable ? buf |= (1 << pin) : buf &= ~(1 << pin);
    return write16(REG_GPPUA, buf);
  };
  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------- Unbuffered read / write -----------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool reMCP23017::portRead(uint16_t * value)
{
  return read16(REG_GPIOA, value);
}

bool reMCP23017::portWrite(uint16_t value)
{
  return write16(REG_GPIOA, value);
}

bool reMCP23017::pinRead(uint8_t pin, bool *level)
{
  uint16_t buf;
  if (read16(REG_GPIOA, &buf)) {
    *level = (buf & (1 << pin)) > 0;
    return true;
  };
  return false;
}

bool reMCP23017::pinWrite(uint8_t pin, bool level)
{
  uint16_t buf;
  if (read16(REG_GPIOA, &buf)) {
    level ? buf |= (1 << pin) : buf &= ~(1 << pin);
    return write16(REG_GPIOA, buf);
  };
  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------- Interrupts ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool reMCP23017::getIntFlags(uint16_t* flags)
{
  return read16(REG_INTFA, flags);
}

bool reMCP23017::getIntCapture(uint16_t* bits)
{
  return read16(REG_INTCAPA, bits);
}

// -----------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------- Latch outputs ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

/**
 * OUTPUT LATCH REGISTER (OLAT)
 * The OLAT register provides access to the output latches. 
 * A read from this register results in a read of the OLAT and not the port itself. 
 * A write to this register modifies the output latches that modifies the pins configured as outputs.
 **/

bool reMCP23017::portGetLatch(uint16_t * value)
{
  return read16(REG_OLATA, value);
}

bool reMCP23017::portSetLatch(uint16_t value)
{
  return write16(REG_OLATA, value);
}

bool reMCP23017::pinGetLatch(uint8_t pin, bool *level)
{
  uint16_t buf;
  if (read16(REG_OLATA, &buf)) {
    *level = (buf & (1 << pin)) > 0;
    return true;
  };
  return false;
}

bool reMCP23017::pinSetLatch(uint8_t pin, bool level)
{
  uint16_t buf;
  if (read16(REG_OLATA, &buf)) {
    level ? buf |= (1 << pin) : buf &= ~(1 << pin);
    return write16(REG_OLATA, buf);
  };
  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- Port interrupts ---------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool reMCP23017::portSetInterrupt(uint16_t mask, mcp23017_gpio_intr_t intr)
{
  // GPINTEN: INTERRUPT-ON-CHANGE PINS (ADDR 0x02)
  // General purpose I/O interrupt-on-change bits. 
  // 1 = Enables GPIO input pin for interrupt-on-change event
  // 0 = Disables GPIO input pin for interrupt-on-change event
  uint16_t int_en;
  if (read16(REG_GPINTENA, &int_en)) {
    if (intr == MCP23017_INT_DISABLED) {
      // Disable interrupts
      int_en &= ~mask;
    } else {
      // Enable interrupts
      int_en |= mask;

      // INTCON: INTERRUPT-ON-CHANGE CONTROL REGISTER (ADDR 0x04)
      // Controls how the associated pin value is compared for interrupt-on-change
      // 1 = Pin value is compared against the associated bit in the DEFVAL register
      // 0 = Pin value is compared against the previous pin value.
      uint16_t int_con;
      if (read16(REG_INTCONA, &int_con)) {
        if (intr == MCP23017_INT_ANY_EDGE) {
          // Pin value is compared against the previous pin value
          int_con &= ~mask;
        } else {
          // Pin value is compared against the associated bit in the DEFVAL register
          int_con |= mask;

          // DEFVAL: DEFAULT VALUE REGISTER (ADDR 0x03)
          // Sets the compare value for pins configured for interrupt-on-change from defaults
          // If the associated pin level is the opposite from the register bit, an interrupt occurs. 
          uint16_t int_def;
          if (read16(REG_DEFVALA, &int_def)) {
            if (intr == MCP23017_INT_LOW_EDGE) {
              int_def |= mask;
            } else {
              int_def &= ~mask;
            };
            if (!write16(REG_DEFVALA, int_def)) return false;
          };
        };
        if (!write16(REG_INTCONA, int_con)) return false;
      };
    };
    return write16(REG_GPINTENA, int_en);
  };
  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------ Refresh of interrupts ------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool reMCP23017::portUpdate(uint16_t mask)
{
  uint16_t pins = 0;
  // Read and reset INTF register
  read16(REG_INTFA, &pins);
  // Read PINs for the current moment
  if (read16(REG_GPIOA, &pins)) {
    // Prepare data for events
    gpio_data_t data;
    data.bus = (uint8_t)_numI2C + 1;
    data.address = _addrI2C;

    // Send events only for pins that have changed
    for (uint8_t i = 0; i < 16; i++) {
      if ((mask & (1 << i)) > 0) {
        data.pin = i;
        data.value = (uint8_t)((pins & (1 << i)) > 0);
        if (_callback) {
          _callback((void*)this, data, 0);
        } else {
          eventLoopPost(RE_GPIO_EVENTS, RE_GPIO_CHANGE, &data, sizeof(data), portMAX_DELAY);
        };
      };
    };
    return true;
  };
  return false;
}

bool reMCP23017::portOnInterrupt(bool useIntCap)
{
  uint16_t flags;
  // Read which PINs caused the interrupt
  if (read16(REG_INTFA, &flags)) {
    if (flags != 0) {
      // At least one PIN caused an interrupt
      uint16_t pins;
      if (useIntCap) {
        // Read PINs at the moment of interrupt generation
        if (!read16(REG_INTCAPA, &pins)) {
          return false;
        };
      } else {
        // Read PINs for the current moment
        if (!read16(REG_GPIOA, &pins)) {
          return false;
        };
      };
      
      // Prepare data for events
      gpio_data_t data;
      data.bus = (uint8_t)_numI2C + 1;
      data.address = _addrI2C;

      // Send events only for pins that have changed
      for (uint8_t i = 0; i < 16; i++) {
        if ((flags & (1 << i)) > 0) {
          data.pin = i;
          data.value = (uint8_t)((pins & (1 << i)) > 0);
          if (_callback) {
            _callback((void*)this, data, 0);
          } else {
            eventLoopPost(RE_GPIO_EVENTS, RE_GPIO_CHANGE, &data, sizeof(data), portMAX_DELAY);
          };
        };
      };
    };
    return true;
  };
  return false;
}