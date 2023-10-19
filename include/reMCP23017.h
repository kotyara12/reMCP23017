/* 
   EN: Class for working with the MCP23017 port expander
   RU: Класс для работы с расширителем портов MCP23017
   --------------------------------------------------------------------------------
   (с) 2022 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_MCP23017_H__
#define __RE_MCP23017_H__

#include <stdint.h>
#include <esp_err.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "rTypes.h"
#include "reGpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// GPIO mode
typedef enum {
  MCP23017_GPIO_OUTPUT = 0,
  MCP23017_GPIO_INPUT
} mcp23017_gpio_mode_t;

// Interrupt outputs (INTA/INTB) mode
typedef enum {
  MCP23017_ACTIVE_LOW = 0,   // Low level on interrupt
  MCP23017_ACTIVE_HIGH,      // High level on interrupt
  MCP23017_OPEN_DRAIN        // Open drain
} mcp23017_int_out_mode_t;

// Interrupt inputs mode
typedef enum {
  MCP23017_INT_DISABLED = 0, // No interrupt
  MCP23017_INT_LOW_EDGE,     // Interrupt on low edge
  MCP23017_INT_HIGH_EDGE,    // Interrupt on high edge
  MCP23017_INT_ANY_EDGE      // Interrupt on any edge
} mcp23017_gpio_intr_t;

class reMCP23017 {
  public:
    reMCP23017(i2c_port_t numI2C, uint8_t addrI2C, cb_gpio_change_t callback);
    ~reMCP23017();

    void setCallback(cb_gpio_change_t callback);

    // Configure
    bool configGet(mcp23017_int_out_mode_t *intOutMode, bool *intOutMirror);
    bool configSet(mcp23017_int_out_mode_t intOutMode, bool intOutMirror);

    // Inputs / outputs
    bool portGetMode(uint16_t * value);
    bool portSetMode(uint16_t value);
    bool pinGetMode(uint8_t pin, mcp23017_gpio_mode_t *mode);
    bool pinSetMode(uint8_t pin, mcp23017_gpio_mode_t mode);
    bool portGetInputPolarity(uint16_t * value);
    bool portSetInputPolarity(uint16_t value);

    // Internal pullup
    bool portGetPullup(uint16_t * value);
    bool portSetPullup(uint16_t value);
    bool pinGetPullup(uint8_t pin, bool *enable);
    bool pinSetPullup(uint8_t pin, bool enable);

    // Unbuffered read / write
    bool portRead(uint16_t * value);
    bool portWrite(uint16_t value);
    bool portUpdate(uint16_t mask);
    bool pinRead(uint8_t pin, bool *level);
    bool pinWrite(uint8_t pin, bool level);

    // Interrupts
    bool getIntFlags(uint16_t* flags);
    bool getIntCapture(uint16_t* bits);

    // Output latch
    bool portGetLatch(uint16_t * value);
    bool portSetLatch(uint16_t value);
    bool pinGetLatch(uint8_t pin, bool *level);
    bool pinSetLatch(uint8_t pin, bool level);

    // Port interrupts
    bool portSetInterrupt(uint16_t mask, mcp23017_gpio_intr_t intr);

    // If interrupts are detected, events will be posted to the event loop
    bool portOnInterrupt(bool useIntCap);
  private:
    i2c_port_t _numI2C = I2C_NUM_0; 
    uint8_t _addrI2C = 0;
    cb_gpio_change_t _callback = nullptr;

    bool read8(uint8_t reg, uint8_t* value);
    bool write8(uint8_t reg, uint8_t value);
    bool read16(uint8_t reg, uint16_t* value);
    bool write16(uint8_t reg, uint16_t value);
};

#ifdef __cplusplus
}
#endif

#endif // __RE_MCP23017_H__