#pragma once

#include "pin.h"
#include "adc.h"
#include "flash.h"
#include "timers.h"
#include "modbus_slave.h"
#include "NTC_table.h"

struct In_regs {
   
   UART::Settings uart_set;         // 0
   uint16_t modbus_address;         // 1
   uint16_t password;               // 2
   uint16_t factory_number;         // 3
}__attribute__((packed));

   struct Out_regs {

   uint16_t uv_level;                // 0
   uint16_t temperature;             // 1
   // uint16_t device_code;            // 0
   // uint16_t factory_number;         // 1
   // UART::Settings uart_set;         // 2
   // uint16_t modbus_address;         // 3
}__attribute__((packed));

#define ADR(reg) GET_ADR(In_regs, reg)

constexpr auto conversion_on_channel {16};
struct ADC_{
   ADC_average& control     = ADC_average::make<mcu::Periph::ADC1>(conversion_on_channel);
   ADC_channel& uv_level    = control.add_channel<mcu::PA0>();
   ADC_channel& temperature = control.add_channel<mcu::PA1>();
};

template<class Flash_data, class Modbus>
class Sensor
{
   ADC_& adc;
   Modbus& modbus;
   Timer refresh{1_s};
   Flash_data& flash;
   uint16_t temperature{0};

   const size_t U = 33;
   const size_t R = 5100;

   void temp (uint16_t adc) {
      adc = adc / conversion_on_channel;
      auto p = std::lower_bound(
         std::begin(NTC::u2904<33,5100>),
         std::end(NTC::u2904<33,5100>),
         adc,
         std::greater<uint32_t>());
      temperature = (p - NTC::u2904<33,5100>);
   }
public:

   Sensor (ADC_& adc, Modbus& modbus, Flash_data& flash) 
      : adc    {adc}
      , modbus {modbus}
      , flash  {flash}
      {
         adc.control.start();
      }

   
   void operator() () {

      temp(adc.temperature);
      if (refresh.event())
         modbus.outRegs.uv_level = adc.uv_level;
      modbus.outRegs.temperature = temperature;
      
      modbus([&](auto registr){
         static bool unblock = false;
         switch (registr) {
            case ADR(uart_set):
               flash.uart_set
                  // = modbus.outRegs.uart_set
                  = modbus.inRegs.uart_set;
            break;
            case ADR(modbus_address):
               flash.modbus_address 
                  // = modbus.outRegs.modbus_address
                  = modbus.inRegs.modbus_address;
            break;
            case ADR(password):
               unblock = modbus.inRegs.password == 208;
            break;
            case ADR(factory_number):
               if (unblock) {
                  unblock = false;
                  flash.factory_number 
                     // = modbus.outRegs.factory_number
                     = modbus.inRegs.factory_number;
               }
               unblock = true;
            break;
         } // switch
      }); // modbus([&](auto registr)
   }
   
};