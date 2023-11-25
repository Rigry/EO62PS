#pragma once

#include "pin.h"
#include "adc.h"
#include "flash.h"
#include "timers.h"
#include "modbus_slave.h"
#include "NTC_table.h"
#include "utils.h"

struct Hold_regs {
   
   uint16_t address;         // 0
   uint16_t baudrate;        // 1
   uint16_t parity;          // 2
   uint16_t stop_bits;       // 3
   uint16_t service_mode;    // 4 code = 0x534D
   uint16_t reset_max;       // 5 code = 0x524D or 21069
}__attribute__((packed));

   struct Input_regs {

   uint16_t uv_level;         // 0
   uint16_t temperature;      // 1
   uint16_t uv_level_percent; // 2
   uint16_t uv_level_max;     // 3
   uint16_t version_device;   // 4
   uint16_t version_lib;      // 5

}__attribute__((packed));

#define ADR(reg) GET_ADR(Hold_regs, reg)

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
   Flash_data& flash;
   Pin& factory;
   Timer refresh{1_s};
   uint16_t temperature{0};
   const uint16_t SERVICE_CODE {0x534D};
   const uint16_t RESET_CODE   {0x524D};

   bool service_mode{false};

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

   Sensor (ADC_& adc, Modbus& modbus, Flash_data& flash, Pin& factory) 
      : adc    {adc}
      , modbus {modbus}
      , flash  {flash}
      , factory{factory}
      {
         adc.control.start();
      }

   
   void operator() () {

      temp(adc.temperature);
      if (refresh.event()) {
         modbus.input_Regs.uv_level = adc.uv_level / conversion_on_channel;
         set_if_greater (&flash.uv_level_max, modbus.input_Regs.uv_level);
         modbus.input_Regs.uv_level_max = flash.uv_level_max;
         modbus.input_Regs.uv_level_percent = modbus.input_Regs.uv_level * 100 / flash.uv_level_max;
      }
      modbus.input_Regs.temperature = temperature;
      
      modbus([&](auto registr){
         switch (registr) {
            case ADR(address):
               if (factory and service_mode) {
                  flash.address 
                  = modbus.holdRegs.address;
               }
            break;
            case ADR(baudrate):
               if (factory and service_mode) {
                  flash.baudrate 
                     = modbus.holdRegs.baudrate;
                  if (modbus.holdRegs.baudrate == 0)
                     flash.uart_set.baudrate = USART::Baudrate::BR9600;
                  else if (modbus.holdRegs.baudrate == 1)
                     flash.uart_set.baudrate = USART::Baudrate::BR14400;
                  else if (modbus.holdRegs.baudrate == 2)
                     flash.uart_set.baudrate = USART::Baudrate::BR19200;
                  else if (modbus.holdRegs.baudrate == 3)
                     flash.uart_set.baudrate = USART::Baudrate::BR28800;
                  else if (modbus.holdRegs.baudrate == 4)
                     flash.uart_set.baudrate = USART::Baudrate::BR38400;
                  else if (modbus.holdRegs.baudrate == 5)
                     flash.uart_set.baudrate = USART::Baudrate::BR57600;
                  else if (modbus.holdRegs.baudrate == 6)
                     flash.uart_set.baudrate = USART::Baudrate::BR76800;
                  else if (modbus.holdRegs.baudrate == 7)
                     flash.uart_set.baudrate = USART::Baudrate::BR115200;
               }
            break;
            case ADR(parity):
               if (factory  and service_mode) {
                  flash.parity 
                     = modbus.holdRegs.parity;
                  if (modbus.holdRegs.parity == 0) {
                     flash.uart_set.parity_enable = false;
                     flash.uart_set.parity = USART::Parity::even;
                  } else if (modbus.holdRegs.parity == 1) {
                     flash.uart_set.parity_enable = true;
                     flash.uart_set.parity = USART::Parity::even;
                  } else if (modbus.holdRegs.parity == 2) {
                     flash.uart_set.parity_enable = true;
                     flash.uart_set.parity = USART::Parity::odd;
                  }
               }
            break;
            case ADR(stop_bits):
               if (factory  and service_mode) {
                  flash.stop_bits 
                     = modbus.holdRegs.stop_bits;
                  if (modbus.holdRegs.stop_bits == 0)
                     flash.uart_set.stop_bits = USART::StopBits::_1;
                  else if (modbus.holdRegs.stop_bits == 1)
                     flash.uart_set.stop_bits = USART::StopBits::_2;
               }
            break;
            case ADR(service_mode):
               if(modbus.holdRegs.service_mode == SERVICE_CODE) {
                  modbus.holdRegs.service_mode = SERVICE_CODE;
                  service_mode = true;
               } else if(modbus.holdRegs.service_mode == 0){
                  modbus.holdRegs.service_mode = 0;
                  service_mode = false;
               }
            break;
            case ADR(reset_max):
               if (modbus.holdRegs.reset_max == RESET_CODE and service_mode) {
                  flash.uv_level_max = 0;
                  modbus.holdRegs.reset_max = 0;
               } else {
                  modbus.holdRegs.reset_max = modbus.holdRegs.reset_max;
               }
            break;
            
         } // switch
      }, [&](auto registr){}); // modbus([&](auto registr)
   }
   
};