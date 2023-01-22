#define STM32F030x6
#define F_CPU   48000000UL

#include "periph_rcc.h"
#include "init_clock.h"
#include "sensor.h"

/// эта функция вызывается первой в startup файле
extern "C" void init_clock () { init_clock<8_MHz,F_CPU>(); }

using UV = mcu::PA0; 
using T  = mcu::PA1; 

using FAC = mcu::PB6;

// using A0 = mcu::PB3;
// using A1 = mcu::PB4;
// using A2 = mcu::PB5;
// using A3 = mcu::PB6;

using TX  = mcu::PA9;
using RX  = mcu::PA10;
using RTS = mcu::PA8;

int main()
{
   struct Flash_data {
      UART::Settings uart_set = {
         .parity_enable  = false,
         .parity         = USART::Parity::even,
         .data_bits      = USART::DataBits::_8,
         .stop_bits      = USART::StopBits::_1,
         .baudrate       = USART::Baudrate::BR9600,
         .res            = 0
      };
      uint8_t address   = 10;
      uint8_t baudrate  = 0;
      uint8_t parity    = 0; 
      uint8_t stop_bits = 0;
      uint8_t version   = 10;
      uint16_t uv_level_max = 0x0100;
   }flash;

   [[maybe_unused]] auto _ = Flash_updater<
        mcu::FLASH::Sector::_31
      , mcu::FLASH::Sector::_30
   >::make (&flash);

   ADC_ adc;

   uint8_t address{10};
   UART::Settings uart_set = {
      .parity_enable  = false,
      .parity         = USART::Parity::even,
      .data_bits      = USART::DataBits::_8,
      .stop_bits      = USART::StopBits::_1,
      .baudrate       = USART::Baudrate::BR9600,
      .res            = 0
   };

   auto& factory = Pin::make<FAC, mcu::PinMode::Input>();

   if (not factory) {
     address = flash.address;
     uart_set = flash.uart_set;
   }

   volatile decltype(auto) modbus = Modbus_slave<In_regs, Out_regs>
                 ::make<mcu::Periph::USART1, TX, RX, RTS>
                       (address, uart_set);

   using Flash  = decltype(flash);
   using Modbus = Modbus_slave<In_regs, Out_regs>;
   Sensor<Flash, Modbus> sensor {adc, modbus, flash, factory};

   modbus.inRegsMin.address  = 1;
   modbus.inRegsMax.address  = 247;
   modbus.inRegsMin.baudrate  = 0;
   modbus.inRegsMax.baudrate  = 7;
   modbus.inRegsMin.parity  = 0;
   modbus.inRegsMax.parity  = 2;
   modbus.inRegsMin.stop_bits  = 0;
   modbus.inRegsMax.stop_bits  = 1;
   modbus.inRegsMin.reset_max  = 0x524D;
   modbus.inRegsMax.reset_max  = 0x524D;

   modbus.outRegs.address   = flash.address;
   modbus.outRegs.baudrate  = flash.baudrate;
   modbus.outRegs.parity    = flash.parity;
   modbus.outRegs.stop_bits = flash.stop_bits;
   modbus.outRegs.version   = flash.version;


   // volatile uint16_t temperature{0};
   // auto temp = [&](uint16_t adc) {
   //    adc = adc / conversion_on_channel;
   //    auto p = std::lower_bound(
   //       std::begin(NTC::u2904<U,R>),
   //       std::end(NTC::u2904<U,R>),
   //       adc,
   //       std::greater<uint32_t>());
   //    temperature = (p - NTC::u2904<U,R>);
   // };

   // auto adress{0};
   // auto[a0, a1, a2, a3] = make_pins<mcu::PinMode::Input, A0, A1, A2, A3>();
   // std::array pins {a0, a1, a2, a3};

   while(1){

      // for (size_t i{0}; i < pins.size(); i++) {
      //    adress |= pins[i] ? 1 << i : 0 << i;
      // }
      // flash.modbus_address = flash.modbus_address != adress ? adress : flash.modbus_address;
      sensor();
      // __WFI();
   }

}



