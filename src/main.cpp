#define STM32F030x6
#define F_CPU   48000000UL

#include "periph_rcc.h"
#include "init_clock.h"
#include "sensor.h"

/// эта функция вызывается первой в startup файле
extern "C" void init_clock () { init_clock<8_MHz,F_CPU>(); }

using UV = mcu::PA0; 
using T  = mcu::PA1; 

using A0 = mcu::PB3;
using A1 = mcu::PB4;
using A2 = mcu::PB5;
using A3 = mcu::PB6;

using TX  = mcu::PA9;
using RX  = mcu::PA10;
using RTS = mcu::PA8;

int main()
{
   struct Flash_data {
      uint16_t factory_number = 0;
      UART::Settings uart_set = {
         .parity_enable  = false,
         .parity         = USART::Parity::even,
         .data_bits      = USART::DataBits::_8,
         .stop_bits      = USART::StopBits::_1,
         .baudrate       = USART::Baudrate::BR9600,
         .res            = 0
      };
      uint8_t  modbus_address = 10;
      uint16_t model_number   = 0;
   }flash;

   [[maybe_unused]] auto _ = Flash_updater<
        mcu::FLASH::Sector::_10
      , mcu::FLASH::Sector::_9
   >::make (&flash);

   ADC_ adc;

   volatile decltype(auto) modbus = Modbus_slave<In_regs, Out_regs>
                 ::make<mcu::Periph::USART1, TX, RX, RTS>
                       (flash.modbus_address, flash.uart_set);

   using Flash  = decltype(flash);
   using Modbus = Modbus_slave<In_regs, Out_regs>;
   Sensor<Flash, Modbus> sensor {adc, modbus, flash};

   // modbus.outRegs.device_code       = 12;
   // modbus.outRegs.factory_number    = flash.factory_number;
   // modbus.outRegs.modbus_address    = flash.modbus_address;
   // modbus.outRegs.uart_set          = flash.uart_set;
   modbus.arInRegsMax[ADR(uart_set)]= 0b11111111;
   modbus.inRegsMin.modbus_address  = 1;
   modbus.inRegsMax.modbus_address  = 255;


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
      __WFI();
   }

}



