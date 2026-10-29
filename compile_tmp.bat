@echo off
setlocal
set TOOLROOT=c:\Users\admin\Desktop\flix-master\flix-stm32\.pio_core\packages\toolchain-gccarmnoneeabi\bin
set FW=c:\Users\admin\Desktop\flix-master\flix-stm32\.pio_core\packages\framework-arduinoststm32
set CMSIS=c:\Users\admin\Desktop\flix-master\flix-stm32\.pio_core\packages\framework-cmsis
set OUT=c:\Users\admin\AppData\Local\Temp\compile.log
"%TOOLROOT%\arm-none-eabi-g++.exe" -o "%TEMP%\temp_out.o" -c ^
  -std=gnu++17 -fno-threadsafe-statics -fno-rtti -fno-exceptions -fno-use-cxa-atexit ^
  -Os -ffunction-sections -fdata-sections -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 ^
  -mfloat-abi=hard -nostdlib -DPLATFORMIO=60200 -DSTM32F4 -DSTM32F407xx -DSTM32F40_41xxx ^
  -DSTM32F4xx -DARDUINO=10808 -DARDUINO_ARCH_STM32 -DNDEBUG -DARDUINO_DISCO_F407VG ^
  -DBOARD_NAME=\"DISCO_F407VG\" -DHAL_UART_MODULE_ENABLED -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER ^
  -DEXTENDED_PIN_MODE -DVARIANT_H=\"variant_DISCO_F407VG.h\" -DVECT_TAB_OFFSET=0x0 ^
  -Ic:\Users\admin\Desktop\flix-master\flix-stm32\src ^
  -Ic:\Users\admin\Desktop\flix-master\flix-stm32\.pio_core\packages\framework-arduinoststm32\libraries\EEPROM\src ^
  -Ic:\Users\admin\Desktop\flix-master\flix-stm32\.pio\libdeps\stm32f407_vet6\SBUS2\src ^
  -Ic:\Users\admin\Desktop\flix-master\flix-stm32\.pio\libdeps\stm32f407_vet6\MAVLink ^
  -Ic:\Users\admin\Desktop\flix-master\flix-stm32\.pio\libdeps\stm32f407_vet6\FlixPeriph ^
  -I%FW%\cores\arduino ^
  -I"%FW%\variants\STM32F4xx\F407V(E-G)T_F417V(E-G)T" ^
  -I%CMSIS%\CMSIS\Core\Include ^
  "%TEMP%\tmpi7r1_6_g" 2>&1 > "%OUT%"
type "%OUT%" | findstr /C:"error:" /C:"note:" /C:"warning:"
endlocal
