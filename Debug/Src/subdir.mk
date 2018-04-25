################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/l3gd20.c \
../Src/lsm303dlhc.c \
../Src/main.c \
../Src/mpu9250.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c \
../Src/usb_device.c \
../Src/usbd_cdc_if.c \
../Src/usbd_conf.c \
../Src/usbd_desc.c 

OBJS += \
./Src/l3gd20.o \
./Src/lsm303dlhc.o \
./Src/main.o \
./Src/mpu9250.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o \
./Src/usb_device.o \
./Src/usbd_cdc_if.o \
./Src/usbd_conf.o \
./Src/usbd_desc.o 

C_DEPS += \
./Src/l3gd20.d \
./Src/lsm303dlhc.d \
./Src/main.d \
./Src/mpu9250.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d \
./Src/usb_device.d \
./Src/usbd_cdc_if.d \
./Src/usbd_conf.d \
./Src/usbd_desc.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/EclipseOxygen/MGR/MGR/Inc" -I"D:/EclipseOxygen/MGR/MGR/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/EclipseOxygen/MGR/MGR/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/EclipseOxygen/MGR/MGR/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"D:/EclipseOxygen/MGR/MGR/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"D:/EclipseOxygen/MGR/MGR/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/EclipseOxygen/MGR/MGR/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


