################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/CANopen/CO_app_STM32.c 

OBJS += \
./Core/Src/CANopen/CO_app_STM32.o 

C_DEPS += \
./Core/Src/CANopen/CO_app_STM32.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/CANopen/%.o Core/Src/CANopen/%.su Core/Src/CANopen/%.cyclo: ../Core/Src/CANopen/%.c Core/Src/CANopen/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Abdurrahman/Desktop/Yeni_klasor/STM32Pro/CANopen/CANopen" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-CANopen

clean-Core-2f-Src-2f-CANopen:
	-$(RM) ./Core/Src/CANopen/CO_app_STM32.cyclo ./Core/Src/CANopen/CO_app_STM32.d ./Core/Src/CANopen/CO_app_STM32.o ./Core/Src/CANopen/CO_app_STM32.su

.PHONY: clean-Core-2f-Src-2f-CANopen

