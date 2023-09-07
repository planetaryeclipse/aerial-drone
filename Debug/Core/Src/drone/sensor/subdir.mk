################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/drone/sensor/imu_sens.c \
../Core/Src/drone/sensor/open_logger.c \
../Core/Src/drone/sensor/pressr_sens.c 

OBJS += \
./Core/Src/drone/sensor/imu_sens.o \
./Core/Src/drone/sensor/open_logger.o \
./Core/Src/drone/sensor/pressr_sens.o 

C_DEPS += \
./Core/Src/drone/sensor/imu_sens.d \
./Core/Src/drone/sensor/open_logger.d \
./Core/Src/drone/sensor/pressr_sens.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/drone/sensor/%.o Core/Src/drone/sensor/%.su Core/Src/drone/sensor/%.cyclo: ../Core/Src/drone/sensor/%.c Core/Src/drone/sensor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/samuel/STM32CubeIDE/workspace_1.13.0/nucleo-f401re-vtol-drone/Libraries" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-drone-2f-sensor

clean-Core-2f-Src-2f-drone-2f-sensor:
	-$(RM) ./Core/Src/drone/sensor/imu_sens.cyclo ./Core/Src/drone/sensor/imu_sens.d ./Core/Src/drone/sensor/imu_sens.o ./Core/Src/drone/sensor/imu_sens.su ./Core/Src/drone/sensor/open_logger.cyclo ./Core/Src/drone/sensor/open_logger.d ./Core/Src/drone/sensor/open_logger.o ./Core/Src/drone/sensor/open_logger.su ./Core/Src/drone/sensor/pressr_sens.cyclo ./Core/Src/drone/sensor/pressr_sens.d ./Core/Src/drone/sensor/pressr_sens.o ./Core/Src/drone/sensor/pressr_sens.su

.PHONY: clean-Core-2f-Src-2f-drone-2f-sensor

