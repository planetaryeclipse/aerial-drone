################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drone/src/cntrl_actuators.c \
../Drone/src/dist_sens.c \
../Drone/src/gps.c \
../Drone/src/imu.c \
../Drone/src/open_logger.c \
../Drone/src/pressr_sens.c \
../Drone/src/radio.c \
../Drone/src/time_util.c \
../Drone/src/usr_interrupts.c 

OBJS += \
./Drone/src/cntrl_actuators.o \
./Drone/src/dist_sens.o \
./Drone/src/gps.o \
./Drone/src/imu.o \
./Drone/src/open_logger.o \
./Drone/src/pressr_sens.o \
./Drone/src/radio.o \
./Drone/src/time_util.o \
./Drone/src/usr_interrupts.o 

C_DEPS += \
./Drone/src/cntrl_actuators.d \
./Drone/src/dist_sens.d \
./Drone/src/gps.d \
./Drone/src/imu.d \
./Drone/src/open_logger.d \
./Drone/src/pressr_sens.d \
./Drone/src/radio.d \
./Drone/src/time_util.d \
./Drone/src/usr_interrupts.d 


# Each subdirectory must supply rules for building sources it contributes
Drone/src/%.o Drone/src/%.su Drone/src/%.cyclo: ../Drone/src/%.c Drone/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -DARM_MATH_CM4 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/samuel/STM32CubeIDE/workspace_1.13.0/nucleo-f401re-aerial-drone/Libraries" -I"/home/samuel/STM32CubeIDE/workspace_1.13.0/nucleo-f401re-aerial-drone/Drone/include" -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core_A/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drone-2f-src

clean-Drone-2f-src:
	-$(RM) ./Drone/src/cntrl_actuators.cyclo ./Drone/src/cntrl_actuators.d ./Drone/src/cntrl_actuators.o ./Drone/src/cntrl_actuators.su ./Drone/src/dist_sens.cyclo ./Drone/src/dist_sens.d ./Drone/src/dist_sens.o ./Drone/src/dist_sens.su ./Drone/src/gps.cyclo ./Drone/src/gps.d ./Drone/src/gps.o ./Drone/src/gps.su ./Drone/src/imu.cyclo ./Drone/src/imu.d ./Drone/src/imu.o ./Drone/src/imu.su ./Drone/src/open_logger.cyclo ./Drone/src/open_logger.d ./Drone/src/open_logger.o ./Drone/src/open_logger.su ./Drone/src/pressr_sens.cyclo ./Drone/src/pressr_sens.d ./Drone/src/pressr_sens.o ./Drone/src/pressr_sens.su ./Drone/src/radio.cyclo ./Drone/src/radio.d ./Drone/src/radio.o ./Drone/src/radio.su ./Drone/src/time_util.cyclo ./Drone/src/time_util.d ./Drone/src/time_util.o ./Drone/src/time_util.su ./Drone/src/usr_interrupts.cyclo ./Drone/src/usr_interrupts.d ./Drone/src/usr_interrupts.o ./Drone/src/usr_interrupts.su

.PHONY: clean-Drone-2f-src

