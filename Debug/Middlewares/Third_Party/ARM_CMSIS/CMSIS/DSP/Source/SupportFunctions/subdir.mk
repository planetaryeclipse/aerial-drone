################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/SupportFunctions/SupportFunctions.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/SupportFunctions/SupportFunctionsF16.c 

OBJS += \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/SupportFunctions/SupportFunctions.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/SupportFunctions/SupportFunctionsF16.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/SupportFunctions/SupportFunctions.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/SupportFunctions/SupportFunctionsF16.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/SupportFunctions/%.o Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/SupportFunctions/%.su Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/SupportFunctions/%.cyclo: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/SupportFunctions/%.c Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/SupportFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -DARM_MATH_CM4 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/samuel/STM32CubeIDE/workspace_1.13.0/nucleo-f401re-aerial-drone/Libraries" -I"/home/samuel/STM32CubeIDE/workspace_1.13.0/nucleo-f401re-aerial-drone/Drone/include" -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core_A/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-CMSIS-2f-DSP-2f-Source-2f-SupportFunctions

clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-CMSIS-2f-DSP-2f-Source-2f-SupportFunctions:
	-$(RM) ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/SupportFunctions/SupportFunctions.cyclo ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/SupportFunctions/SupportFunctions.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/SupportFunctions/SupportFunctions.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/SupportFunctions/SupportFunctions.su ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/SupportFunctions/SupportFunctionsF16.cyclo ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/SupportFunctions/SupportFunctionsF16.d ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/SupportFunctions/SupportFunctionsF16.o ./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Source/SupportFunctions/SupportFunctionsF16.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-CMSIS-2f-DSP-2f-Source-2f-SupportFunctions

