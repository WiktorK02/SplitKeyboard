################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/Core/Src/usbd_core.c \
../Middlewares/ST/Core/Src/usbd_ctlreq.c \
../Middlewares/ST/Core/Src/usbd_ioreq.c 

OBJS += \
./Middlewares/ST/Core/Src/usbd_core.o \
./Middlewares/ST/Core/Src/usbd_ctlreq.o \
./Middlewares/ST/Core/Src/usbd_ioreq.o 

C_DEPS += \
./Middlewares/ST/Core/Src/usbd_core.d \
./Middlewares/ST/Core/Src/usbd_ctlreq.d \
./Middlewares/ST/Core/Src/usbd_ioreq.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/Core/Src/%.o Middlewares/ST/Core/Src/%.su Middlewares/ST/Core/Src/%.cyclo: ../Middlewares/ST/Core/Src/%.c Middlewares/ST/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../STM32_WPAN/App -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Utilities/lpm/tiny_lpm -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/core -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Utilities/sequencer -I../Middlewares/ST/STM32_WPAN/ble -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-Core-2f-Src

clean-Middlewares-2f-ST-2f-Core-2f-Src:
	-$(RM) ./Middlewares/ST/Core/Src/usbd_core.cyclo ./Middlewares/ST/Core/Src/usbd_core.d ./Middlewares/ST/Core/Src/usbd_core.o ./Middlewares/ST/Core/Src/usbd_core.su ./Middlewares/ST/Core/Src/usbd_ctlreq.cyclo ./Middlewares/ST/Core/Src/usbd_ctlreq.d ./Middlewares/ST/Core/Src/usbd_ctlreq.o ./Middlewares/ST/Core/Src/usbd_ctlreq.su ./Middlewares/ST/Core/Src/usbd_ioreq.cyclo ./Middlewares/ST/Core/Src/usbd_ioreq.d ./Middlewares/ST/Core/Src/usbd_ioreq.o ./Middlewares/ST/Core/Src/usbd_ioreq.su

.PHONY: clean-Middlewares-2f-ST-2f-Core-2f-Src

