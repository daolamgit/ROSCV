################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../boundingBox.cpp \
../captureBackground.cpp \
../readTransform.cpp \
../simpleSubscriber.cpp \
../simplecanny.cpp \
../subtractBackground.cpp 

OBJS += \
./boundingBox.o \
./captureBackground.o \
./readTransform.o \
./simpleSubscriber.o \
./simplecanny.o \
./subtractBackground.o 

CPP_DEPS += \
./boundingBox.d \
./captureBackground.d \
./readTransform.d \
./simpleSubscriber.d \
./simplecanny.d \
./subtractBackground.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


