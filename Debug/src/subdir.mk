################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/boundingBox.cpp \
../src/captureBackground.cpp \
../src/readTransform.cpp \
../src/simpleSubscriber.cpp \
../src/simplecanny.cpp \
../src/subtractBackground.cpp 

OBJS += \
./src/boundingBox.o \
./src/captureBackground.o \
./src/readTransform.o \
./src/simpleSubscriber.o \
./src/simplecanny.o \
./src/subtractBackground.o 

CPP_DEPS += \
./src/boundingBox.d \
./src/captureBackground.d \
./src/readTransform.d \
./src/simpleSubscriber.d \
./src/simplecanny.d \
./src/subtractBackground.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/ros/groovy/include/opencv -I/opt/ros/groovy/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


