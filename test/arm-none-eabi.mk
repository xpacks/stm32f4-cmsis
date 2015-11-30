PREFIX=arm-none-eabi-
CC=$(PREFIX)gcc
CPP=$(PREFIX)g++

CFLAGS=-std=gnu11 -Og -fmessage-length=0 -fsigned-char -ffunction-sections \
-fdata-sections -fno-move-loop-invariants
CPPFLAGS=-std=gnu++11 -Og -fmessage-length=0 -fsigned-char -ffunction-sections \
-fdata-sections -fno-move-loop-invariants -fabi-version=0 -fno-exceptions \
-fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics

# Since GCC does not have an option to enable really all warnings,
# enumerate as many as possible.
WARNFLAGS=-Werror -Wall -Wextra -Wunused -Wuninitialized \
-Wmissing-declarations -Wconversion -Wpointer-arith \
-Wpadded -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal
CWARNFLAGS=-Wmissing-prototypes -Wstrict-prototypes -Wbad-function-cast
CPPWARNFLAGS=-Wabi -Wctor-dtor-privacy -Wnoexcept -Wnon-virtual-dtor \
-Wstrict-null-sentinel -Wsign-promo

CPU=-mcpu=cortex-m4 -mthumb -mfloat-abi=soft 

INCLUDES=-I"$(PARENT)/Drivers/CMSIS/Device/ST/STM32F4xx/Include" 
INCLUDES+=-I"$(PARENT)/test/include"
INCLUDES+=-I"$(PARENT)/../../arm/arm-cmsis-xpack/CMSIS/Include"

%.o: %.c
	$(CC) $(CPU) $(DEFINES) $(CFLAGS) $(WARNFLAGS) $(CWARNFLAGS) $(INCLUDES) -c -o "$@" "$<"

%.o: %.cpp
	$(CPP) $(CPU) $(DEFINES) $(CPPFLAGS) $(WARNFLAGS)  $(CPPWARNFLAGS) $(INCLUDES) -c -o "$@" "$<"
