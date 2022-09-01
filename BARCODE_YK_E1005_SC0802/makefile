# Target file name (without extension)
TARGET=BARCODE_YK_E1005_SC0802

# Define commands
CC=C:\Program Files\GNU Tools ARM Embedded\gcc-arm-none-eabi-10.3-2021.07\bin\arm-none-eabi-gcc
OBJCOPY=C:\Program Files\GNU Tools ARM Embedded\gcc-arm-none-eabi-10.3-2021.07\bin\arm-none-eabi-objcopy
SIZE = C:\Program Files\GNU Tools ARM Embedded\gcc-arm-none-eabi-10.3-2021.07\bin\arm-none-eabi-size
STLINK=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe
LDSCRIPT=standalone.ld
REMOVE=	rm -f -r
COPY= cp -f
MOVE= mv -f
REMOVEDIR = rmdir
MAKEDIR= mkdir

LINKER_FLAGS=-nostartfiles -Xlinker -o$(TARGET).elf -Xlinker -M -Xlinker -Map=$(TARGET).map

DEBUG=-g
OPTIM=-O0
M_CPU=cortex-m3

CFLAGS=$(DEBUG) -I. \
	-I ../Libraries/CMSIS/Include \
	-I ../Libraries/SPL/inc \
	-mthumb \
	-mcpu=$(M_CPU) $(OPTIM) \
	-D STM32F10X_MD \
	-T $(LDSCRIPT) -ggdb

SOURCE=	main.c \
		startup.c \
		fifo.c \
		../Libraries/CMSIS/Source/system_stm32f10x.c \
		../Libraries/SPL/src/misc.c \
		../Libraries/SPL/src/stm32f10x_rcc.c \
		../Libraries/SPL/src/stm32f10x_gpio.c \
		../Libraries/SPL/src/stm32f10x_i2c.c \
		../Libraries/SPL/src/stm32f10x_usart.c \
		../Libraries/SPL/src/stm32f10x_spi.c \

OBJS = $(SOURCE:.c=.o)

all: begin Debug $(TARGET).bin size finished move end
$(TARGET).bin : $(TARGET).elf
	$(OBJCOPY) $(TARGET).elf -O ihex $(TARGET).hex
$(TARGET).elf : $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LINKER_FLAGS)
$(OBJS) : %.o : %.c
	$(CC) -c $(CFLAGS) $< -o $@
size:
	 $(SIZE) --format=berkeley $(TARGET).elf

# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_MOVE = ------  Moving output files into Debug folder  ------
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
begin:
	@echo
	@echo $(MSG_BEGIN)
finished:
	@echo $(MSG_ERRORS_NONE)
move: Debug
	@echo
	@echo $(MSG_MOVE)
	@echo
	$(MOVE) $(OBJS) Debug
	$(MOVE) $(TARGET).elf Debug
	$(MOVE) $(TARGET).hex Debug
	$(MOVE) $(TARGET).map Debug
end:
	@echo $(MSG_END)
	@echo
Debug:
	$(MAKEDIR) Debug
clean : begin clean_list end
clean_list :
	@echo $(MSG_CLEANING)
	$(REMOVE) $(OBJS)
	$(REMOVE) $(TARGET).hex
	$(REMOVE) $(TARGET).bin
	$(REMOVE) $(TARGET).elf
	$(REMOVE) $(TARGET).map
	$(REMOVE) Debug
download : begin program end
program:
	$(STLINK) -p Debug/$(TARGET).hex -run
reset:
	$(STLINK) -rst	