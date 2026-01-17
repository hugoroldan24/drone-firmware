# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# ============================================================================= #
# 			      Project Makefile												#
# ============================================================================= #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# USB port for programming
PORT        = /dev/ttyUSB0

# TX build output directory (relative to this Makefile)
TX_ROUTE    = build/tx

# RX build output directory (relative to this Makefile)
RX_ROUTE    = build/rx

# Path to the TX source files
TX_SRC_ROUTE = transmitter/

# Path to the RX source files
RX_SRC_ROUTE = receiver/

# Path to the common source files
COMMON_ROUTE = include/common_files/

# Path to header files
HEADERS_ROUTE = include/headers

# Target MCU and clock frequency
MCU         = atmega328p
F_CPU       = 16000000UL

# Toolchain binaries
CC          = avr-gcc
OBJCOPY     = avr-objcopy
AVRDUDE     = avrdude

# Optimization level and C standard
OPT         = -Os
CSTD        = -std=c99

# Compiler flags for compiling .c → .o
#   -mmcu: selects the device and linker script
#   -DF_CPU: defines CPU clock for <util/delay.h>
#   -I:    header search path
#   -Wall: enable all warnings
CFLAGS      = -mmcu=$(MCU) -DF_CPU=$(F_CPU) $(OPT) -I $(HEADERS_ROUTE) -Wall $(CSTD)

# Linker flags for linking .o → .elf
LFLAGS      = -mmcu=$(MCU)

# Output filenames (without extension)
TX_OUTPUT   = program
RX_OUTPUT   = program


# ==============================================================================
# Source Files
# ==============================================================================

# TX source files
TX_SRC      = $(TX_SRC_ROUTE)main.c \
               $(TX_SRC_ROUTE)radio_transmitter.c \
               $(TX_SRC_ROUTE)adc.c \
               $(TX_SRC_ROUTE)transmitter.c \
			   $(TX_SRC_ROUTE)hc06.c \
			   $(TX_SRC_ROUTE)tx_tasks.c \
               $(COMMON_ROUTE)spi.c \
			   $(COMMON_ROUTE)circular_queue.c \
			   $(COMMON_ROUTE)scheduler.c \
               $(COMMON_ROUTE)usart.c

# RX source files
RX_SRC      = $(RX_SRC_ROUTE)main.c \
               $(RX_SRC_ROUTE)radio_receiver.c \
               $(RX_SRC_ROUTE)pwm.c \
               $(RX_SRC_ROUTE)receiver.c \
               $(COMMON_ROUTE)spi.c \
			   $(COMMON_ROUTE)circular_queue.c \
			   $(COMMON_ROUTE)scheduler.c \
               $(COMMON_ROUTE)usart.c


# Generate object filenames by stripping path (notdir) and changing .c → .o, then prefixing (addprefix) with build dirs
TX_OBJ      = $(addprefix $(TX_ROUTE)/, $(notdir $(TX_SRC:.c=.o)))
RX_OBJ      = $(addprefix $(RX_ROUTE)/, $(notdir $(RX_SRC:.c=.o)))


.PHONY: all transmitter receiver upload-tx upload-rx clean

all: transmitter receiver


# ==============================================================================
# Transmitter Build
# ==============================================================================

# Top-level rule for transmitter: build the .hex
transmitter: $(TX_ROUTE)/$(TX_OUTPUT).hex

# ------------------------------------------------------------------------------
# Compile TX .c files
# ------------------------------------------------------------------------------
$(TX_ROUTE)/%.o: $(TX_SRC_ROUTE)%.c
	@echo "  [CC]       Compiling $(notdir $<)"
	@$(CC) $(CFLAGS) -c $< -o $@

# ------------------------------------------------------------------------------
# Compile shared/common .c files for TX
# ------------------------------------------------------------------------------
$(TX_ROUTE)/%.o: $(COMMON_ROUTE)%.c
	@echo "  [CC]       Compiling $(notdir $<)"
	@$(CC) $(CFLAGS) -c $< -o $@

# ------------------------------------------------------------------------------
# Link TX object files into .elf
# ------------------------------------------------------------------------------
$(TX_ROUTE)/$(TX_OUTPUT).elf: $(TX_OBJ)
	@echo "  [LD]       Linking to $(TX_OUTPUT).elf"
	@$(CC) $(LFLAGS) $^ -o $@

# ------------------------------------------------------------------------------
# Convert TX .elf to .hex
# ------------------------------------------------------------------------------
$(TX_ROUTE)/$(TX_OUTPUT).hex: $(TX_ROUTE)/$(TX_OUTPUT).elf
	@echo "  [OBJCOPY]  Generating $(notdir $@)"
	@$(OBJCOPY) -O ihex -R .eeprom $< $@
	@echo "  [DONE]     TX build complete."
	@echo ""


# ==============================================================================
# Receiver Build
# ==============================================================================

# Top-level rule for receiver: build the .hex
receiver: $(RX_ROUTE)/$(RX_OUTPUT).hex

# ------------------------------------------------------------------------------
# Compile RX .c files
# ------------------------------------------------------------------------------
$(RX_ROUTE)/%.o: $(RX_SRC_ROUTE)%.c
	@echo "  [CC]       Compiling $(notdir $<)"
	@$(CC) $(CFLAGS) -c $< -o $@

# ------------------------------------------------------------------------------
# Compile shared/common .c files for RX
# ------------------------------------------------------------------------------
$(RX_ROUTE)/%.o: $(COMMON_ROUTE)%.c
	@echo "  [CC]       Compiling $(notdir $<)"
	@$(CC) $(CFLAGS) -c $< -o $@

# ------------------------------------------------------------------------------
# Link RX object files into .elf
# ------------------------------------------------------------------------------
$(RX_ROUTE)/$(RX_OUTPUT).elf: $(RX_OBJ)
	@echo "  [LD]       Linking to $(RX_OUTPUT).elf"
	@$(CC) $(LFLAGS) $^ -o $@

# ------------------------------------------------------------------------------
# Convert RX .elf to .hex
# ------------------------------------------------------------------------------
$(RX_ROUTE)/$(RX_OUTPUT).hex: $(RX_ROUTE)/$(RX_OUTPUT).elf
	@echo "  [OBJCOPY]  Generating $(notdir $@)"
	@$(OBJCOPY) -O ihex -R .eeprom $< $@
	@echo "  [DONE]     RX build complete."


# ==============================================================================
# Upload Targets
# ==============================================================================

# Upload TX firmware to MCU
upload-tx: $(TX_ROUTE)/$(TX_OUTPUT).hex
	@echo "  [UPLOAD]  Flashing TX firmware"
	@$(AVRDUDE) -c arduino -p $(MCU) -P $(PORT) -b 115200 -U flash:w:$<
	@echo "  [DONE]    TX flash complete."

# Upload RX firmware to MCU
upload-rx: $(RX_ROUTE)/$(RX_OUTPUT).hex
	@echo "  [UPLOAD]  Flashing RX firmware"
	@$(AVRDUDE) -c arduino -p $(MCU) -P $(PORT) -b 115200 -U flash:w:$<
	@echo "  [DONE]    RX flash complete."


# ==============================================================================
# Clean Build Artifacts
# ==============================================================================

clear:
	@echo "  [CLEAR]  Removing build artifacts"
	@rm -rf $(TX_ROUTE)/*.o $(TX_ROUTE)/*.elf $(TX_ROUTE)/*.hex
	@rm -rf $(RX_ROUTE)/*.o $(RX_ROUTE)/*.elf $(RX_ROUTE)/*.hex


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# ============================================================================= #
# 			      End of Makefile												#
# ============================================================================= #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
	

