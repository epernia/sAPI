include config.mk

SRC=$(wildcard app/src/*.c) $(foreach m, $(MODULES), $(wildcard libs/$(m)/src/*.c))
INCLUDES=$(foreach m, $(MODULES), -Ilibs/$(m)/inc) -Iapp/inc
_DEFINES=$(foreach m, $(DEFINES), -D$(m))
OUT=out
OBJECTS=$(SRC:%.c=$(OUT)/%.o)
DEPS=$(OBJECTS:%.o=%.d)
LDSCRIPT=libs/ciaa_lpc4337.ld
OOCD_SCRIPT=libs/ciaa-nxp.cfg

TARGET=$(OUT)/$(APP).elf
TARGET_BIN=$(basename $(TARGET)).bin
TARGET_LST=$(basename $(TARGET)).lst
TARGET_MAP=$(basename $(TARGET)).map

ARCH_FLAGS=-mcpu=cortex-m4 -mthumb

ifeq ($(USE_FPU),y)
ARCH_FLAGS+=-mfloat-abi=hard -mfpu=fpv4-sp-d16
endif

CFLAGS=$(ARCH_FLAGS) $(INCLUDES) $(_DEFINES) -ggdb3 -O$(OPT) -ffunction-sections -fdata-sections
LDFLAGS=$(ARCH_FLAGS) -T$(LDSCRIPT) -nostartfiles -Wl,-gc-sections -Wl,-Map=$(TARGET_MAP) -Wl,--cref

ifeq ($(USE_NANO),y)
LDFLAGS+=--specs=nano.specs
endif

ifeq ($(SEMIHOST),y)
LDFLAGS+=--specs=rdimon.specs
endif

CROSS=arm-none-eabi-
CC=$(CROSS)gcc
LD=$(CROSS)gcc
SIZE=$(CROSS)size
LIST=$(CROSS)objdump -xdS
OBJCOPY=$(CROSS)objcopy
GDB=$(CROSS)gdb
OOCD=openocd

ifeq ($(VERBOSE),y)
Q=
else
Q=@
endif

Compilar_proyecto: $(TARGET) $(TARGET_BIN) $(TARGET_LST) Uso_de_memoria

-include $(DEPS)

$(OUT)/%.o: %.c
	@echo Compilando $(notdir $<)
	@mkdir -p $(dir $@)
	$(Q)$(CC) -MMD $(CFLAGS) -c -o $@ $<

$(TARGET): $(OBJECTS)
	@echo Enlazando $@...
	$(Q)$(LD) $(LDFLAGS) -o $@ $(OBJECTS)

$(TARGET_BIN): $(TARGET)
	@echo Generando archivo binario...
	$(Q)$(OBJCOPY) -v -O binary $< $@

$(TARGET_LST): $(TARGET)
	@echo Generando desensamblado...
	$(Q)$(LIST) $< > $@

Uso_de_memoria: $(TARGET)
	@echo Calculando uso de memoria
	$(Q)$(SIZE) $<
	@echo  
	@echo Listo.

Grabar_proyecto_en_flash: $(TARGET_BIN)
	@echo Programando...
	$(Q)$(OOCD) -f $(OOCD_SCRIPT) \
		-c "init" \
		-c "halt 0" \
		-c "flash write_image erase unlock $< 0x1A000000 bin" \
		-c "reset run" \
		-c "shutdown" 2>&1
	@echo  
	@echo Listo.
	@echo En caso de que el programa no se ejecute directamente resetee la placa.

Borrar_memoria_flash:
	@echo Borrando memoria...
	$(Q)$(OOCD) -f $(OOCD_SCRIPT) \
		-c "init" \
		-c "halt 0" \
		-c "flash erase_sector 0 0 last" \
		-c "shutdown" 2>&1
	@echo  
	@echo Listo.
	@echo Por favor resetee la placa.

Limpiar_Proyecto:
	@echo Limpiando el proyecto...
	$(Q)rm -fR $(OBJECTS) $(TARGET) $(TARGET_BIN) $(TARGET_LST) $(DEPS) $(OUT)
	@echo  
	@echo Listo.

.PHONY: Compilar_proyecto Uso_de_memoria Limpiar_Proyecto Grabar_proyecto_en_flash Borrar_memoria_flash
