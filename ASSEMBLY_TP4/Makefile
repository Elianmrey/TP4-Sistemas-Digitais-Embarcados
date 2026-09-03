TARGET ?= tp4_program
AS := aarch64-linux-gnu-as
LD := aarch64-linux-gnu-ld
ASFLAGS ?=
LDFLAGS ?= -e _start
SRC := src
OBJ := build
LEGACY_OBJS := $(OBJ)/main.o $(OBJ)/sub_pot.o $(OBJ)/sub_lm35.o $(OBJ)/sub_bmp180.o $(OBJ)/sub_motor.o
TP4_OBJS := $(OBJ)/tp4_numeric.o $(OBJ)/tp4_neon.o
TEST_TARGET := $(OBJ)/test_tp4_numeric
TEST_OBJS := $(OBJ)/test_tp4_numeric.o $(TP4_OBJS)

all: $(TARGET) tests

$(TARGET): $(LEGACY_OBJS) $(TP4_OBJS)
	$(LD) $(LDFLAGS) -o $@ $^

$(OBJ):
	mkdir -p $(OBJ)

$(OBJ)/%.o: $(SRC)/%.s $(SRC)/common.inc | $(OBJ)
	$(AS) $(ASFLAGS) -I $(SRC) -o $@ $<

$(OBJ)/tp4_neon.o: $(SRC)/neon/tp4_neon.s | $(OBJ)
	$(AS) $(ASFLAGS) -o $@ $<

$(TEST_TARGET): $(TEST_OBJS)
	$(LD) -e _start -o $@ $^

$(OBJ)/test_tp4_numeric.o: tests/test_tp4_numeric.s | $(OBJ)
	$(AS) $(ASFLAGS) -o $@ $<

tests: $(TEST_TARGET)
	@if command -v qemu-aarch64 >/dev/null 2>&1; then \
		qemu-aarch64 $(TEST_TARGET); \
	elif [ "$$(uname -m)" = "aarch64" ]; then \
		$(TEST_TARGET); \
	else \
		echo "Testes assembly montados: execute $(TEST_TARGET) em AArch64 ou com qemu-aarch64"; \
	fi

clean:
	rm -rf $(OBJ) $(TARGET)

.PHONY: all clean tests
