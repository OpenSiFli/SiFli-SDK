# Makefile for building synax
# (c) 2017 Sayantan, Nilangshu

CFLAGS = -Wall -Wreturn-type -Werror -std=c11
DBG_FLAGS := -g -g3 -O0 -DENABLE_DEBUG
REL_FLAGS := -O2
LDFLAGS := -lzip -lxml2

EXEC := epb_test
BUILD_DIR := build
INC_DIR := . -I/usr/include/libxml2/
SRC_DIR := src
SRCS := $(wildcard src/*.c)
OBJS := $(patsubst %.c, $(BUILD_DIR)/%.o, $(notdir $(SRCS)))

.PHONY: all release debug link clean docs clean-docs

all: $(BUILD_DIR) debug

docs:
	$(info Generating documentation)
	@doxygen docs/Doxyfile

clean-docs:
	@if [ ! -d "./docs/html" ]; then echo "No docs exist"; else rm -r ./docs/html; fi

$(BUILD_DIR):
	mkdir $(BUILD_DIR)

debug: CFLAGS += $(DBG_FLAGS)
debug: link

release: CFLAGS += $(REL_FLAGS)
release: link

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(info Building objects)
	$(CC) -c $< $(CFLAGS) -I$(INC_DIR) -o $@

link: $(OBJS)
	$(info Linking objects)
	$(CC) $(OBJS) $(CFLAGS) $(LDFLAGS) -o $(BUILD_DIR)/$(EXEC)

clean:
	@echo "Cleaning build files"
	@if [ ! -d "./build/" ]; then echo "Already clean"; else rm -r ./build/; fi
