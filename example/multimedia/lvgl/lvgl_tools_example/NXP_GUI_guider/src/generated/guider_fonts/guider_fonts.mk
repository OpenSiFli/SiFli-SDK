-include $(PRJ_DIR)/generated/guider_fonts/fonts_list.mk

DEPPATH += --dep-path $(PRJ_DIR)/generated/guider_fonts
VPATH += :$(PRJ_DIR)/generated/guider_fonts

CFLAGS += "-I$(PRJ_DIR)/generated/guider_fonts"