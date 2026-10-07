import os
import argparse

SIFLI_SDK=os.getenv('SIFLI_SDK')
TARGET_NAME = 'bf0_qemu'
WERROR = True
JLINK_DEVICE = ''
CORE = "HCPU"
# The chip being emulated, not a chip that is present. It is named here so that
# the build's chip-keyed behaviour (link script lookup, chip-specific proj.conf)
# agrees with the Kconfig identity selected in Kconfig.board -- two sources of
# truth disagreeing about the chip is how the original hand-injected defines
# produced errors that looked like header problems.
#
# Nothing is programmed: the QEMU boards carry no bootloader and no flash table,
# and building.py skips both for any SOC_QEMU build regardless of CHIP.
CHIP = 'SF32LB58X'
CROSS_TOOL='gcc'
