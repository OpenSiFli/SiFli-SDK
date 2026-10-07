# Deliberately empty, like example/test/hal_mock/project/rtconfig.py.
#
# PrepareEnv() -> LoadRtconfig(board) loads the *board's* rtconfig.py
# (customer/boards/qemu_sf32lb52x/hcpu/rtconfig.py: TARGET_NAME, CHIP,
# CROSS_TOOL), so there is nothing for a project to add here. An earlier project
# spelled the chip identity out in the SConstruct instead and silently compiled
# the wrong chip's branches -- see the comment in ../SConstruct.
