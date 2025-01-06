Supported color format:
a8
rgb565
rgb888
argb8888
rgba8888



# bin2bmp.py example:
# Show rgb565 454x454 framebuffer in hcpu_ram.bin at offset 0x2000
python bin2bmp.py hcpu_ram.bin rgb565 454 454 0x2000





# jlinkbin2bmp.py example:
# Connect target device by 12MHz SWD£¬and show rgb565 454x454 framebuffer at 0x20027610
python jlinkbin2bmp.py "-if SWD -speed 12000" rgb565 454 454 0x20027610


# Connect target device by 1MHz remote SWD(on localhost 19025 port)£¬and show rgb565 454x454 framebuffer at 0x20027610
python jlinkbin2bmp.py "-ip 127.0.0.1:19025 -if SWD -speed 1000" rgb565 454 454 0x20027610
