#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import sys
import struct
import random


from array import array


class bmp:
    """ bmp data structure """
 
    def __init__(self, w=1080, h=1920, color = 0xffffff):
        self.w = w
        self.h = h
        self.gen_bmp_header()
        self.paint_bgcolor(color)
 
    def calc_data_size (self):
        if((self.w*3)%4 == 0):
            self.dataSize = self.w * 3 * self.h
        else:
            self.dataSize = (((self.w * 3) // 4 + 1) * 4) * self.h
 
        self.fileSize = self.dataSize + 54
 
    def conv2byte(self, l, num, len):
        tmp = num
        for i in range(len):
            l.append(tmp & 0x000000ff)
            tmp >>= 8
 
    def gen_bmp_header (self):
        self.calc_data_size();
        self.bmp_header = [0x42, 0x4d]
        self.conv2byte(self.bmp_header, self.fileSize, 4) #file size
        self.conv2byte(self.bmp_header, 0, 2)
        self.conv2byte(self.bmp_header, 0, 2)
        self.conv2byte(self.bmp_header, 54, 4) #rgb data offset
        self.conv2byte(self.bmp_header, 40, 4) #info block size
        self.conv2byte(self.bmp_header, self.w, 4)
        self.conv2byte(self.bmp_header, self.h, 4)
        self.conv2byte(self.bmp_header, 1, 2)
        self.conv2byte(self.bmp_header, 24, 2) #888
        self.conv2byte(self.bmp_header, 0, 4)  #no compression
        self.conv2byte(self.bmp_header, self.dataSize, 4) #rgb data size
        self.conv2byte(self.bmp_header, 0, 4)
        self.conv2byte(self.bmp_header, 0, 4)
        self.conv2byte(self.bmp_header, 0, 4)
        self.conv2byte(self.bmp_header, 0, 4)
 
    def print_bmp_header (self):
        length = len(self.bmp_header)
        for i in range(length):
            print("{:0>2x}".format(self.bmp_header[i]), end=' ')
            if i%16 == 15:
                print('')
        print('')
 
    def paint_bgcolor(self, color=0xffffff):
        self.rgbData = []
        for r in range(self.h):
            self.rgbDataRow = []
            for c in range(self.w):
                self.rgbDataRow.append(color)
            self.rgbData.append(self.rgbDataRow)
 
    def paint_line(self, x1, y1, x2, y2, color):
        k = (y2 - y1) / (x2 - x1)
        for x in range(x1, x2+1):
            y = int(k * (x - x1) + y1)
            self.rgbData[y][x] = color
 
    def paint_rect(self, x1, y1, w, h, color):
        for x in range(x1, x1+w):
            for y in range(y1, y1+h):
                self.rgbData[y][x] = color
 
    def paint_point(self, x, y, color=0x000000):
        self.rgbData[y][x] = color

    def rand_c(self, v, range, mask):
        rand = random.randint(-range,range)
        if v + rand < 0:
            return 0
        elif v+rand > 255:
            return 255 & mask
        else:
            return (v + rand) & mask
#|          | bit31~bit25 | bit24~bit17 | bit16~bit8   | bit7~bit0   |
#| ------   | ------      | ------      | ------       | ------      |
#| RGB565   |    /        |    /        | R4~R0G5~G3   | G2~G0B4~B0  |
#| ARGB8565 |    /        | A7 ~ A0     | R4~R0G5~G3   | G2~G0B4~B0  |
#| RGB888   |    /        | R7 ~ R0     | G7 ~ G0      | B7 ~ B0     |
#| ARGB8888 | A7 ~ A0     | R7 ~ R0     | G7 ~ G0      | B7 ~ B0     |


    def bin_format_depth(self, bin_format):
        if bin_format == "a8":
            return 8
        elif bin_format == "a4":
            return 4
        elif bin_format == "a2":
            return 2
        elif bin_format == "rgb565" or bin_format == "bgr565":
            return 16
        elif bin_format == "rgb888" or bin_format == "argb8565":
            return 24
        elif bin_format == "argb8888" or bin_format == "rgba8888":
            return 32
        else:
            return 0


    #bin_format       - bin format
    #bin_array - bin array
    #offset   - bin offset to read
    def convert_bin2bmp(self, bin_format, bin_array, offset):
        index = offset
        bit_mask = 0
        bit_pos  = 0
        for y in range(self.h-1,-1,-1):

            if bin_format == "a4":
                bit_mask = 0x0F
            elif bin_format == "a2":
                bit_mask = 0x03
            
            bit_pos  = 0

            for x in range(0,self.w,1):
                a = 255

                if bin_format == "a8":
                    r = ord(bin_array[index])
                    g = r
                    b = r
                    index=index+1
                elif bin_format == "a4":
                    r = (ord(bin_array[index]) & bit_mask)>>bit_pos
                    #print ("x,y = %d,%d, src=%02x, mask=%02x,  v=%02x"%(x,y,ord(bin_array[index]),bit_mask,r))
                    
                    r = r << 4 #To RGB888
                    if (r > 0):
                        r = r | 0x0f;
                    
                    g = r
                    b = r

                    if (x + 1 == self.w) or (4 == bit_pos):
                        index=index+1

                    if 4 == bit_pos:
                        bit_mask = 0x0F
                        bit_pos  = 0
                    else:
                        bit_mask = 0xF0
                        bit_pos  = 4

                        
                elif bin_format == "a2":
                    
                    r = (ord(bin_array[index]) & bit_mask)>>bit_pos
                    #print ("x,y = %d,%d, src=%02x, mask=%02x,  v=%02x"%(x,y,ord(bin_array[index]),bit_mask,r))
                    
                    r = r << 6 #To RGB888
                    if (r > 0):
                        r = r | 0x3f;

                    g = r
                    b = r
                    
                    

                    if (x + 1 == self.w) or (6 == bit_pos):
                        index=index+1

                    if 6 == bit_pos:
                        bit_mask = 0x03
                        bit_pos  = 0
                    else:
                        bit_mask = bit_mask << 2
                        bit_pos  = bit_pos + 2



                elif bin_format == "rgb565":
                    r = ord(bin_array[index+1]) & 0xF8
                    g_h = (ord(bin_array[index+1]) & 0x7)<<5
                    g_l = (ord(bin_array[index]) & 0xE0) >> 3 
                    g = g_h | g_l
                    b = (ord(bin_array[index]) & 0x1F)<<3
                    index=index+2
                elif bin_format == "argb8565":
                    r = ord(bin_array[index+1]) & 0xF8
                    g_h = (ord(bin_array[index+1]) & 0x7)<<5
                    g_l = (ord(bin_array[index]) & 0xE0) >> 3 
                    g = g_h | g_l
                    b = (ord(bin_array[index]) & 0x1F)<<3
                    a = ord(bin_array[index+2])
                    index=index+3
                elif bin_format == "rgb888":
                    b = ord(bin_array[index])
                    g = ord(bin_array[index+1])
                    r = ord(bin_array[index+2])

                    #r = self.rand_c(r,3,0xF8)
                    #g = self.rand_c(g,2,0xFC)
                    #b = self.rand_c(b,3,0xF8)

                    index=index+3
                elif bin_format == "argb8888":
                    b = ord(bin_array[index])
                    g = ord(bin_array[index+1])
                    r = ord(bin_array[index+2])
                    a = ord(bin_array[index+3])
                    index=index+4
                elif bin_format == "rgba8888":
                    a = ord(bin_array[index])
                    b = ord(bin_array[index+1])
                    g = ord(bin_array[index+2])
                    r = ord(bin_array[index+3])
                    index=index+4
                else:
                    r = 0
                    g = 0
                    b = 0

                #mix with white background

                r = int((r * a)/255 + (255-a))
                g = int((g * a)/255 + (255-a))
                b = int((b * a)/255 + (255-a))
                rgb888 = r << 16 | g<<8 | b
                self.paint_point(x, y, rgb888)

    def save_image(self, name="save.bmp"):
        f = open(name, 'wb')
 
        #write bmp header
        #f.write(array('B', self.bmp_header).tobytes())
        f.write(array('B', self.bmp_header))
 
        #write rgb data
        zeroBytes = self.dataSize // self.h - self.w * 3
 
        for r in range(self.h):
            l = []
            for i in range(len(self.rgbData[r])):
                p = self.rgbData[r][i]
                l.append(p & 0x0000ff)
                p >>= 8
                l.append(p & 0x0000ff)
                p >>= 8
                l.append(p & 0x0000ff)
 
            #f.write(array('B', l).tobytes())
            f.write(array('B', l))

            for i in range(zeroBytes):
                f.write(bytes(0x00))
 
        #close file
        f.close()
 