#
# File      : building.py
# This file is part of RT-Thread RTOS
# COPYRIGHT (C) 2006 - 2015, RT-Thread Development Team
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License along
#  with this program; if not, write to the Free Software Foundation, Inc.,
#  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#
# Change Logs:
# Date           Author       Notes
# 2015-01-20     Bernard      Add copyright information
#

import os
import sys
import string

from SCons.Script import *

BuildOptions = {}
Projects = []
Env = None

def GetCurrentDir():
    conscript = File('SConscript')
    fn = conscript.rfile()
    name = fn.name
    path = os.path.dirname(fn.abspath)
    return path

def PrepareHostModuleBuilding(env):
    global Env
    Env = env;

def GetDepend(depend):
    # always true
    return True

def MergeGroup(src_group, group):
    src_group['src'] = src_group['src'] + group['src']
    if group.has_key('CCFLAGS'):
        if src_group.has_key('CCFLAGS'):
            src_group['CCFLAGS'] = src_group['CCFLAGS'] + group['CCFLAGS']
        else:
            src_group['CCFLAGS'] = group['CCFLAGS']
    if group.has_key('CPPPATH'):
        if src_group.has_key('CPPPATH'):
            src_group['CPPPATH'] = src_group['CPPPATH'] + group['CPPPATH']
        else:
            src_group['CPPPATH'] = group['CPPPATH']
    if group.has_key('CPPDEFINES'):
        if src_group.has_key('CPPDEFINES'):
            src_group['CPPDEFINES'] = src_group['CPPDEFINES'] + group['CPPDEFINES']
        else:
            src_group['CPPDEFINES'] = group['CPPDEFINES']

    # for local CCFLAGS/CPPPATH/CPPDEFINES
    if group.has_key('LOCAL_CCFLAGS'):
        if src_group.has_key('LOCAL_CCFLAGS'):
            src_group['LOCAL_CCFLAGS'] = src_group['LOCAL_CCFLAGS'] + group['LOCAL_CCFLAGS']
        else:
            src_group['LOCAL_CCFLAGS'] = group['LOCAL_CCFLAGS']
    if group.has_key('LOCAL_CPPPATH'):
        if src_group.has_key('LOCAL_CPPPATH'):
            src_group['LOCAL_CPPPATH'] = src_group['LOCAL_CPPPATH'] + group['LOCAL_CPPPATH']
        else:
            src_group['LOCAL_CPPPATH'] = group['LOCAL_CPPPATH']
    if group.has_key('LOCAL_CPPDEFINES'):
        if src_group.has_key('LOCAL_CPPDEFINES'):
            src_group['LOCAL_CPPDEFINES'] = src_group['LOCAL_CPPDEFINES'] + group['LOCAL_CPPDEFINES']
        else:
            src_group['LOCAL_CPPDEFINES'] = group['LOCAL_CPPDEFINES']

    if group.has_key('LINKFLAGS'):
        if src_group.has_key('LINKFLAGS'):
            src_group['LINKFLAGS'] = src_group['LINKFLAGS'] + group['LINKFLAGS']
        else:
            src_group['LINKFLAGS'] = group['LINKFLAGS']
    if group.has_key('LIBS'):
        if src_group.has_key('LIBS'):
            src_group['LIBS'] = src_group['LIBS'] + group['LIBS']
        else:
            src_group['LIBS'] = group['LIBS']
    if group.has_key('LIBPATH'):
        if src_group.has_key('LIBPATH'):
            src_group['LIBPATH'] = src_group['LIBPATH'] + group['LIBPATH']
        else:
            src_group['LIBPATH'] = group['LIBPATH']

def DefineGroup(name, src, depend, **parameters):
    global Env
    if not GetDepend(depend):
        return []

    # find exist group and get path of group
    group_path = ''
    for g in Projects:
        if g['name'] == name:
            group_path = g['path']
    if group_path == '':
        group_path = GetCurrentDir()

    group = parameters
    group['name'] = name
    group['path'] = group_path
    if type(src) == type(['src1']):
        group['src'] = File(src)
    else:
        group['src'] = src

    if group.has_key('CCFLAGS'):
        Env.Append(CCFLAGS = group['CCFLAGS'])
    if group.has_key('CPPPATH'):
        Env.Append(CPPPATH = group['CPPPATH'])
    if group.has_key('CPPDEFINES'):
        Env.Append(CPPDEFINES = group['CPPDEFINES'])
    if group.has_key('LINKFLAGS'):
        Env.Append(LINKFLAGS = group['LINKFLAGS'])

    if group.has_key('LIBS'):
        Env.Append(LIBS = group['LIBS'])
    if group.has_key('LIBPATH'):
        Env.Append(LIBPATH = group['LIBPATH'])

    if group.has_key('LIBRARY'):
        objs = Env.Library(name, group['src'])
    else:
        objs = group['src']

    # merge group
    for g in Projects:
        if g['name'] == name:
            # merge to this group
            MergeGroup(g, group)
            return objs

    # add a new group
    Projects.append(group)

    return objs
