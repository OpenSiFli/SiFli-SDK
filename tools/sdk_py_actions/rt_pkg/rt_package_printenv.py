# -*- coding:utf-8 -*-
#
# File      : rt_package_printenv.py
# This file is part of RT-Thread RTOS
# COPYRIGHT (C) 2006 - 2020, RT-Thread Development Team
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
# Date           Author          Notes
# 2020-04-08     SummerGift      Optimize program structure
#

import os
import platform


def package_print_env():
    print("Here are some environmental variables.")
    print("If you meet some problems,please check them. Make sure the configuration is correct.")
    print("RTT_EXEC_PATH:%s" % (os.getenv("RTT_EXEC_PATH")))
    print("RTT_CC:%s" % (os.getenv("RTT_CC")))
    print("SCONS:%s" % (os.getenv("SCONS")))
    print("PKGS_ROOT:%s" % (os.getenv("PKGS_ROOT")))

    env_root = os.getenv('ENV_ROOT')
    if env_root is None:
        if platform.system() != 'Windows':
            env_root = os.path.join(os.getenv('HOME'), '.env')

    print("ENV_ROOT:%s" % env_root)