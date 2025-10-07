# -*- coding:utf-8 -*-
#
# File      : rt_package_list.py
# This file is part of RT-Thread RTOS
# COPYRIGHT (C) 2006 - 2020, RT-Thread Development Team

import os
import platform
import rt_pkg.kconfig as kconfig
from rt_pkg.package import PackageOperation

def get_packages():
    """Get the packages list in env.

    Read the.config file in the BSP directory,
    and return the version number of the selected package.
    """

    config_file = 'proj.conf'
    pkgs_root = os.environ.get('PKGS_ROOT', '')
    packages = []
    if not os.path.isfile(config_file):
        print(
            "\033[1;31;40mWarning: Can't find .config.\033[0m"
            '\033[1;31;40mYou should use <menuconfig> command to config bsp first.\033[0m'
        )

        return packages

    config_file = kconfig.gen_config_tmp()
    try:
        packages = kconfig.parse(config_file)
    finally:
        os.remove(config_file)

    for pkg in packages:
        pkg_path = pkg['path']
        if pkg_path[0] == '/' or pkg_path[0] == '\\':
            pkg_path = pkg_path[1:]

        # parse package to get information
        package = PackageOperation()
        pkg_path = os.path.join(pkgs_root, pkg_path, 'package.json')
        if os.path.exists(pkg_path):
            package.parse(pkg_path)

            # update package name
            package_name_in_json = package.get_name()
            pkg['name'] = package_name_in_json

    return packages


def list_packages():
    """Print the packages list in env.

    Read the.config file in the BSP directory,
    and list the version number of the selected package.
    """

    config_file = 'proj.conf'
    pkgs_root = os.environ.get('PKGS_ROOT', '')

    if not os.path.isfile(config_file):
        if platform.system() == "Windows":
            os.system('chcp 65001  > nul')

        print("\033[1;31;40mWarning: Can't find .config.\033[0m")
        print('\033[1;31;40mYou should use <menuconfig> command to config bsp first.\033[0m')

        if platform.system() == "Windows":
            os.system('chcp 437  > nul')

        return

    config_file = kconfig.gen_config_tmp()
    try:
        packages = kconfig.parse(config_file)
    finally:
        os.remove(config_file)

    for pkg in packages:
        package = PackageOperation()
        pkg_path = pkg['path']
        if pkg_path[0] == '/' or pkg_path[0] == '\\':
            pkg_path = pkg_path[1:]

        pkg_path = os.path.join(pkgs_root, pkg_path, 'package.json')
        if os.path.exists(pkg_path):
            package.parse(pkg_path)

            package_name_in_json = package.get_name().encode("utf-8")
            print("package name : %s, ver : %s " % (package_name_in_json, pkg['ver'].encode("utf-8")))

    if not packages:
        print("Packages list is empty.")
        print('You can use < menuconfig > command to select online packages.')
        print('Then use < pkgs --update > command to install them.')
    return