# -*- coding:utf-8 -*-
#
# File      : archive.py
# This file is part of RT-Thread RTOS
# COPYRIGHT (C) 2006 - 2018, RT-Thread Development Team
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
# 2018-5-28      SummerGift      Add copyright information
# 2020-4-10      SummerGift      Code clear up
#

import logging
import os
import shutil
import tarfile
import zipfile
import rt_pkg.pkgsdb as pkgsdb
from rt_pkg.rt_package_utils import is_windows, remove_folder
try:
    from tqdm import tqdm
except ImportError:
    # Fallback if tqdm is not available
    def tqdm(iterable, *args, **kwargs):
        return iterable


def unpack(archive_filename, bsp_package_path, package_info, package_name):
    if archive_filename.endswith(".zip"):
        return handle_zip_package(archive_filename, bsp_package_path, package_name, package_info)

    if ".tar." in archive_filename:
        return handle_tar_package(archive_filename, bsp_package_path, package_name, package_info)

    return True


def handle_tar_package(archive_filename, bsp_package_path, package_name, package_info):
    package_version = package_info['ver']
    package_temp_path = os.path.join(bsp_package_path, "package_temp")

    try:
        if remove_folder(package_temp_path):
            os.makedirs(package_temp_path)
    except Exception as e:
        logging.warning('Error message : {0}'.format(e))

    logging.info("BSP packages path {0}".format(bsp_package_path))
    logging.info("BSP package temp path: {0}".format(package_temp_path))
    logging.info("archive filename : {0}".format(archive_filename))

    try:
        flag = True
        package_folder_name = ""
        package_name_with_version = ""
        arch = tarfile.open(archive_filename, "r")
        for item in tqdm(arch.getnames()):
            arch.extract(item, package_temp_path)
            if not os.path.isdir(os.path.join(package_temp_path, item)):
                # Gets the folder name and changed folder name only once
                if flag:
                    package_folder_name = item.split('/')[0]
                    package_name_with_version = package_name + '-' + package_version
                    flag = False

                if is_windows():
                    right_path = item.replace('/', '\\')
                else:
                    right_path = item

                right_name_to_db = right_path.replace(package_folder_name, package_name_with_version, 1)
                right_path = os.path.join("package_temp", right_path)
                pkgsdb.save_to_database(right_name_to_db, archive_filename, right_path)
        arch.close()

        if not move_package_to_bsp_packages(
            package_folder_name, package_name, package_temp_path, package_version, bsp_package_path
        ):
            return False

    except Exception as e:
        logging.warning('unpack error message : {0}'.format(e))
        logging.warning('unpack {0} failed'.format(os.path.basename(archive_filename)))
        # remove temp folder and archive file
        remove_folder(package_temp_path)
        os.remove(archive_filename)
        return False

    return True


def handle_zip_package(archive_filename, bsp_package_path, package_name, package_info):
    package_version = package_info['ver']
    package_temp_path = os.path.join(bsp_package_path, "package_temp")

    try:
        if remove_folder(package_temp_path):
            os.makedirs(package_temp_path)
    except Exception as e:
        logging.warning('Error message : {0}'.format(e))

    logging.info("BSP packages path {0}".format(bsp_package_path))
    logging.info("BSP package temp path: {0}".format(package_temp_path))
    logging.info("archive filename : {0}".format(archive_filename))

    try:
        flag = True
        package_folder_name = ""
        package_name_with_version = ""
        arch = zipfile.ZipFile(archive_filename, "r")
        for item in tqdm(arch.namelist()):
            arch.extract(item, package_temp_path)
            if not os.path.isdir(os.path.join(package_temp_path, item)):
                # Gets the folder name and changed folder name only once
                if flag:
                    package_folder_name = item.split('/')[0]
                    package_name_with_version = package_name + '-' + package_version
                    flag = False
                if is_windows():
                    right_path = item.replace('/', '\\')
                else:
                    right_path = item

                right_name_to_db = right_path.replace(package_folder_name, package_name_with_version, 1)
                right_path = os.path.join("package_temp", right_path)
                pkgsdb.save_to_database(right_name_to_db, archive_filename, right_path)
        arch.close()

        if not move_package_to_bsp_packages(
            package_folder_name, package_name, package_temp_path, package_version, bsp_package_path
        ):
            return False
    except Exception as e:
        logging.warning('unpack error message : {0}'.format(e))
        logging.warning('unpack {0} failed'.format(os.path.basename(archive_filename)))
        # remove temp folder and archive file
        remove_folder(package_temp_path)
        os.remove(archive_filename)
        return False

    return True


def move_package_to_bsp_packages(package_folder_name, package_name, package_temp_path, package_version, bsp_packages_path):
    """move package in temp folder to bsp packages folder."""
    origin_package_folder_path = os.path.join(package_temp_path, package_folder_name)
    package_name_with_version = package_name + '-' + package_version
    package_folder_in_temp = os.path.join(package_temp_path, package_name_with_version)
    bsp_package_path = os.path.join(bsp_packages_path, package_name_with_version)
    logging.info("origin name: {0}".format(origin_package_folder_path))
    logging.info("rename name: {0}".format(package_folder_in_temp))

    result = True
    try:
        # rename package folder name to package name with version
        os.rename(origin_package_folder_path, package_folder_in_temp)

        # if there is no specified version package in the bsp package path,
        # then move package from package_folder_in_temp to bsp_package_path
        if not os.path.isdir(bsp_package_path):
            shutil.move(package_folder_in_temp, bsp_package_path)
    except Exception as e:
        logging.warning('{0}'.format(e))
        result = False
    finally:
        # must remove temp folder
        remove_folder(package_temp_path)

    return result


def package_integrity_test(path):
    ret = True

    if path.endswith(".zip"):
        try:
            if zipfile.is_zipfile(path):
                # Test zip again to make sure it's a right zip file.
                arch = zipfile.ZipFile(path, "r")
                if arch.testzip():
                    ret = False
                arch.close()
            else:
                ret = False
                print('package check error. \n')
        except Exception as e:
            print('Package test error message:%s\t' % e)
            print("The archive package is broken. \n")
            arch.close()
            ret = False

    # if ".tar.*" in path:.
    if path.endswith(".tar.bz2") or path.endswith(".tar.gz") or path.endswith(".tar.xz"):
        try:
            if not tarfile.is_tarfile(path):
                ret = False
        except Exception as e:
            print('Error message:%s' % e)
            ret = False

    return ret