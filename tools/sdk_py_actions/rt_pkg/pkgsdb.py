# -*- coding:utf-8 -*-
#
# File      : pkgsdb.py
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

import sqlite3
import os
import hashlib
from .rt_package_utils import user_input

SHOW_SQL = False


def get_file_md5(filename):
    if not os.path.isfile(filename):
        return
    hash_value = hashlib.md5()
    f = open(filename, 'rb')
    while True:
        b = f.read(8096)
        if not b:
            break
        hash_value.update(b)
    f.close()
    return hash_value.hexdigest()

def get_conn(path):
    """Get database connection."""
    # create dbfile if not present
    conn = sqlite3.connect(path)
    if os.path.exists(path) and os.path.isfile(path):
        return conn
    else:
        print('on memory:[:memory:]')
        return sqlite3.connect(':memory:')


def close_all(conn):
    if conn is not None:
        conn.close()


def get_cursor(conn):
    if conn is not None:
        return conn.cursor()
    else:
        return get_conn('').cursor()

def create_table(conn, sql):
    """Create table in database."""
    if conn and sql:
        cu = conn.cursor()
        cu.execute(sql)
        conn.commit()


def save(conn, sql, data):
    """insert data to database"""
    if sql is not None and sql != '':
        if data is not None:
            cu = get_cursor(conn)
            for d in data:
                if SHOW_SQL:
                    print('execute sql:[{}],arguments:[{}]'.format(sql, d))
                cu.execute(sql, d)
                conn.commit()
            close_all(conn)
    else:
        print('the [{}] is empty or equal None!'.format(sql))

# Add data to the database, if the data already exists, don't add again
def save_to_database(pathname, package_pathname, before_change_name):
    """Save to database."""
    print(f"Saving to database: {pathname}, {package_pathname}, {before_change_name}")
    db_pathname = os.getenv('SIFLI_SDK_RT_PKG_DB_PATH')
    bsp_root = os.getenv('BSP_ROOT')
    bsp_packages_path = os.path.join(bsp_root, 'packages')

    conn = get_conn(db_pathname)
    save_sql = '''insert into packagefile values (?, ?, ?)'''
    package = os.path.basename(package_pathname)
    md5pathname = os.path.join(bsp_packages_path, before_change_name)

    if not os.path.isfile(md5pathname):
        print("md5pathname is Invalid")

    md5 = get_file_md5(md5pathname)
    data = [(pathname, package, md5)]
    save(conn, save_sql, data)


def remove_unchanged_file(pathname, dbfilename, dbsqlname):
    """delete unchanged files"""
    flag = True

    conn = get_conn(dbfilename)
    c = get_cursor(conn)
    filemd5 = get_file_md5(pathname)
    dbmd5 = 0

    sql = 'SELECT md5 from packagefile where pathname = "' + dbsqlname + '"'
    # print sql
    cursor = c.execute(sql)
    for row in cursor:
        # fetch md5 from database
        dbmd5 = row[0]

    if dbmd5 == filemd5:
        # delete file info from database
        sql = "DELETE from packagefile where pathname = '" + dbsqlname + "'"
        conn.commit()
        os.remove(pathname)
    else:
        print("%s has been changed." % pathname)
        print('Are you sure you want to permanently delete the file: %s?' % os.path.basename(pathname))
        print(
            'If you choose to keep the changed file,you should copy the file to another folder. '
            '\nbecaues it may be covered by the next update.'
        )

        rc = user_input('Press the Y Key to delete the folder or just press Enter to keep it : ')
        if rc == 'y' or rc == 'Y':
            sql = "DELETE from packagefile where pathname = '" + dbsqlname + "'"
            conn.commit()
            os.remove(pathname)
            print("%s has been removed.\n" % pathname)
        else:
            flag = False
    conn.close()
    return flag

def deletepackdir(dirpath, dbpathname):
    """Delete package directory."""
    print(f"Deleting package directory: {dirpath}")
    flag = getdirdisplay(dirpath, dbpathname)

    if flag:
        if os.path.exists(dirpath):
            for root, dirs, files in os.walk(dirpath, topdown=False):
                for name in files:
                    os.remove(os.path.join(root, name))
                for name in dirs:
                    os.rmdir(os.path.join(root, name))
            os.rmdir(dirpath)
        # print "the dir should be delete"
    return flag


# walk through all files in filepath, include subfolder
def displaydir(filepath, basepath, length, dbpathname):
    flag = True
    if os.path.isdir(filepath):
        files = os.listdir(filepath)
        for fi in files:
            fi_d = os.path.join(filepath, fi)
            if os.path.isdir(fi_d):
                displaydir(fi_d, basepath, length, dbpathname)
            else:
                pathname = os.path.join(filepath, fi_d)
                dbsqlname = basepath + os.path.join(filepath, fi_d)[length:]
                if not remove_unchanged_file(pathname, dbpathname, dbsqlname):
                    flag = False
    return flag


def getdirdisplay(filepath, dbpathname):
    display = filepath
    length = len(display)
    basepath = os.path.basename(filepath)
    flag = displaydir(filepath, basepath, length, dbpathname)
    return flag
