# -*- coding:utf-8 -*-
#
# File      : rt_package_update.py
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
# 2020-04-13     SummerGift      refactoring
#

import json
import logging
import os
import platform
import shutil
import time

import requests

import rt_pkg.archive as archive
import rt_pkg.kconfig as kconfig
import rt_pkg.pkgsdb as pkgsdb
from rt_pkg.package import PackageOperation, Bridge_SConscript
from rt_pkg.rt_package_utils import get_url_from_mirror_server, execute_command, git_pull_repo, user_input, find_bool_macro_in_config


def determine_support_chinese(env_root):
    get_flag_file_path = os.path.join(env_root, 'tools', 'bin', 'env_above_ver_1_1')
    if os.path.isfile(get_flag_file_path):
        return True
    else:
        return False


def get_mirror_giturl(submodule_name):
    """Gets the submodule's url on mirror server.

    Retrurn the download address of the submodule on the mirror server from the submod_name.
    """

    mirror_url = 'https://gitee.com/RT-Thread-Mirror/submod_' + submodule_name + '.git'
    return mirror_url


def get_esp_mirror_giturl(submodule_name):
    if submodule_name == "cexception":
        submodule_name = "CException"
    elif submodule_name == "unity":
        submodule_name = "Unity"
    mirror_url = 'https://gitee.com/esp-submodules/' + submodule_name + '.git'
    return mirror_url


def modify_submod_file_to_mirror(gitmodule_path, use_esp_mirror):
    """Modify the.gitmodules file based on the submodule to be updated"""

    replace_list = []
    try:
        with open(gitmodule_path, 'r') as f:
            line = f.readline()
            while line:
                line = line.replace('\t', '').replace(' ', '').replace('\n', '').replace('\r', '')
                if line.startswith('path'):
                    submodule_path = line.split("=")[1]
                    line = f.readline()
                    line = line.replace('\t', '').replace(' ', '').replace('\n', '').replace('\r', '')
                    if line.startswith('url'):
                        submodule_git_url = line.split('=')[1]
                        submodule_name = submodule_git_url.split('/')[-1].replace('.git', '')
                        if not use_esp_mirror:
                            query_submodule_name = 'submod_' + submodule_name
                            get_package_url, get_ver_sha = get_url_from_mirror_server(query_submodule_name, 'latest')
                        else:
                            get_package_url = get_esp_mirror_giturl(submodule_name)

                        if get_package_url is not None and determine_url_valid(get_package_url):
                            replace_list.append((submodule_git_url, get_package_url, submodule_name, submodule_path))
                line = f.readline()

        with open(gitmodule_path, 'r+') as f:
            submod_file_count = f.read()

        write_content = submod_file_count

        for item in replace_list:
            write_content = write_content.replace(item[0], item[1])

        with open(gitmodule_path, 'w') as f:
            f.write(str(write_content))

        return replace_list

    except Exception as e:
        logging.warning('Error message:%s\t' % e)


def determine_url_valid(url_from_srv):
    headers = {
        'Connection': 'keep-alive',
        'Accept-Encoding': 'gzip, deflate',
        'Accept': '*/*',
        'User-Agent': 'curl/7.54.0',
    }

    # noinspection PyBroadException
    try:
        for i in range(0, 3):
            r = requests.get(url_from_srv, stream=True, headers=headers)
            if r.status_code == requests.codes.not_found:
                if i == 2:
                    print("Warning : %s is invalid." % url_from_srv)
                    return False
                time.sleep(1)
            else:
                break

        return True

    except Exception as e:
        # So much error message should be ignore
        logging.error('Network connection error or the url : %s is invalid.\n' % url_from_srv.encode("utf-8"))


def is_user_mange_package(bsp_package_path, pkg):
    for root, dirs, files in os.walk(bsp_package_path, topdown=True):
        for name in dirs:
            package_name_lower = pkg["name"].lower()
            folder_name_lower = name.lower()
            folder_name_common = folder_name_lower.replace("-", "_")
            if folder_name_lower == package_name_lower or folder_name_common == package_name_lower:
                return True
        break
    return False


is_China_ip = None


def need_using_mirror_download():
    global is_China_ip

    if is_China_ip != None:
        return is_China_ip

    server_decision = ""
    config_file = os.path.join(os.getenv("ENV_ROOT"), 'tools', 'scripts', 'cmds', '.config')
    if os.path.isfile(config_file) and find_bool_macro_in_config(config_file, 'SYS_DOWNLOAD_SERVER_GITHUB'):
        is_China_ip = False  # Github which means not China IP
        server_decision = "manually decision"
    elif os.path.isfile(config_file) and find_bool_macro_in_config(config_file, 'SYS_DOWNLOAD_SERVER_GITEE'):
        is_China_ip = True  # Gitee which means China IP
        server_decision = "manually decision"
    else:
        try:
            ip = requests.get('https://ifconfig.me/ip').content.decode()
            url = 'http://www.ip-api.com/json/' + ip
            if requests.get(url).json()['country'] == 'China':
                is_China_ip = True
            else:
                is_China_ip = False
            server_decision = "auto decision based on IP location"
        except:
            if (-time.timezone) / 3600 == 8:
                is_China_ip = True
            else:
                is_China_ip = False
            server_decision = "auto decision based on timezone"

    print("[Use {} server - {}]".format(("Gitee" if is_China_ip else "Github"), server_decision))
    return is_China_ip


def is_git_url(package_url):
    return package_url.endswith('.git')


def update_submodule(repo_path, use_esp_mirror):
    print(repo_path)
    # If there is a .gitmodules file in the package, prepare to update submodule.
    gitmodules_path = os.path.join(repo_path, '.gitmodules')
    if os.path.isfile(gitmodules_path):
        if need_using_mirror_download():
            replace_list = modify_submod_file_to_mirror(gitmodules_path, use_esp_mirror)
            cmd = 'git submodule update --init'
            execute_command(cmd, cwd=repo_path)
            for item in replace_list:
                submodule_path = os.path.join(repo_path, item[3])
                print(submodule_path)
                if os.path.isdir(submodule_path):
                    update_submodule(submodule_path, use_esp_mirror)
            cmd = 'git checkout .gitmodules'
            execute_command(cmd, cwd=repo_path)
            cmd = 'git submodule sync'
            execute_command(cmd, cwd=repo_path)
        else:
            cmd = 'git submodule update --init --recursive'
            execute_command(cmd, cwd=repo_path)


def install_git_package(bsp_package_path, package_name, package_info, package_url, ver_sha, upstream_changed, url_origin):
    try:
        repo_path = os.path.join(bsp_package_path, package_name)
        repo_path = repo_path + '-' + package_info['ver']
        repo_name_with_version = '"' + repo_path + '"'

        clone_cmd = 'git clone ' + package_url + ' ' + repo_name_with_version + ' --depth=1'
        logging.info(clone_cmd)
        execute_command(clone_cmd, cwd=bsp_package_path)

        git_check_cmd = 'git checkout -q ' + ver_sha
        execute_command(git_check_cmd, cwd=repo_path)
    except Exception as e:
        print('Error message:%s' % e)
        print("\nFailed to download software package with git. Please check the network connection.")
        return False

    # change upstream to origin url
    if upstream_changed:
        cmd = 'git remote set-url origin ' + url_origin
        execute_command(cmd, cwd=repo_path)

    # If there is a .gitmodules file in the package, prepare to update submodule.
    gitmodules_path = os.path.join(repo_path, '.gitmodules')
    if os.path.isfile(gitmodules_path):
        print("Start to update submodule")
        update_submodule(repo_path, package_name == u"ESP-IDF")

    return True


def install_not_git_package(package, package_info, local_pkgs_path, package_url, bsp_package_path, package_name):
    result = True
    # Download a package of compressed package type.
    if not package.download(package_info['ver'], local_pkgs_path, package_url):
        return False

    pkg_dir = package.get_filename(package_info['ver'])
    pkg_dir = os.path.splitext(pkg_dir)[0]
    package_path = os.path.join(local_pkgs_path, package.get_filename(package_info['ver']))

    if not archive.package_integrity_test(package_path):
        print("package : %s is invalid" % package_path.encode("utf-8"))
        return False

    # unpack package
    if not os.path.exists(pkg_dir):
        try:
            if not archive.unpack(package_path, bsp_package_path, package_info, package_name):
                result = False
        except Exception as e:
            result = False
            logging.error('Error message: {0}'.format(e))
    else:
        print("The file does not exist.")

    return result


# noinspection PyUnboundLocalVariable
def install_package(env_root, pkgs_root, bsp_root, package_info, force_update):
    """Install the required packages."""

    result = True
    local_pkgs_path = os.path.join(env_root, 'local_pkgs')
    if not os.path.exists(local_pkgs_path):
        os.mkdir(local_pkgs_path)
    bsp_package_path = os.path.join(bsp_root, 'packages')

    if not force_update:
        logging.info("Begin to check if it's an user managed package {0}, {1} \n".format(bsp_package_path, package_info))
        if is_user_mange_package(bsp_package_path, package_info):
            logging.info("User managed package {0}, {1} no need install. \n".format(bsp_package_path, package_info))
            return result
        else:
            logging.info("NOT User managed package {0}, {1} need install. \n".format(bsp_package_path, package_info))

    package = PackageOperation()
    pkg_path = package_info['path']
    if pkg_path[0] == '/' or pkg_path[0] == '\\':
        pkg_path = pkg_path[1:]
    pkg_path = os.path.join(pkgs_root, pkg_path, 'package.json')
    package.parse(pkg_path)

    try:
        url_from_json = package.get_url(package_info['ver'])
    except:
        print(
            "Warning: cannot find the corresponding version. Please check {}'s Kconfig file".format(
                package_info['name'].capitalize()
            )
        )
        return False

    if not url_from_json:
        return False

    package_url = url_from_json

    pkgs_name_in_json = package.get_name()
    logging.info("begin to install packages: {0}".format(pkgs_name_in_json))
    if is_git_url(package_url):
        ver_sha = package.get_versha(package_info['ver'])

    upstream_changed = False

    # noinspection PyBroadException
    try:
        if need_using_mirror_download():
            get_package_url, get_ver_sha = get_url_from_mirror_server(pkgs_name_in_json, package_info['ver'])

            #  Check whether the package package url is valid
            if get_package_url and determine_url_valid(get_package_url):
                package_url = get_package_url

                if get_ver_sha:
                    ver_sha = get_ver_sha

                upstream_changed = True
    except Exception as e:
        logging.warning("Failed to connect to the mirror server, package will be downloaded from non-mirror server.\n")

    if is_git_url(package_url):
        if not install_git_package(
            bsp_package_path, pkgs_name_in_json, package_info, package_url, ver_sha, upstream_changed, url_from_json
        ):
            result = False
    else:
        if not install_not_git_package(
            package, package_info, local_pkgs_path, package_url, bsp_package_path, pkgs_name_in_json
        ):
            result = False
    return result


def sub_list(aList, bList):
    """Return the items in aList but not in bList."""

    tmp = []
    for a in aList:
        if a not in bList:
            tmp.append(a)
    return tmp


def and_list(aList, bList):
    """Return the items in aList and in bList."""

    tmp = []
    for a in aList:
        if a in bList:
            tmp.append(a)
    return tmp


def get_package_folder(origin_path, version):
    return origin_path + '-' + version


def git_cmd_exec(cmd, cwd):
    try:
        execute_command(cmd, cwd=cwd)
    except Exception as e:
        logging.warning('Error message:%s%s. %s \n\t' % (cwd.encode("utf-8"), " path doesn't exist", e))
        print("You can solve this problem by manually removing old packages and re-downloading them using env.")


def update_latest_packages(sys_value):
    """update the packages that are latest version.

    If the selected package is the latest version,
    check to see if it is the latest version after the update command,
    if not, then update the latest version from the remote repository.
    If the download has a conflict, you are currently using the prompt
    message provided by git.
    """

    logging.info("Begin to update latest version packages")

    result = True

    package_filename = sys_value[3]
    bsp_packages_path = sys_value[5]

    env_root = os.getenv('ENV_ROOT')
    pkgs_root = os.getenv('PKGS_ROOT')

    with open(package_filename, 'r') as f:
        read_back_pkgs_json = json.load(f)

    for pkg in read_back_pkgs_json:
        right_path_flag = True
        package = PackageOperation()
        pkg_path = pkg['path']
        if pkg_path[0] == '/' or pkg_path[0] == '\\':
            pkg_path = pkg_path[1:]

        pkg_path = os.path.join(pkgs_root, pkg_path, 'package.json')
        package.parse(pkg_path)
        pkgs_name_in_json = package.get_name()

        # Find out the packages which version is 'latest'
        if pkg['ver'] == "latest_version" or pkg['ver'] == "latest":
            repo_path = os.path.join(bsp_packages_path, pkgs_name_in_json)
            repo_path = get_package_folder(repo_path, pkg['ver'])

            # noinspection PyBroadException
            try:
                # If mirror acceleration is enabled, get the update address from the mirror server.
                if need_using_mirror_download():
                    payload_pkgs_name_in_json = pkgs_name_in_json.encode("utf-8")

                    # Change repo's upstream address.
                    mirror_url = get_url_from_mirror_server(payload_pkgs_name_in_json, pkg['ver'])

                    # if git root is same as repo path, then change the upstream
                    get_git_root = get_git_root_path(repo_path)
                    if get_git_root:
                        if os.path.normcase(repo_path) == os.path.normcase(get_git_root):
                            if mirror_url[0] is not None:
                                cmd = 'git remote set-url origin ' + mirror_url[0]
                                git_cmd_exec(cmd, repo_path)
                        else:
                            print("\n==============================> updating")
                            print("Package path: %s" % repo_path)
                            print("Git root: %s" % get_git_root)
                            print("Error: Not currently in a git root directory, cannot switch upstream.\n")
                            right_path_flag = False
                            result = False
                    else:
                        right_path_flag = False
                        result = False

            except Exception as e:
                logging.warning("Failed to connect to the mirror server, using non-mirror server to update.")

            if not right_path_flag:
                continue

            # Update the package repository from upstream.
            git_pull_repo(repo_path)

            # If the package has submodules, update the submodules.
            update_submodule(repo_path, pkgs_name_in_json == u"ESP-IDF")

            # recover origin url to the path which get from packages.json file
            if package.get_url(pkg['ver']):
                cmd = 'git remote set-url origin ' + package.get_url(pkg['ver'])
                git_cmd_exec(cmd, repo_path)
            else:
                print("Can't find the package : %s's url in file : %s" % (payload_pkgs_name_in_json, pkg_path))

            print("==============================>  %s update done\n" % pkgs_name_in_json)

    return result


def get_git_root_path(repo_path):
    if os.path.isdir(repo_path):
        try:
            before = os.getcwd()
            os.chdir(repo_path)
            result = os.popen("git rev-parse --show-toplevel")
            result = result.read()
            for line in result.splitlines()[:5]:
                get_git_root = line
                break
            os.chdir(before)
            return get_git_root
        except Exception as e:
            logging.warning("Error message : %s" % e)
            return None
    else:
        logging.warning("Missing path {0}".format(repo_path))
        logging.warning("If you manage this package manually, Env tool will not update it.")
        return None


def pre_package_update():
    """Make preparations before updating the software package."""

    logging.info("Begin prepare package update")
    bsp_root = os.getenv('BSP_ROOT')
    env_root = os.getenv('ENV_ROOT')

    if not os.path.exists('proj.conf'):
        if platform.system() == "Windows":
            os.system('chcp 65001  > nul')

        print("\n\033[1;31;40m当前路径下没有发现 proj.conf 文件，请确保当前目录为 BSP 根目录。\033[0m")
        print("\033[1;31;40m如果确定当前目录为 BSP 根目录，请先使用 <menuconfig> 命令来生成 proj.conf 文件。\033[0m\n")

        print('No system configuration file : proj.conf')
        print('You should use < menuconfig > command to config bsp first.')

        if platform.system() == "Windows":
            os.system('chcp 437  > nul')

        return False

    # According to the env version, whether Chinese output is supported or not
    if determine_support_chinese(env_root):
        if platform.system() == "Windows":
            os.system('chcp 65001 > nul')

    # create packages folder
    bsp_packages_path = os.path.join(bsp_root, 'packages')
    if not os.path.exists(bsp_packages_path):
        os.mkdir("packages")
        os.chdir(bsp_packages_path)
        fp = open("pkgs.json", 'w')
        fp.write("[]")
        fp.close()

        fp = open("pkgs_error.json", 'w')
        fp.write("[]")
        fp.close()
        os.chdir(bsp_root)

    # prepare target packages file
    dbsqlite_pathname = os.path.join(bsp_packages_path, 'packages.dbsqlite')
    os.environ['SIFLI_SDK_RT_PKG_DB_PATH'] = dbsqlite_pathname
    dbsqlite_pathname = dbsqlite_pathname.encode('utf-8').decode('gbk')

    # avoid creating tables more than one time
    if not os.path.isfile(dbsqlite_pathname):
        conn = pkgsdb.get_conn(dbsqlite_pathname)
        sql = '''CREATE TABLE packagefile
                    (pathname   TEXT  ,package  TEXT  ,md5  TEXT );'''
        pkgsdb.create_table(conn, sql)


    fn = kconfig.gen_config_tmp()
    try:
        pkgs = kconfig.parse(fn)
    finally:
        os.remove(fn)
    newpkgs = pkgs

    # regenerate file : packages/pkgs.json
    package_json_filename = os.path.join(bsp_packages_path, 'pkgs.json')
    if not os.path.exists(package_json_filename):
        os.chdir(bsp_packages_path)
        fp = open("pkgs.json", 'w')
        fp.write("[]")
        fp.close()
        os.chdir(bsp_root)

    # Reading data back from pkgs.json
    with open(package_json_filename, 'r') as f:
        oldpkgs = json.load(f)

    # regenerate file : packages/pkgs_error.json
    pkgs_error_list_fn = os.path.join(bsp_packages_path, 'pkgs_error.json')

    if not os.path.exists(pkgs_error_list_fn):
        os.chdir(bsp_packages_path)
        fp = open("pkgs_error.json", 'w')
        fp.write("[]")
        fp.close()
        os.chdir(bsp_root)

    # read data back from pkgs_error.json
    with open(pkgs_error_list_fn, 'r') as f:
        package_error = json.load(f)

    # create SConscript file
    if not os.path.isfile(os.path.join(bsp_packages_path, 'SConscript')):
        with open(os.path.join(bsp_packages_path, 'SConscript'), 'w') as f:
            f.write(str(Bridge_SConscript))

    return [oldpkgs, newpkgs, package_error, package_json_filename, pkgs_error_list_fn, bsp_packages_path, dbsqlite_pathname]


def error_packages_handle(error_packages_list, read_back_pkgs_json, package_filename, force_update):
    bsp_root = os.getenv('BSP_ROOT')
    env_root = os.getenv('ENV_ROOT')
    pkgs_root = os.getenv('PKGS_ROOT')
    download_error = []
    flag = True

    if len(error_packages_list):
        print("\n==============================> Packages list to download :  \n")
        for package in error_packages_list:
            print("Package name : %s, Ver : %s" % (package['name'].encode("utf-8"), package['ver'].encode("utf-8")))
        print("\nThe packages in the list above are accidentally deleted or renamed.")
        print("\nIf you manually delete the version suffix of the package folder name, ")
        print("you can use <pkgs --force-update> command to re-download these packages.")
        print("In case of accidental deletion, the ENV tool will automatically re-download these packages.")

        # re-download the packages in error_packages_list
        for package in error_packages_list:
            if install_package(env_root, pkgs_root, bsp_root, package, force_update):
                print(
                    "\n==============================> %s %s update done \n"
                    % (package['name'].encode("utf-8"), package['ver'].encode("utf-8"))
                )
            else:
                download_error.append(package)
                print(package, 'download failed.')
                flag = False

        if len(download_error):
            print("%s" % download_error)

            for package in download_error:
                print(
                    "Packages:%s, %s re-download error, you can use <pkgs --update> command to re-download them."
                    % (package['name'].encode("utf-8"), package['ver'].encode("utf-8"))
                )

            error_write_back = sub_list(read_back_pkgs_json, download_error)
            with open(package_filename, 'w') as f:
                f.write(json.dumps(error_write_back, indent=1))

    return flag


def rm_package(dir_remove):
    logging.info("remove dir: {0}".format(dir_remove))

    if platform.system() != "Windows":
        shutil.rmtree(dir_remove)
    else:
        dir_remove = '"' + dir_remove + '"'
        cmd = 'rd /s /q ' + dir_remove
        os.system(cmd)

    if os.path.isdir(dir_remove):
        if platform.system() != "Windows":
            shutil.rmtree(dir_remove)
        else:
            dir_remove = '"' + dir_remove + '"'
            cmd = 'rmdir /s /q ' + dir_remove
            os.system(cmd)

        if os.path.isdir(dir_remove):
            print("Folder path: %s" % dir_remove.encode("utf-8"))
            return False
    else:
        print("Path: %s \nSuccess: Folder has been removed. " % dir_remove.encode("utf-8"))
        return True


def get_package_remove_path(pkg, bsp_packages_path):
    dir_path = pkg['path']
    ver = pkg['ver']
    if dir_path[0] == '/' or dir_path[0] == '\\':
        dir_path = dir_path[1:]

    if platform.system() == "Windows":
        dir_path = os.path.basename(dir_path.replace('/', '\\'))
    else:
        dir_path = os.path.basename(dir_path)

    # Handles the deletion of git repository folders with version Numbers
    remove_path = os.path.join(bsp_packages_path, dir_path)
    remove_path_ver = get_package_folder(remove_path, ver)
    return remove_path_ver


def handle_download_error_packages(sys_value, force_update):
    """handle download error packages.

    Check to see if the packages stored in the Json file list actually exist,
    and then download the packages if they don't exist.
    """

    logging.info("begin to handel download error packages")
    package_filename = sys_value[3]
    bsp_packages_path = sys_value[5]

    with open(package_filename, 'r') as f:
        package_json = json.load(f)

    error_packages_list = []

    for pkg in package_json:
        remove_path = get_package_remove_path(pkg, bsp_packages_path)
        if os.path.exists(remove_path):
            continue
        else:
            print("Error package : %s" % pkg)
            error_packages_list.append(pkg)

    # Handle the failed download packages
    get_flag = error_packages_handle(error_packages_list, package_json, package_filename, force_update)

    return get_flag


def delete_useless_packages(sys_value):
    logging.info("Begin to delete useless packages")
    package_delete_error_list = sys_value[2]
    bsp_packages_path = sys_value[5]

    # try to delete useless packages, exit command if fails
    if len(package_delete_error_list):
        for error_package in package_delete_error_list:
            remove_path_with_version = get_package_remove_path(error_package, bsp_packages_path)
            if os.path.isdir(remove_path_with_version):
                print("\nError: %s package delete failed, begin to remove it." % error_package['name'].encode("utf-8"))

                if not rm_package(remove_path_with_version):
                    print(
                        "Error: Delete package %s failed! Please delete the folder manually.\n"
                        % error_package['name'].encode("utf-8")
                    )
                    return False
    return True


def is_git_package(pkg, bsp_packages_path):
    remove_path_with_version = get_package_remove_path(pkg, bsp_packages_path)
    remove_path_git = os.path.join(remove_path_with_version, '.git')
    return os.path.isdir(remove_path_with_version) and os.path.isdir(remove_path_git)


def delete_git_package(pkg, remove_path_with_version, force_update, package_delete_fail_list):
    git_folder_to_remove = remove_path_with_version

    print("\nStart to remove %s \nplease wait..." % git_folder_to_remove.encode("utf-8"))
    if force_update:
        logging.info("package force update, Begin to remove package {0}".format(git_folder_to_remove))
        if not rm_package(git_folder_to_remove):
            print("Floder delete fail: %s" % git_folder_to_remove.encode("utf-8"))
            print("Please delete this folder manually.")
    else:
        print("The folder is managed by git. Do you want to delete this folder?\n")
        rc = user_input('Press the Y Key to delete the folder or just press Enter to keep it : ')
        if rc == 'y' or rc == 'Y':
            try:
                if not rm_package(git_folder_to_remove):
                    package_delete_fail_list.append(pkg)
                    print("Error: Please delete the folder manually.")
            except Exception as e:
                logging.warning(
                    'Error message:%s%s. error.message: %s\n\t'
                    % ("Delete folder failed: ", git_folder_to_remove.encode("utf-8"), e)
                )


def delete_zip_package(pkg, remove_path_with_version, package_delete_fail_list, sqlite_pathname):
    if os.path.isdir(remove_path_with_version):
        print("Start to remove %s \nplease wait..." % remove_path_with_version.encode("utf-8"))
        try:
            pkgsdb.deletepackdir(remove_path_with_version, sqlite_pathname)
        except Exception as e:
            package_delete_fail_list.append(pkg)
            logging.warning(
                'Error message:\n%s %s. %s \n\t'
                % ("Delete folder failed, please delete the folder manually", remove_path_with_version.encode("utf-8"), e)
            )


def remove_packages(sys_value, force_update):
    logging.info("Begin to remove packages")
    old_package = sys_value[0]
    new_package = sys_value[1]
    package_error_list_filename = sys_value[4]
    bsp_packages_path = sys_value[5]
    sqlite_pathname = sys_value[6]

    case_delete = sub_list(old_package, new_package)
    package_delete_fail_list = []

    for pkg in case_delete:
        remove_path_with_version = get_package_remove_path(pkg, bsp_packages_path)

        # delete .git directory
        if is_git_package(pkg, bsp_packages_path):
            delete_git_package(pkg, remove_path_with_version, force_update, package_delete_fail_list)
        else:
            delete_zip_package(pkg, remove_path_with_version, package_delete_fail_list, sqlite_pathname)

    # write error messages
    with open(package_error_list_filename, 'w') as f:
        f.write(str(json.dumps(package_delete_fail_list, indent=1)))

    if len(package_delete_fail_list):
        return False

    return True


def install_packages(sys_value, force_update):
    """
    If the package download fails, record it,
    and then download again when the update command is executed.
    """

    logging.info("Begin to install packages")

    old_package = sys_value[0]
    new_package = sys_value[1]

    logging.info("old_package {0}".format(old_package))
    logging.info("new_package {0}".format(new_package))

    package_filename = sys_value[3]
    bsp_root = os.getenv('BSP_ROOT')
    pkgs_root = os.getenv('PKGS_ROOT')
    env_root = os.getenv('ENV_ROOT')

    case_download = sub_list(new_package, old_package)
    packages_download_fail_list = []

    for package in case_download:
        if install_package(env_root, pkgs_root, bsp_root, package, force_update):
            print("==============================>  %s %s is downloaded successfully. \n" % (package['name'], package['ver']))
        else:
            # if package download fails, record it in the packages_download_fail_list
            packages_download_fail_list.append(package)
            print(package, 'download failed.')
            return False

    # Get the currently updated configuration.
    new_package = sub_list(new_package, packages_download_fail_list)

    # Give hints based on the success of the download.
    if len(packages_download_fail_list):
        print("\nPackage download failed list:")
        for item in packages_download_fail_list:
            print(item)

        print("You need to reuse the <pkgs --update> command to download again.")

    # Update pkgs.json and SConscript
    with open(package_filename, 'w') as f:
        f.write(str(json.dumps(new_package, indent=1)))

    return True


def package_update(force_update=False):
    """Update env's packages.

    Compare the old and new software package list and update the package.
    Remove unwanted packages and download the newly selected package.-
    Check if the files in the deleted packages have been changed, and if so,
    remind the user saved the modified file.
    """

    sys_value = pre_package_update()
    if not sys_value:
        return

    flag = True

    if not delete_useless_packages(sys_value):
        return

    # 1.in old and not in new : Software packages that need to be removed
    if not remove_packages(sys_value, force_update):
        return

    # 2.in new not in old : Software packages to be installed.
    if not install_packages(sys_value, force_update):
        flag = False

    # 3.handle download error packages.
    if not handle_download_error_packages(sys_value, force_update):
        flag = False

    # 4.update the software packages, which the version is 'latest'
    if not update_latest_packages(sys_value):
        flag = False

    if flag:
        print("Operation completed successfully.")
    else:
        print("Operation failed.")
