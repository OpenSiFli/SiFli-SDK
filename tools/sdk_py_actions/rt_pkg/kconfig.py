# -*- coding:utf-8 -*-
#
# File      : kconfig.py
# This file is part of RT-Thread RTOS

import os
import subprocess


def gen_config_tmp():
    dot_config_tmp = '.config.pkg.tmp'
    config_h_tmp = 'config.pkg.h.tmp'
    kconfiglist_tmp = 'kconfiglist.pkg.tmp'

    try:
        SIFLI_SDK = os.getenv('SIFLI_SDK') or os.getenv('SIFLI_SDK_PATH')
        if not SIFLI_SDK:
            print("Warning: SIFLI_SDK environment variable not set")
            return dot_config_tmp
            
        KCONFIG_PATH = os.path.join(SIFLI_SDK, "tools/kconfig/kconfig.py")
        
        if os.path.exists(KCONFIG_PATH):
            retcode = subprocess.call(['python', KCONFIG_PATH, '--handwritten-input-configs', 'Kconfig',
                                    dot_config_tmp, config_h_tmp, kconfiglist_tmp, 'proj.conf'])
            if retcode != 0:
                print("Warning: Failed to generate .config")
        else:
            print(f"Warning: kconfig.py not found at {KCONFIG_PATH}")
            
    except Exception as e:
        print(f"Warning: Error generating config: {e}")
    finally:
        # Clean up temporary files
        try:
            if os.path.exists(config_h_tmp):
                os.remove(config_h_tmp)
            if os.path.exists(kconfiglist_tmp):
                os.remove(kconfiglist_tmp)
        except:
            pass

    return dot_config_tmp


def pkgs_path(pkgs, name, path):
    for pkg in pkgs:
        if 'name' in pkg and pkg['name'] == name:
            pkg['path'] = path
            return

    pkg = {}
    pkg['name'] = name
    pkg['path'] = path
    pkgs.append(pkg)


def pkgs_ver(pkgs, name, ver):
    for pkg in pkgs:
        if 'name' in pkg and pkg['name'] == name:
            pkg['ver'] = ver
            return

    pkg = {}
    pkg['name'] = name
    pkg['ver'] = ver
    pkgs.append(pkg)


def parse(filename):
    ret = []

    # noinspection PyBroadException
    try:
        config = open(filename, "r")
    except Exception as e:
        print('open .config failed')
        return ret

    for line in config:
        line = line.lstrip(' ').replace('\n', '').replace('\r', '')

        if len(line) == 0:
            continue

        if line[0] == '#':
            continue
        else:
            setting = line.split('=', 1)
            if len(setting) >= 2:
                if setting[0].startswith('CONFIG_PKG_'):
                    pkg_prefix = setting[0][11:]
                    if pkg_prefix.startswith('USING_'):
                        pkg_name = pkg_prefix[6:]
                    else:
                        if pkg_prefix.endswith('_PATH'):
                            pkg_name = pkg_prefix[:-5]
                            pkg_path = setting[1]
                            if pkg_path.startswith('"'):
                                pkg_path = pkg_path[1:]
                            if pkg_path.endswith('"'):
                                pkg_path = pkg_path[:-1]
                            pkgs_path(ret, pkg_name, pkg_path)

                        if pkg_prefix.endswith('_VER'):
                            pkg_name = pkg_prefix[:-4]
                            pkg_ver = setting[1]
                            if pkg_ver.startswith('"'):
                                pkg_ver = pkg_ver[1:]
                            if pkg_ver.endswith('"'):
                                pkg_ver = pkg_ver[:-1]
                            pkgs_ver(ret, pkg_name, pkg_ver)

    config.close()
    return ret