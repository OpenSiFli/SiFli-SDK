# -*- coding:utf-8 -*-
#
# File      : package.py
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
# 2018-12-28     Ernest Chen     Add package information and enjoy package maker
# 2020-4-7       SummerGift      Code improvement
#

import json
import logging
import os
import sys
import requests
import rt_pkg.archive as archive
try:
    from tqdm import tqdm
except ImportError:
    # Fallback if tqdm is not available
    def tqdm(iterable, *args, **kwargs):
        return iterable

"""Template for creating a new file"""

Bridge_SConscript = '''import os
from building import *

objs = []
cwd  = GetCurrentDir()
list = os.listdir(cwd)

for item in list:
    if os.path.isfile(os.path.join(cwd, item, 'SConscript')):
        objs = objs + SConscript(os.path.join(item, 'SConscript'))

Return('objs')
'''

Kconfig_file = '''
# Kconfig file for package ${lowercase_name}
menuconfig PKG_USING_${name}
    bool "${description}"
    default n

if PKG_USING_${name}

    config PKG_${name}_PATH
        string
        default "/packages/${pkgs_class}/${lowercase_name}"

    choice
        prompt "Version"
        help
            Select the package version

        config PKG_USING_${name}_V${version_standard}
            bool "v${version}"

        config PKG_USING_${name}_LATEST_VERSION
            bool "latest"
    endchoice

    config PKG_${name}_VER
       string
       default "v${version}"    if PKG_USING_${name}_V${version_standard}
       default "latest"    if PKG_USING_${name}_LATEST_VERSION

endif

'''

Package_json_file = '''{
  "name": "${name}",
  "description": "${description}",
  "description_zh": "${description_zh}",
  "enable": "PKG_USING_${pkgs_using_name}",
  "keywords": [
    "${keyword}"
  ],
  "category": "${pkgsclass}",
  "author": {
    "name": "${authorname}",
    "email": "${authoremail}",
    "github": "${authorname}"
  },
  "license": "${license}",
  "repository": "${repository}",
  "icon": "unknown",
  "homepage": "${repository}#readme",
  "doc": "unknown",
  "site": [
    {
      "version": "v${version}",
      "URL": "https://${name}-${version}.zip",
      "filename": "${name}-${version}.zip"
    },
    {
      "version": "latest",
      "URL": "${repository}.git",
      "filename": "",
      "VER_SHA": "master"
    }
  ]
}
'''

import codecs


class PackageOperation:
    pkg = None

    def parse(self, filename):
        with codecs.open(filename, "r", encoding='utf-8') as f:
            json_str = f.read()

        if json_str:
            self.pkg = json.loads(json_str)

    def get_name(self):
        return self.pkg['name']

    def get_filename(self, ver):
        for item in self.pkg['site']:
            if item['version'].lower() == ver.lower():
                return item['filename']

        return None

    def get_url(self, ver):
        url = None
        for item in self.pkg['site']:
            if item['version'].lower() == ver.lower():
                url = item['URL']

        if not url:
            logging.warning("Can't find right url {0}, please check {1}".format(ver.lower(), self.pkg['site']))

        return url

    def get_versha(self, ver):
        for item in self.pkg['site']:
            if item['version'].lower() == ver.lower():
                return item['VER_SHA']

        return None

    def get_site(self, ver):
        for item in self.pkg['site']:
            if item['version'].lower() == ver.lower():
                return item

        return None

    def download(self, ver, path, url_from_srv):
        ret = True
        url = self.get_url(ver)
        site = self.get_site(ver)
        if site and 'filename' in site:
            filename = site['filename']
            path = os.path.join(path, filename)
        else:
            basename = os.path.basename(url)
            path = os.path.join(path, basename)

        if os.path.isfile(path):
            if not os.path.getsize(path):
                os.remove(path)
            else:
                if archive.package_integrity_test(path):
                    return True
                else:
                    os.remove(path)

        retry_count = 0

        headers = {'Connection': 'keep-alive', 'Accept-Encoding': 'gzip, deflate', 'Accept': '*/*', 'User-Agent': 'curl/7.54.0'}

        print('downloading ' + filename + ' ...')

        while True:
            try:
                r = requests.get(url_from_srv, stream=True, headers=headers)
                total_size = int(r.headers.get('content-length', 0))

                with open(path, 'wb') as f, tqdm(total=total_size, unit='B', unit_scale=True) as bar:
                    # if the chunk_size is too large, the progress bar will not display
                    for chunk in r.iter_content(chunk_size=1024):
                        if chunk:
                            f.write(chunk)
                            bar.update(len(chunk))

                retry_count = retry_count + 1

                if archive.package_integrity_test(path):  # make sure the file is right
                    ret = True
                    break
                else:
                    if os.path.isfile(path):
                        os.remove(path)
                    if retry_count > 5:
                        print("error: Have tried downloading 5 times.\nstop Downloading file :%s" % path)
                        if os.path.isfile(path):
                            os.remove(path)
                        ret = False
                        break
            except Exception as e:
                print(url_from_srv)
                print('error message:%s\t' % e)
                retry_count = retry_count + 1
                if retry_count > 5:
                    print('%s download fail!\n' % path.encode("utf-8"))
                    if os.path.isfile(path):
                        os.remove(path)
                    return False
        return ret