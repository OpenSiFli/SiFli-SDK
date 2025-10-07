# -*- coding:utf-8 -*-
#
# File      : rt_pkg_ext.py
# This file is part of SiFli-SDK
# RT-Thread package management extension for sdk.py
#
# Change Logs:
# Date           Author          Notes
# 2025-09-22     SiFli           Add rt-pkg command for rt-thread package management
#

import os
import sys
from typing import Any, Dict

import click
from click.core import Context
from sdk_py_actions.tools import PropertyDict
import rt_pkg
import rt_pkg.rt_package_list
import rt_pkg.rt_package_printenv
import rt_pkg.rt_package_update
import rt_pkg.rt_package_upgrade
import rt_pkg.rt_package_utils
# import rt_pkg.rt_package_wizard

def setup_rt_pkg_environment():
    """Setup RT-Thread package environment."""
    # Set environment variables if not already set
    if 'SIFLI_SDK' not in os.environ and 'SIFLI_SDK_PATH' in os.environ:
        os.environ['SIFLI_SDK'] = os.environ['SIFLI_SDK_PATH']
    
    # Set RTT_CC if not already set
    if 'RTT_CC' not in os.environ:
        os.environ['RTT_CC'] = 'gcc'
    
    # Add rt_pkg module path to sys.path
    rt_pkg_path = os.path.join(os.environ.get('SIFLI_SDK_PATH', ''), 'tools', 'sdk_py_actions', 'rt_pkg')
    if rt_pkg_path not in sys.path:
        sys.path.insert(0, rt_pkg_path)


def action_extensions(base_actions: Dict, project_path: str) -> Any:
    """RT-Thread package management extension for sdk.py."""
    
    def update_callback(target_name: str, ctx: Context, args: PropertyDict, force: bool = False) -> None:
        """Update packages by menuconfig settings."""
        setup_rt_pkg_environment()
        try:
            rt_pkg.rt_package_update.package_update(force_update=force)
        except Exception as e:
            print(f"Error updating packages: {e}")
    
    def list_callback(target_name: str, ctx: Context, args: PropertyDict) -> None:
        """List target packages."""
        setup_rt_pkg_environment()
        try:
            rt_pkg.rt_package_list.list_packages()
        except Exception as e:
            print(f"Error listing packages: {e}")
    
    def wizard_callback(target_name: str, ctx: Context, args: PropertyDict) -> None:
        """Create a new package with wizard."""
        setup_rt_pkg_environment()
        try:
            rt_pkg.rt_package_wizard.package_wizard()
        except Exception as e:
            print(f"Error running package wizard: {e}")
    
    def upgrade_callback(target_name: str, ctx: Context, args: PropertyDict, force: bool = False, script: bool = False) -> None:
        """Upgrade local packages index."""
        setup_rt_pkg_environment()
        try:
            rt_pkg.rt_package_upgrade.package_upgrade(force_upgrade=force, upgrade_script=script)
        except Exception as e:
            print(f"Error upgrading packages: {e}")
    
    def upgrade_modules_callback(target_name: str, ctx: Context, args: PropertyDict) -> None:
        """Upgrade python modules."""
        setup_rt_pkg_environment()
        try:
            rt_pkg.rt_package_upgrade.package_upgrade_modules()
        except Exception as e:
            print(f"Error upgrading python modules: {e}")
    
    def printenv_callback(target_name: str, ctx: Context, args: PropertyDict) -> None:
        """Print environment variables."""
        setup_rt_pkg_environment()
        try:
            rt_pkg.rt_package_printenv.package_print_env()
        except Exception as e:
            print(f"Error printing environment: {e}")

    rt_pkg_actions = {
        'actions': {
            'rt-pkg-update': {
                'callback': update_callback,
                'help': 'Update packages by menuconfig settings.',
                'options': [
                    {
                        'names': ['--force'],
                        'help': 'Force update and clean packages.',
                        'is_flag': True,
                        'default': False,
                    },
                ],
            },
            'rt-pkg-list': {
                'callback': list_callback,
                'help': 'List target packages.',
            },
            'rt-pkg-wizard': {
                'callback': wizard_callback,
                'help': 'Create a new package with wizard.',
            },
            'rt-pkg-upgrade': {
                'callback': upgrade_callback,
                'help': 'Upgrade local packages index from git repository.',
                'options': [
                    {
                        'names': ['--force'],
                        'help': 'Force upgrade local packages index.',
                        'is_flag': True,
                        'default': False,
                    },
                    {
                        'names': ['--script'],
                        'help': 'Also upgrade Env script from git repository.',
                        'is_flag': True,
                        'default': False,
                    },
                ],
            },
            'rt-pkg-upgrade-modules': {
                'callback': upgrade_modules_callback,
                'help': 'Upgrade python modules (e.g. requests).',
            },
            'rt-pkg-printenv': {
                'callback': printenv_callback,
                'help': 'Print environment variables to check.',
            },
        }
    }

    return rt_pkg_actions