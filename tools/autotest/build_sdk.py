# -*- coding: utf-8 -*-
# SPDX-FileCopyrightText: 2025-2026 SiFli
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import print_function
import argparse
import datetime
import os
import subprocess
import sys
import yaml
import shutil
import codecs
import tempfile
import platform
import glob
import fnmatch
import concurrent.futures
import threading


class SDKBuilder(object):
    def __init__(self, config_file, compiler="keil", max_workers=None):
        self.config = self._load_config(config_file)
        self.compiler = compiler
        self.root_dir = os.getcwd()
        self.failed_projects = []
        self.build_log_dir = os.path.join(self.root_dir, self.config['common']['log_dir'])
        if not os.path.exists(self.build_log_dir):
            os.makedirs(self.build_log_dir)
        self.is_windows = platform.system() == 'Windows'
        # 设置多线程构建的工作线程数，如果未指定则使用 CPU 核心数
        self.max_workers = max_workers if max_workers else os.cpu_count()
        # 添加线程锁用于保护 failed_projects 列表
        self.lock = threading.Lock()

    def _load_config(self, config_file):
        with codecs.open(config_file, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)

    def _create_build_script(self, commands, work_dir=None, log_file=None):
        """创建构建脚本，根据不同的操作系统创建.bat或.sh文件"""
        if self.is_windows:
            return self._create_windows_script(commands, work_dir, log_file)
        else:
            return self._create_unix_script(commands, work_dir, log_file)

    def _create_windows_script(self, commands, work_dir=None, log_file=None):
        """创建Windows构建脚本(.bat)"""
        fd, path = tempfile.mkstemp(suffix='.bat')
        try:
            with os.fdopen(fd, 'w') as f:
                f.write('@echo off\n')
                if log_file:
                    log_dir = os.path.dirname(log_file)
                    if log_dir:
                        f.write(f'if not exist "{log_dir}" mkdir "{log_dir}"\n')

                f.write(f'call "{os.path.join(self.root_dir, "set_env.bat")}" {self.compiler}\n')

                if work_dir:
                    f.write(f'cd /d "{work_dir}"\n')

                for cmd in commands:
                    if log_file:
                        f.write(f'echo [%date% %time%] Executing: {cmd} >> "{log_file}"\n')
                        f.write(f'{cmd} >> "{log_file}" 2>&1\n')
                        f.write('set BUILD_ERROR=%ERRORLEVEL%\n')
                        f.write('if %BUILD_ERROR% neq 0 (\n')
                        f.write(
                            f'    echo [%date% %time%] Command failed with error code %BUILD_ERROR% >> "{log_file}"\n')
                        f.write('    exit /b %BUILD_ERROR%\n')
                        f.write(')\n')
                    else:
                        f.write(f'{cmd}\n')
            return path
        except Exception as e:
            os.remove(path)
            raise e

    def _create_unix_script(self, commands, work_dir=None, log_file=None):
        """创建Unix构建脚本(.sh)，适用于Linux和macOS"""
        fd, path = tempfile.mkstemp(suffix='.sh')
        try:
            with os.fdopen(fd, 'w') as f:
                f.write('#!/bin/bash\n')
                
                if log_file:
                    log_dir = os.path.dirname(log_file)
                    if log_dir:
                        f.write(f'mkdir -p "{log_dir}"\n')

                # 使用export.sh脚本设置环境
                f.write(f'source "{os.path.join(self.root_dir, "export.sh")}" {self.compiler}\n')

                if work_dir:
                    f.write(f'cd "{work_dir}" || exit 1\n')

                for cmd in commands:
                    if log_file:
                        f.write(f'echo "[$(date)] Executing: {cmd}" >> "{log_file}"\n')
                        f.write(f'{cmd} >> "{log_file}" 2>&1\n')
                        f.write('BUILD_ERROR=$?\n')
                        f.write('if [ $BUILD_ERROR -ne 0 ]; then\n')
                        f.write(f'    echo "[$(date)] Command failed with error code $BUILD_ERROR" >> "{log_file}"\n')
                        f.write('    exit $BUILD_ERROR\n')
                        f.write('fi\n')
                    else:
                        f.write(f'{cmd}\n')
            # 确保脚本可执行
            os.chmod(path, 0o755)
            return path
        except Exception as e:
            os.remove(path)
            raise e

    def _run_commands(self, commands, work_dir=None, log_file=None):
        batch_file = self._create_build_script(commands, work_dir, log_file)
        try:
            process = subprocess.Popen(
                f'cmd.exe /c "{batch_file}"' if self.is_windows else f'bash "{batch_file}"',
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                shell=True
            )
            stdout, stderr = process.communicate()

            if log_file and os.path.exists(log_file):
                with open(log_file, 'a') as f:
                    f.write('\n=== Additional Build Output ===\n')
                    if stdout:
                        f.write(f'=== STDOUT ===\n{stdout}\n')
                    if stderr:
                        f.write(f'=== STDERR ===\n{stderr}\n')

            return process.returncode, stdout, stderr
        finally:
            try:
                os.remove(batch_file)
            except:
                pass
    
    def _copy_artifacts(self, src_path, dst_path, patterns=None, fail_log_file=None):
        if not os.path.exists(src_path):
            err_msg = f"Source path does not exist: {src_path}"
            print(err_msg)
            if fail_log_file:
                with codecs.open(fail_log_file, 'a', encoding='utf-8') as f:
                    f.write(f"{err_msg}\n")
            return False

        # 确保目标目录存在
        if not os.path.exists(dst_path):
            try:
                os.makedirs(dst_path)
            except Exception as e:
                err_msg = f"Failed to create destination directory {dst_path}: {str(e)}"
                print(err_msg)
                if fail_log_file:
                    with codecs.open(fail_log_file, 'a', encoding='utf-8') as f:
                        f.write(f"{err_msg}\n")
                return False

        # 默认复制所有文件
        if patterns is None:
            patterns = ["*.*"]

        copied_files = []
        try:
            # 遍历源目录中的所有文件
            for root, _, files in os.walk(src_path):
                # 计算相对路径
                rel_path = os.path.relpath(root, src_path)
                dst_root = os.path.join(dst_path, rel_path) if rel_path != "." else dst_path
                
                # 确保目标子目录存在
                if not os.path.exists(dst_root):
                    os.makedirs(dst_root)
                
                # 复制匹配的文件
                for file in files:
                    if any(fnmatch.fnmatch(file, pattern) for pattern in patterns):
                        src_file = os.path.join(root, file)
                        dst_file = os.path.join(dst_root, file)
                        shutil.copy2(src_file, dst_file)
                        copied_files.append(dst_file)
            
            print(f"Copied {len(copied_files)} files to {dst_path}")
            return True
        except Exception as e:
            err_msg = f"Error copying files: {str(e)}"
            print(err_msg)
            if fail_log_file:
                with codecs.open(fail_log_file, 'a', encoding='utf-8') as f:
                    f.write(f"{err_msg}\n")
            return False

    def build_project(self, project_path, board=None, cpu=None, is_common=False):
        try:
            if is_common:
                cmd = f'scons --board={board} -j8'
            else:
                cmd = 'scons -j8'

            if cpu:
                project_path = os.path.join(project_path, cpu)

            # timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            project_path_normalized = project_path.replace('\\', '/')

            project_name = project_path_normalized.replace('/', '_').replace(':', '')
            if board:
                log_file = os.path.join(self.build_log_dir, f'{project_name}_{board}.log')
            else:
                log_file = os.path.join(self.build_log_dir, f'{project_name}.log')

            print(f"\nBuilding {project_path} {('for ' + board) if board else ''}")
            print(f"Build log will be saved to: {project_name}")

            returncode, stdout, stderr = self._run_commands(
                [cmd],
                work_dir=os.path.abspath(project_path),
                log_file=log_file
            )

            if returncode != 0:
                # 使用线程锁保护对 failed_projects 的修改
                with self.lock:
                    self.failed_projects.append((project_path, board if board else "", log_file))
                print(f"xxxxx Build failed for {project_path} {('(' + board + ')') if board else ''}")
                print(f"xxxxx Error details can be found in: {project_name}")
                return False

            print(f"Successfully built {project_path} {('for ' + board) if board else ''}")
            
            # 复制构建产物
            print("-------- Copy artifacts --------")
            artifacts_dir = os.path.join(self.root_dir, self.config['common'].get('artifacts_dir', 'artifacts'))
            if not os.path.exists(artifacts_dir):
                os.makedirs(artifacts_dir)
                
            # 根据项目类型确定源路径和目标路径
            if not is_common:
                # 普通项目
                src_path = os.path.join(project_path_normalized, "build")
                dst_path = os.path.join(artifacts_dir, project_name)
                success = self._copy_artifacts(
                    src_path, 
                    dst_path,
                    patterns=["*.bin", "*.hex", "*.ini", "*.bat", "*.jlink"],
                    fail_log_file=log_file
                )
                if not success:
                    print(f"--- Copy artifacts failed ---: {project_path_normalized}")
            else:
                # 通用项目
                board_name = board if "hcpu" in board else f"{board}_hcpu"
                src_path = os.path.join(project_path, f"build_{board_name}")
                dst_path = os.path.join(artifacts_dir, project_name, board_name)
                success = self._copy_artifacts(
                    src_path, 
                    dst_path,
                    patterns=["*.bin", "*.hex", "*.ini", "*.bat", "*.jlink", "*.map", "*.axf", "*.elf"],
                    fail_log_file=log_file
                )
                if not success:
                    print(f"--- Copy common artifacts failed ---: {project_path} board={board_name}")
            
            return True

        except Exception as e:
            print(f"Error building {project_path}: {str(e)}")
            # 使用线程锁保护对 failed_projects 的修改
            with self.lock:
                self.failed_projects.append(
                    (project_path, board if board else "", log_file if 'log_file' in locals() else None))
            return False

    def build_all(self):
        """使用多线程执行构建任务"""
        print(f"\n--------- Building projects with {self.max_workers} parallel workers ---------")
        
        # 收集所有待构建任务
        build_tasks = []
        
        # 收集普通项目任务
        if "projects" in self.config:
            print("Scheduling normal projects...")
            for project in self.config['projects']:
                for prj_path in project['path']:
                    build_tasks.append((prj_path, None, None, False))
        
        # 收集通用项目任务
        if "common_projects" in self.config:
            print("Scheduling common projects...")
            for common_project in self.config['common_projects']:
                exclude_boards = set(common_project.get('exclude_boards', []))
                for board in common_project['boards']:
                    if board in exclude_boards:
                        continue
                    build_tasks.append((common_project['path'], board, None, True))
        
        # 创建线程池执行任务
        total_tasks = len(build_tasks)
        completed_tasks = 0
        success_tasks = 0
        
        print(f"Total tasks to build: {total_tasks}")
        
        # 使用线程池执行构建任务
        with concurrent.futures.ThreadPoolExecutor(max_workers=self.max_workers) as executor:
            # 提交所有任务
            futures = {executor.submit(self.build_project, task[0], task[1], task[2], task[3]): task for task in build_tasks}
            
            # 处理完成的任务
            for future in concurrent.futures.as_completed(futures):
                task = futures[future]
                project_path, board, _, _ = task
                completed_tasks += 1
                
                try:
                    result = future.result()
                    if result:
                        success_tasks += 1
                    progress = f"[{completed_tasks}/{total_tasks}]"
                    status = "Success" if result else "Failed"
                    project_info = f"{project_path} {('for ' + board) if board else ''}"
                    print(f"{progress} {status}: {project_info}")
                except Exception as e:
                    with self.lock:
                        self.failed_projects.append((project_path, board if board else "", None))
                    print(f"[{completed_tasks}/{total_tasks}] Error: {project_path} - {str(e)}")
        
        print(f"\nBuild summary: {success_tasks} succeeded, {total_tasks - success_tasks} failed")

    def generate_report(self):
        log_dir = os.path.join(self.config['common']['report_dir'])
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        report_path = os.path.join(
            self.config['common']['report_dir'],
            'build_report.txt'
        )
        with codecs.open(report_path, 'w', encoding='utf-8') as f:
            f.write("SDK Build Report\n")
            f.write("=" * 50 + "\n")
            f.write(f"Build Time: {datetime.datetime.now()}\n\n")
            f.write("=" * 50 + "\n")
            if self.failed_projects:
                f.write("Failed Projects:\n")
                for project, board, log_file in self.failed_projects:
                    f.write(f"- {project} {('(' + board + ')') if board else ''}\n")
                    # if log_file:
                    #     f.write(f"  Log file: {log_file}\n")
                f.write("=" * 50 + "\n\n")
            else:
                f.write("All projects built successfully!\n")

            # f.write(f"\nBuild logs directory: {self.build_log_dir}\n")

            f.write("All build projects:\n\n")
            f.write("--------- normal projects ---------\n")
            if "projects" in self.config:
                for project in self.config['projects']:
                    f.write(f"\n{project['group']}\n")
                    for prj_path in project['path']:
                        f.write(f"- {prj_path}\n")
            f.write("\n---------  common projects ---------\n")
            if "common_projects" in self.config:
                for common_project in self.config['common_projects']:
                    f.write(f"\n{common_project['path']}\n")
                    exclude_boards = set(common_project.get('exclude_boards', []))
                    for board in common_project['boards']:
                        if board in exclude_boards:
                            continue
                        f.write(f"- {board}\n")
            f.write("\n")
            f.write("=" * 50 + "\n")
        print(f"Build report generated at {report_path}")

    def backup_failed_logs(self):
        if not self.failed_projects:
            return

        backup_dir = os.path.join(self.root_dir, self.config['common']['report_dir'])
        if not os.path.exists(backup_dir):
            os.makedirs(backup_dir)

        for project, board, log_file in self.failed_projects:
            if log_file and os.path.exists(log_file):
                backup_file = os.path.join(backup_dir, os.path.basename(log_file))
                shutil.copy2(log_file, backup_file)


def main():
    parser = argparse.ArgumentParser(description='SDK Builder')
    parser.add_argument('-c', '--config', default='build_config.yaml',
                        help='Path to the build configuration file')
    parser.add_argument('-p', '--compiler', default='keil',
                        help='Compiler')                        
    parser.add_argument('-j', '--jobs', type=int, default=None,
                        help='Number of parallel build jobs (default: number of CPU cores)')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Enable verbose output')
    args = parser.parse_args()

    try:
        builder = SDKBuilder(args.config, args.compiler, args.jobs)
        builder.build_all()
        builder.generate_report()
        builder.backup_failed_logs()

        if builder.failed_projects:
            print("\n--------- Build completed with failures. See report for details. ---------")
            sys.exit(1)
        else:
            print("\n--------- Build completed successfully! ---------")
            sys.exit(0)

    except Exception as e:
        print(f"Error during build process: {str(e)}")
        sys.exit(1)


if __name__ == '__main__':
    main()
