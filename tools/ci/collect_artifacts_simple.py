#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-

import os
import sys
import shutil
import time
import argparse
from pathlib import Path
import glob


class SimpleArtifactsCollector:
    def __init__(self, output_dir='merged_artifacts', failed_only=False):
        self.merged_dir = Path(output_dir)
        self.failed_only = failed_only
        self.job_status_map = self._load_job_status_map()
        
        # 确保根目录存在
        self.merged_dir.mkdir(parents=True, exist_ok=True)

    def _parse_status_file(self, status_file):
        """解析Job状态文件"""
        metadata = {}

        with open(status_file, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or '=' not in line:
                    continue
                key, value = line.split('=', 1)
                metadata[key] = value

        return metadata

    def _infer_status_from_log(self, job_name):
        """在状态文件缺失时，从日志尾部推断Job状态"""
        log_path = Path('ci_build_logs') / f'{job_name}.log'
        if not log_path.exists():
            return 'unknown'

        with open(log_path, 'rb') as f:
            f.seek(max(0, log_path.stat().st_size - 4096))
            tail = f.read().decode('utf-8', errors='ignore')

        if '构建失败' in tail or '❌' in tail:
            return 'failed'
        if '构建成功' in tail or '✅' in tail:
            return 'success'
        return 'unknown'

    def _load_job_status_map(self):
        """加载所有Job状态"""
        status_map = {}

        for status_file in glob.glob('job_status/*.env'):
            status_path = Path(status_file)
            job_name = status_path.stem
            metadata = self._parse_status_file(status_path)
            status = metadata.get('JOB_STATUS', '').strip().lower() or self._infer_status_from_log(job_name)

            status_map[job_name] = {
                'status': status,
                'metadata': metadata,
                'path': status_path,
            }

        return status_map

    def _get_job_status(self, job_name):
        """获取Job状态，优先使用状态文件，缺失时回退到日志推断"""
        status_info = self.job_status_map.get(job_name)
        if status_info:
            return status_info['status']
        return self._infer_status_from_log(job_name)

    def _should_collect_job(self, job_name):
        """判断当前Job是否应该被收集"""
        if not self.failed_only:
            return True
        return self._get_job_status(job_name) == 'failed'

    def _ensure_job_entry(self, collected_jobs, job_name):
        """确保Job统计项存在"""
        if job_name not in collected_jobs:
            collected_jobs[job_name] = {
                'logs': 0,
                'artifacts': 0,
                'status': self._get_job_status(job_name)
            }
        return collected_jobs[job_name]

    def _copy_status_file(self, job_name):
        """复制Job状态文件到合并目录"""
        status_info = self.job_status_map.get(job_name)
        if not status_info:
            return

        job_dir = self.merged_dir / job_name
        status_subdir = job_dir / 'job_status'
        status_subdir.mkdir(parents=True, exist_ok=True)

        dest_file = status_subdir / status_info['path'].name
        if not dest_file.exists():
            shutil.copy2(status_info['path'], dest_file)
    
    def collect_local_artifacts(self):
        collected_jobs = {}
        
        # 收集日志文件并映射到对应的job
        log_pattern = "ci_build_logs/*.log"
        log_files = glob.glob(log_pattern, recursive=True)
        
        print(f"📄 发现 {len(log_files)} 个日志文件")
        for log_file in log_files:
            log_path = Path(log_file)
            job_name = log_path.stem  # 去掉.log后缀

            if not self._should_collect_job(job_name):
                continue
            
            # 为每个job创建目录
            job_dir = self.merged_dir / job_name
            job_dir.mkdir(exist_ok=True)
            
            # 创建ci_build_logs子目录
            logs_subdir = job_dir / 'ci_build_logs'
            logs_subdir.mkdir(exist_ok=True)
            
            # 复制日志文件
            dest_file = logs_subdir / log_path.name
            shutil.copy2(log_file, dest_file)
            print(f"  📄 复制: {job_name}/ci_build_logs/{log_path.name}")
            
            self._copy_status_file(job_name)
            job_stats = self._ensure_job_entry(collected_jobs, job_name)
            job_stats['logs'] += 1
        
        # 收集artifacts目录中的文件
        artifacts_base = Path('artifacts')
        if artifacts_base.exists():
            for job_artifacts_dir in artifacts_base.iterdir():
                if job_artifacts_dir.is_dir():
                    job_name = job_artifacts_dir.name

                    if not self._should_collect_job(job_name):
                        continue
                    
                    # 为每个job创建目录
                    job_dir = self.merged_dir / job_name
                    job_dir.mkdir(exist_ok=True)
                    
                    # 创建artifacts子目录
                    artifacts_subdir = job_dir / 'artifacts'
                    artifacts_subdir.mkdir(exist_ok=True)
                    
                    # 复制整个artifacts子目录内容
                    dest_job_artifacts = artifacts_subdir / job_name
                    if dest_job_artifacts.exists():
                        shutil.rmtree(dest_job_artifacts)
                    shutil.copytree(job_artifacts_dir, dest_job_artifacts)
                    
                    # 统计文件数量
                    artifact_files = [f for f in dest_job_artifacts.rglob('*') if f.is_file()]
                    file_count = len(artifact_files)
                    print(f"  📦 复制: {job_name}/artifacts/{job_name}/ ({file_count} 个文件)")
                    
                    self._copy_status_file(job_name)
                    job_stats = self._ensure_job_entry(collected_jobs, job_name)
                    job_stats['artifacts'] = file_count

        # 补齐只有状态文件但没有日志/产物的Job
        for job_name in sorted(self.job_status_map.keys()):
            if not self._should_collect_job(job_name):
                continue
            self._ensure_job_entry(collected_jobs, job_name)
            self._copy_status_file(job_name)
        
        return collected_jobs
    
    def create_summary_report(self, collected_jobs):
        report_path = self.merged_dir / 'build_summary.txt'
        
        total_logs = sum(job['logs'] for job in collected_jobs.values())
        total_artifacts = sum(job['artifacts'] for job in collected_jobs.values())
        
        with open(report_path, 'w', encoding='utf-8') as f:
            report_title = "SDK失败构建汇总报告" if self.failed_only else "SDK构建汇总报告"
            f.write(f"{report_title}\n")
            f.write("=" * 50 + "\n")
            f.write(f"Pipeline ID: {os.getenv('CI_PIPELINE_ID', 'N/A')}\n")
            f.write(f"项目ID: {os.getenv('CI_PROJECT_ID', 'N/A')}\n")
            f.write(f"构建时间: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            f.write(f"汇总统计:\n")
            f.write(f"  - 构建任务数: {len(collected_jobs)} 个\n")
            f.write(f"  - 构建日志: {total_logs} 个\n")
            f.write(f"  - 构建产物: {total_artifacts} 个\n\n")
            
            f.write("各Job详细统计:\n")
            f.write("-" * 50 + "\n")
            for job_name, stats in sorted(collected_jobs.items()):
                f.write(f"{job_name}:\n")
                f.write(f"  - 状态: {stats.get('status', 'unknown')}\n")
                f.write(f"  - 日志文件: {stats['logs']} 个\n")
                f.write(f"  - 构建产物: {stats['artifacts']} 个\n")
                
                # 列出具体文件
                job_dir = self.merged_dir / job_name
                if job_dir.exists():
                    log_files = list((job_dir / 'ci_build_logs').glob('*.log')) if (job_dir / 'ci_build_logs').exists() else []
                    if log_files:
                        f.write(f"  日志文件列表:\n")
                        for log_file in sorted(log_files):
                            size = log_file.stat().st_size
                            f.write(f"    - {log_file.name} ({size:,} bytes)\n")
                    
                    artifacts_dir = job_dir / 'artifacts' / job_name
                    if artifacts_dir.exists():
                        artifact_files = list(artifacts_dir.rglob('*'))
                        artifact_files = [f for f in artifact_files if f.is_file()]
                        if artifact_files:
                            f.write(f"  构建产物列表:\n")
                            for artifact_file in sorted(artifact_files):
                                size = artifact_file.stat().st_size
                                rel_path = artifact_file.relative_to(artifacts_dir)
                                f.write(f"    - {rel_path} ({size:,} bytes)\n")
                f.write("\n")
            
            # 计算总大小
            total_size = sum(f.stat().st_size for f in self.merged_dir.rglob('*') if f.is_file())
            f.write(f"总大小: {total_size:,} bytes ({total_size / (1024*1024):.2f} MB)\n")
        
        print(f"📄 汇总报告已生成: {report_path}")
    
    def create_file_index(self, collected_jobs):
        index_path = self.merged_dir / 'file_index.json'
        
        import json
        
        index_data = {
            "generated_at": time.strftime('%Y-%m-%d %H:%M:%S'),
            "pipeline_id": os.getenv('CI_PIPELINE_ID', 'N/A'),
            "project_id": os.getenv('CI_PROJECT_ID', 'N/A'),
            "merge_strategy": "failed_only" if self.failed_only else "all",
            "jobs": {}
        }
        
        # 为每个job创建索引
        for job_name in collected_jobs.keys():
            job_dir = self.merged_dir / job_name
            if not job_dir.exists():
                continue
            
            job_index = {
                "status": collected_jobs[job_name].get('status', 'unknown'),
                "logs": [],
                "artifacts": []
            }
            
            # 索引日志文件
            logs_dir = job_dir / 'ci_build_logs'
            if logs_dir.exists():
                for log_file in sorted(logs_dir.glob('*.log')):
                    job_index["logs"].append({
                        "name": log_file.name,
                        "size": log_file.stat().st_size,
                        "path": f"{job_name}/ci_build_logs/{log_file.name}"
                    })
            
            # 索引构建产物
            artifacts_dir = job_dir / 'artifacts' / job_name
            if artifacts_dir.exists():
                for artifact_file in sorted(artifacts_dir.rglob('*')):
                    if artifact_file.is_file():
                        rel_path = artifact_file.relative_to(artifacts_dir)
                        job_index["artifacts"].append({
                            "name": artifact_file.name,
                            "size": artifact_file.stat().st_size,
                            "path": f"{job_name}/artifacts/{job_name}/{rel_path}",
                            "type": artifact_file.suffix.lstrip('.')
                        })
            
            index_data["jobs"][job_name] = job_index
        
        with open(index_path, 'w', encoding='utf-8') as f:
            json.dump(index_data, f, indent=2, ensure_ascii=False)
        
        print(f"📋 文件索引已生成: {index_path}")
    
    def collect_artifacts(self):
        # 显示当前目录结构（用于调试）
        print("📁 当前目录结构:")
        for item in sorted(Path('.').rglob('*')):
            if item.is_file() and any(str(item).startswith(prefix) for prefix in ['ci_build_logs', 'artifacts', 'job_status']):
                print(f"  {item}")

        print(f"📌 发现 {len(self.job_status_map)} 个Job状态文件")
        if self.failed_only:
            print("🚨 当前运行模式: 仅收集失败Job")
        else:
            print("📦 当前运行模式: 收集全部Job")
        
        # 收集本地artifacts并保持原始目录结构
        collected_jobs = self.collect_local_artifacts()
        
        # 创建报告和索引
        self.create_summary_report(collected_jobs)
        self.create_file_index(collected_jobs)
        
        total_jobs = len(collected_jobs)
        total_logs = sum(job['logs'] for job in collected_jobs.values())
        total_artifacts = sum(job['artifacts'] for job in collected_jobs.values())
        
        print(f"\n✅ artifacts收集完成!")
        print(f"📊 统计: {total_jobs} 个job, {total_logs} 个日志文件, {total_artifacts} 个构建产物")
        
        # 显示合并后的目录结构
        print(f"\n📁 合并后的artifacts结构:")
        for job_dir in sorted(self.merged_dir.iterdir()):
            if job_dir.is_dir():
                job_status = collected_jobs.get(job_dir.name, {}).get('status', 'unknown')
                print(f"  📂 {job_dir.name}/ [{job_status}]")
                
                # 显示ci_build_logs目录
                logs_dir = job_dir / 'ci_build_logs'
                if logs_dir.exists():
                    log_files = list(logs_dir.glob('*.log'))
                    if log_files:
                        print(f"    📂 ci_build_logs/ ({len(log_files)} 个日志文件)")
                
                # 显示artifacts目录
                artifacts_dir = job_dir / 'artifacts'
                if artifacts_dir.exists():
                    artifact_files = list(artifacts_dir.rglob('*'))
                    artifact_files = [f for f in artifact_files if f.is_file()]
                    if artifact_files:
                        print(f"    📂 artifacts/ ({len(artifact_files)} 个构建产物)")
        
        # 显示汇总文件
        for summary_file in ['build_summary.txt', 'file_index.json']:
            summary_path = self.merged_dir / summary_file
            if summary_path.exists():
                size = summary_path.stat().st_size
                print(f"  📄 {summary_file} ({size:,} bytes)")


def main():
    try:
        parser = argparse.ArgumentParser(description='收集本地构建artifacts')
        parser.add_argument('--failed-only', action='store_true', help='仅收集失败Job的artifacts')
        parser.add_argument('--output-dir', default='merged_artifacts', help='输出目录')
        args = parser.parse_args()

        collector = SimpleArtifactsCollector(output_dir=args.output_dir, failed_only=args.failed_only)
        collector.collect_artifacts()
        print("\n🎉 Artifacts收集成功完成！")
    except Exception as e:
        print(f"❌ 收集artifacts失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
