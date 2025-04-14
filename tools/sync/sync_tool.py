#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SiFli SDK同步工具
提取tools.json中的下载链接、替换GitHub链接、下载文件并上传到腾讯云COS
"""

import os
import json
import re
import argparse
import requests
import hashlib
from pathlib import Path
from tqdm import tqdm
from concurrent.futures import ThreadPoolExecutor, as_completed
from urllib.parse import urlparse, unquote
from qcloud_cos import CosConfig, CosS3Client

# 常量
TOOLS_JSON_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools.json")
DOWNLOAD_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "downloads")
GITHUB_PATTERN = re.compile(r'^https://github\.com/')
GITHUB_MIRROR = "downloads.sifli.com/github_assets"

def read_tools_json():
    """读取tools.json文件"""
    try:
        with open(TOOLS_JSON_PATH, 'r', encoding='utf-8') as file:
            return json.load(file)
    except Exception as e:
        print(f"读取tools.json失败: {e}")
        return None

def extract_download_urls(tools_data):
    """提取所有下载链接"""
    urls = []
    if not tools_data or 'tools' not in tools_data:
        return urls

    for tool in tools_data['tools']:
        for version in tool.get('versions', []):
            for platform, info in version.items():
                if isinstance(info, dict) and 'url' in info:
                    urls.append({
                        'url': info['url'],
                        'sha256': info.get('sha256', ''),
                        'size': info.get('size', 0),
                        'tool_name': tool.get('name', ''),
                        'version': version.get('name', ''),
                        'platform': platform
                    })
    
    print(f"从tools.json中提取了 {len(urls)} 个下载链接")
    return urls

def replace_github_url(url):
    """将GitHub链接替换为镜像链接"""
    if GITHUB_PATTERN.match(url):
        return GITHUB_PATTERN.sub(f"https://{GITHUB_MIRROR}/", url)
    return url

def get_filename_from_url(url):
    """从URL中提取文件名"""
    parsed_url = urlparse(url)
    return os.path.basename(unquote(parsed_url.path))

def verify_file_hash(file_path, expected_sha256):
    """验证文件的SHA256哈希值"""
    if not expected_sha256:
        return True
    
    sha256_hash = hashlib.sha256()
    with open(file_path, "rb") as f:
        # 读取文件块并更新哈希
        for byte_block in iter(lambda: f.read(4096), b""):
            sha256_hash.update(byte_block)
    
    calculated_hash = sha256_hash.hexdigest().upper()
    return calculated_hash == expected_sha256.upper()

def download_file(url_info):
    """下载单个文件"""
    url = url_info['url']
    filename = get_filename_from_url(url)
    output_path = os.path.join(DOWNLOAD_DIR, filename)
    
    # 如果文件已存在且哈希值匹配，跳过下载
    if os.path.exists(output_path) and verify_file_hash(output_path, url_info.get('sha256', '')):
        print(f"文件已存在且哈希值匹配: {filename}")
        return output_path
    
    try:
        response = requests.get(url, stream=True)
        response.raise_for_status()
        
        total_size = int(response.headers.get('content-length', 0))
        block_size = 8192  # 8KB
        
        with open(output_path, 'wb') as file, tqdm(
                desc=filename,
                total=total_size,
                unit='B',
                unit_scale=True,
                unit_divisor=1024,
        ) as progress_bar:
            for data in response.iter_content(block_size):
                file.write(data)
                progress_bar.update(len(data))
        
        # 验证下载的文件
        if url_info.get('sha256') and not verify_file_hash(output_path, url_info['sha256']):
            print(f"警告: 文件 {filename} 的哈希值不匹配!")
        
        return output_path
    except Exception as e:
        print(f"下载 {url} 失败: {e}")
        if os.path.exists(output_path):
            os.remove(output_path)
        return None

def download_all_files(urls_info):
    """并行下载所有文件"""
    os.makedirs(DOWNLOAD_DIR, exist_ok=True)
    downloaded_files = []
    
    with ThreadPoolExecutor(max_workers=5) as executor:
        future_to_url = {executor.submit(download_file, url_info): url_info for url_info in urls_info}
        for future in as_completed(future_to_url):
            url_info = future_to_url[future]
            try:
                file_path = future.result()
                if file_path:
                    downloaded_files.append({
                        'file_path': file_path,
                        'url': url_info['url'],
                        'tool_name': url_info['tool_name'],
                        'version': url_info['version'],
                        'platform': url_info['platform']
                    })
            except Exception as e:
                print(f"处理 {url_info['url']} 时发生错误: {e}")
    
    return downloaded_files

def upload_to_cos(file_info, cos_client, bucket_name):
    """上传文件到腾讯云COS"""
    file_path = file_info['file_path']
    filename = os.path.basename(file_path)
    
    # 为GitHub链接创建特殊路径
    if GITHUB_PATTERN.match(file_info['url']):
        parsed_url = urlparse(file_info['url'])
        path_parts = parsed_url.path.strip('/').split('/')
        
        # 所有GitHub资源都以github_assets开头
        cos_key = f"github_assets/{'/'.join(path_parts)}"
    else:
        # 对于非GitHub链接，不做上传处理
        print(f"跳过非GitHub链接: {file_info['url']}")
        return
    
    try:
        with open(file_path, 'rb') as f:
            response = cos_client.put_object(
                Bucket=bucket_name,
                Body=f,
                Key=cos_key,
                EnableMD5=True
            )
        print(f"上传成功: {cos_key}")
        return True
    except Exception as e:
        print(f"上传文件 {filename} 到 COS 失败: {e}")
        return False

def update_tools_json(tools_data):
    """更新tools.json中的URL为镜像链接"""
    if not tools_data or 'tools' not in tools_data:
        return False
    
    modified = False
    
    for tool in tools_data['tools']:
        for version in tool.get('versions', []):
            for platform, info in version.items():
                if isinstance(info, dict) and 'url' in info:
                    original_url = info['url']
                    if GITHUB_PATTERN.match(original_url):
                        new_url = replace_github_url(original_url)
                        if new_url != original_url:
                            info['url'] = new_url
                            modified = True
    
    if modified:
        try:
            # 备份原始文件
            backup_path = f"{TOOLS_JSON_PATH}.bak"
            if not os.path.exists(backup_path):
                import shutil
                shutil.copy2(TOOLS_JSON_PATH, backup_path)
            
            # 写入更新后的文件
            with open(TOOLS_JSON_PATH, 'w', encoding='utf-8') as file:
                json.dump(tools_data, file, indent=2)
            print(f"已更新 {TOOLS_JSON_PATH} 中的GitHub链接")
            return True
        except Exception as e:
            print(f"更新tools.json失败: {e}")
            return False
    
    print("没有需要更新的GitHub链接")
    return False

def main():
    parser = argparse.ArgumentParser(description='SiFli SDK同步工具')
    parser.add_argument('--secret-id', required=True, help='腾讯云COS SecretId')
    parser.add_argument('--secret-key', required=True, help='腾讯云COS SecretKey')
    parser.add_argument('--region', default='ap-shanghai', help='腾讯云COS Region')
    parser.add_argument('--bucket', required=True, help='腾讯云COS Bucket名称')
    parser.add_argument('--download-only', action='store_true', help='仅下载文件，不上传到COS')
    parser.add_argument('--update-json', action='store_true', help='更新tools.json中的URL为镜像链接')
    args = parser.parse_args()
    
    # 读取tools.json
    tools_data = read_tools_json()
    if not tools_data:
        return
    
    # 提取下载链接
    urls_info = extract_download_urls(tools_data)
    if not urls_info:
        print("未找到下载链接")
        return
    
    # 下载所有文件
    downloaded_files = download_all_files(urls_info)
    print(f"成功下载了 {len(downloaded_files)} 个文件")
    
    # 如果需要上传到COS
    if not args.download_only:
        try:
            # 配置腾讯云COS
            config = CosConfig(
                Region=args.region,
                SecretId=args.secret_id,
                SecretKey=args.secret_key,
            )
            cos_client = CosS3Client(config)
            
            # 上传文件到COS
            upload_count = 0
            for file_info in downloaded_files:
                if upload_to_cos(file_info, cos_client, args.bucket):
                    upload_count += 1
            
            print(f"成功上传了 {upload_count} 个文件到腾讯云COS")
        except Exception as e:
            print(f"上传到COS时出错: {e}")
    
    # 如果需要更新tools.json
    if args.update_json:
        update_tools_json(tools_data)

if __name__ == "__main__":
    main()