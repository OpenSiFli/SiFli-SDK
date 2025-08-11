# -*- coding: utf-8 -*-
import os
import shutil
import sys


class BoardDocCopier:
    README_FILENAME = "README.md"
    README_EN_FILENAME = "README_EN.md"
    INDEX_FILENAME = "index.md"
    INDEX_EN_FILENAME = "index_EN.md"
    
    def __init__(self, lang='zh_CN'):
        self.lang = lang
        self.readme_filename = self._get_readme_filename()
    
    def _get_readme_filename(self):
        return self.README_EN_FILENAME if self.lang == 'en' else self.README_FILENAME
    
    def _copy_assets_if_exists(self, src_dir, target_dir):
        """
        通用方法：复制assets目录（如果存在）
        """
        asset_dir = os.path.join(src_dir, "assets")
        if os.path.isdir(asset_dir):
            target_asset_dir = os.path.join(target_dir, "assets")
            print("copy assets from {} to {}".format(asset_dir, target_asset_dir))
            shutil.copytree(asset_dir, target_asset_dir, dirs_exist_ok=True)
    
    def copy_files(self, src, tgt):
        readme_path = os.path.join(src, self.readme_filename)
        if not os.path.exists(readme_path):
            return
        
        target_dir = os.path.join(tgt, src)
        print("mkdir: {}".format(target_dir))
        os.makedirs(target_dir, exist_ok=True)

        # 拷贝assets目录（使用通用方法）
        self._copy_assets_if_exists(src, target_dir)
        
        # 拷贝README文件并重命名为README.md（统一输出文件名）
        print("copy {}".format(self.readme_filename))    
        shutil.copy(readme_path, os.path.join(target_dir, self.README_FILENAME))

    def copy_index_file(self, src, tgt, skip_customer=False):
        indexfile_path = os.path.join(src, self.INDEX_FILENAME)
        if not os.path.exists(indexfile_path):
            return

        # 如果是customer目录且需要跳过，则不创建customer层级
        if skip_customer and os.path.basename(src) == "customer":
            target_dir = tgt
        else:
            target_dir = os.path.join(tgt, src)
            
        print("mkdir: {}".format(target_dir))
        os.makedirs(target_dir, exist_ok=True)

        # 复制assets目录（如果存在）
        self._copy_assets_if_exists(src, target_dir)

        if self.lang != 'zh_CN':
            if os.path.exists(os.path.join(src, self.INDEX_EN_FILENAME)):
                indexfile_path = os.path.join(src, self.INDEX_EN_FILENAME)
                print("copy {}".format(self.INDEX_EN_FILENAME))
                target_file = os.path.join(target_dir, self.INDEX_FILENAME)
                shutil.copy(indexfile_path, target_file)
                return

        print("copy {}".format(self.INDEX_FILENAME))
        target_file = os.path.join(target_dir, self.INDEX_FILENAME)
        shutil.copy(indexfile_path, target_file)

    def is_board_doc_dir(self, path):
        """
        检查是否是板子文档目录
        路径格式应为：customer/boards/板子型号/doc
        并且包含README文件
        """
        # 检查路径是否以doc结尾
        if not os.path.basename(path) == "doc":
            return False
        
        # 检查路径结构：应该是 .../customer/boards/板子型号/doc
        path_parts = os.path.normpath(path).split(os.sep)
        if len(path_parts) >= 4:
            # 检查倒数第4个是customer，倒数第3个是boards，倒数第1个是doc
            if (path_parts[-4] == "customer" and 
                path_parts[-3] == "boards" and 
                path_parts[-1] == "doc"):
                
                # 列出doc目录的内容
                print(f"Found doc directory: {path}")
                try:
                    files = os.listdir(path)
                    print(f"  Files in doc: {files}")
                    
                    # 检查是否存在README文件（任何形式）
                    readme_files = [f for f in files if f.upper().startswith('README')]
                    if readme_files:
                        print(f"  Found README files: {readme_files}")
                        return True
                    else:
                        print(f"  No README files found")
                        return False
                except Exception as e:
                    print(f"  Error listing files: {e}")
                    return False
        
        return False

    def get_board_name_from_path(self, path):
        """
        从路径中提取板子型号
        路径格式：customer/boards/板子型号/doc
        """
        path_parts = os.path.normpath(path).split(os.sep)
        if len(path_parts) >= 2 and path_parts[-1] == "doc":
            return path_parts[-2]  # 返回板子型号
        return None

    def copy_board_files(self, src, tgt, board_name):
        """
        复制板子文档文件到目标目录
        src: 源doc目录路径
        tgt: 目标根目录路径
        board_name: 板子名称
        """
        # 查找README文件（任何形式）
        readme_path = None
        try:
            files = os.listdir(src)
            readme_files = [f for f in files if f.upper().startswith('README')]
            if readme_files:
                # 优先使用指定语言的README，否则使用第一个找到的
                for readme_file in readme_files:
                    if readme_file == self.readme_filename:
                        readme_path = os.path.join(src, readme_file)
                        break
                if not readme_path:
                    readme_path = os.path.join(src, readme_files[0])
        except Exception as e:
            print(f"Error listing files in {src}: {e}")
            return
        
        if not readme_path or not os.path.exists(readme_path):
            print(f"No README file found in {src}")
            return
        
        # 目标目录直接使用板子名称
        target_dir = os.path.join(tgt, board_name)
        print("mkdir: {}".format(target_dir))
        os.makedirs(target_dir, exist_ok=True)

        # 拷贝assets目录（使用通用方法）
        self._copy_assets_if_exists(src, target_dir)
        
        # 拷贝README文件并重命名为README.md（统一输出文件名）
        print("copy {} for {}".format(os.path.basename(readme_path), board_name))    
        shutil.copy(readme_path, os.path.join(target_dir, self.README_FILENAME))

    def create_index_file(self, path):
        with os.scandir(path) as entries:
            index_file = []
            for entry in entries:
                if entry.is_dir(follow_symlinks=False):
                    if os.path.exists(os.path.join(entry.path, self.README_FILENAME)):
                        index_file.append("{}/{}".format(entry.name, self.README_FILENAME))
                    else:
                        index_file.append("{}/index.md".format(entry.name))
                        self.create_index_file(entry.path)
            
            s = ""
            if "." == path:
                title = "Supported Boards"
            else:
                title = os.path.split(path)[1]            
            s += "# {}\n\n".format(title)
            s += "```{toctree}\n"
            s += ":hidden:\n"
            s += "\n"
            for l in index_file:
                s += l + "\n"
            s += "\n"
            s += "```\n"    

            with open(os.path.join(path, "index.md"), "w", encoding='utf-8') as f:
                f.write(s)

    def traverse_folder_with_scandir(self, src, tgt, skip_customer=False):
        # 特殊处理customer目录
        if skip_customer and os.path.basename(src) == "customer":
            self.copy_index_file(src, tgt, skip_customer=True)
            # 直接处理customer下的子目录，但不创建customer层级
            with os.scandir(src) as entries:
                for entry in entries:
                    if entry.is_dir(follow_symlinks=False):
                        # 计算相对路径，跳过customer层级
                        relative_path = os.path.relpath(entry.path, src)
                        self.traverse_folder_with_scandir_internal(entry.path, tgt, relative_path)
        else:
            self.copy_index_file(src, tgt)
            with os.scandir(src) as entries:
                for entry in entries:
                    if entry.is_dir(follow_symlinks=False):
                        self.traverse_folder_with_scandir(entry.path, tgt)

    def traverse_folder_with_scandir_internal(self, src, tgt, relative_base):
        """
        内部遍历方法，用于处理去除customer层级后的路径
        src: 当前源目录
        tgt: 目标根目录
        relative_base: 相对于customer的路径（不包含customer）
        """
        # 复制index文件到正确的位置
        indexfile_path = os.path.join(src, self.INDEX_FILENAME)
        if os.path.exists(indexfile_path):
            target_dir = os.path.join(tgt, relative_base)
            print("mkdir: {}".format(target_dir))
            os.makedirs(target_dir, exist_ok=True)
            
            # 复制assets目录（如果存在）
            self._copy_assets_if_exists(src, target_dir)
            
            if self.lang != 'zh_CN':
                if os.path.exists(os.path.join(src, self.INDEX_EN_FILENAME)):
                    indexfile_path = os.path.join(src, self.INDEX_EN_FILENAME)
                    print("copy {}".format(self.INDEX_EN_FILENAME))
                    target_file = os.path.join(target_dir, self.INDEX_FILENAME)
                    shutil.copy(indexfile_path, target_file)
                else:
                    print("copy {}".format(self.INDEX_FILENAME))
                    shutil.copy(indexfile_path, os.path.join(target_dir, self.INDEX_FILENAME))
            else:
                print("copy {}".format(self.INDEX_FILENAME))
                shutil.copy(indexfile_path, os.path.join(target_dir, self.INDEX_FILENAME))
        
        with os.scandir(src) as entries:
            for entry in entries:
                if entry.is_dir(follow_symlinks=False):
                    if self.is_board_doc_dir(entry.path):
                        # 如果是板子文档目录，复制文档到以板子名称命名的目录
                        board_name = self.get_board_name_from_path(entry.path)
                        if board_name:
                            print(f"Found board doc: {board_name}")
                            # 计算板子在目标目录中的路径（去除customer层级）
                            # relative_base是相对于customer的路径，如"boards"
                            parent_path = os.path.dirname(relative_base)
                            if parent_path:
                                board_target_path = os.path.join(parent_path, board_name)
                            else:
                                board_target_path = board_name
                            
                            self.copy_board_files(entry.path, tgt, board_target_path)
                    else:
                        # 继续递归遍历子目录
                        new_relative = os.path.join(relative_base, entry.name)
                        self.traverse_folder_with_scandir_internal(entry.path, tgt, new_relative)
    
    def copy_board_docs(self, source, dest):
        """
        复制板子文档
        source: 源目录路径（包含customer目录）
        dest: 目标目录路径
        """
        curr_path = os.getcwd()
        os.chdir(source)
        
        if not os.path.isabs(dest):
            dest = os.path.join(curr_path, dest)
        
        # 查找customer目录
        customer_dir = None
        if os.path.exists("customer"):
            customer_dir = "customer"
        else:
            # 如果当前目录没有customer，递归查找
            for root, dirs, files in os.walk("."):
                if "customer" in dirs:
                    customer_dir = os.path.join(root, "customer")
                    break
        
        if customer_dir:
            print(f"Found customer directory: {customer_dir}")
            # 使用skip_customer=True来跳过customer层级
            self.traverse_folder_with_scandir(customer_dir, dest, skip_customer=True)
        else:
            print("Customer directory not found!")
        
        os.chdir(curr_path)