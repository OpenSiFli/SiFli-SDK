# -*- coding: utf-8 -*-
import os
import shutil
import sys


class ExampleDocCopier:
    README_FILENAME = "README.md"
    README_EN_FILENAME = "README_EN.md"
    INDEX_FILENAME = "index.md"
    INDEX_EN_FILENAME = "index_EN.md"
    
    def __init__(self, lang='zh_CN'):
        self.lang = lang
        self.readme_filename = self._get_readme_filename()
    
    def _get_readme_filename(self):
        return self.README_EN_FILENAME if self.lang == 'en' else self.README_FILENAME
    
    def copy_files(self, src, tgt):
        readme_path = os.path.join(src, self.readme_filename)
        if not os.path.exists(readme_path):
            return
        
        target_dir = os.path.join(tgt, src)
        print("mkdir: {}".format(target_dir))
        os.makedirs(target_dir, exist_ok=True)

        # 拷贝assets目录
        asset_dir = os.path.join(src, "assets")
        if os.path.isdir(asset_dir):
            print("copy assets")
            shutil.copytree(asset_dir, os.path.join(target_dir, os.path.basename(asset_dir)), dirs_exist_ok=True)
        
        # 拷贝README文件并重命名为README.md（统一输出文件名）
        print("copy {}".format(self.readme_filename))    
        shutil.copy(readme_path, os.path.join(target_dir, self.README_FILENAME))

    def copy_index_file(self, src, tgt):
        indexfile_path = os.path.join(src, self.INDEX_FILENAME)
        if not os.path.exists(indexfile_path):
            return

        target_dir = os.path.join(tgt, src)
        print("mkdir: {}".format(target_dir))
        os.makedirs(target_dir, exist_ok=True)

        if self.lang != 'zh_CN':
            if os.path.exists(os.path.join(src, self.INDEX_EN_FILENAME)):
                indexfile_path = os.path.join(src, self.INDEX_EN_FILENAME)
                print("copy {}".format(self.INDEX_EN_FILENAME))
                target_dir = os.path.join(target_dir, self.INDEX_FILENAME)
                shutil.copy(indexfile_path, target_dir)
                return

        print("copy {}".format(self.INDEX_FILENAME))
        shutil.copy(indexfile_path, target_dir)

    def is_project_root_dir(self, path):
        if (os.path.exists(os.path.join(path, "project")) 
                and os.path.exists(os.path.join(path, self.readme_filename))):
            return True
        elif (os.path.exists(os.path.join(path, "common")) 
                and os.path.exists(os.path.join(path, self.readme_filename))):
            return True
        else:
            return False

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
                title = "Examples"
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

            with open(os.path.join(path, "index.md"), "w") as f:
                f.write(s)

    def traverse_folder_with_scandir(self, src, tgt):
        self.copy_index_file(src, tgt)
        with os.scandir(src) as entries:
            for entry in entries:
                if entry.is_dir(follow_symlinks=False):
                    if self.is_project_root_dir(entry.path):
                        self.copy_files(entry.path, tgt)
                    else:
                        self.traverse_folder_with_scandir(entry.path, tgt)  # 递归遍历子目录
                # elif entry.is_file():
                #     print("File:", entry.path)
    
    def copy_example_docs(self, source, dest):
        curr_path = os.getcwd()
        os.chdir(source)
        
        if not os.path.isabs(dest):
            dest = os.path.join(curr_path, dest)
        
        self.traverse_folder_with_scandir(".", dest)
        os.chdir(curr_path)
