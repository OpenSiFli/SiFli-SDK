# -*- coding: utf-8 -*-
import os
import shutil
import sys

README_FILENAME = "README.md"
INDEX_FILENAME = "index.md"

def copyfiles(src, tgt):
    readme_path = os.path.join(src, README_FILENAME)
    if not os.path.exists(readme_path):
        return
    
    target_dir = os.path.join(tgt, src)
    print("mkdir: {}".format(target_dir))
    os.makedirs(target_dir, exist_ok=True)

    asset_dir = os.path.join(src, "assets")
    if os.path.isdir(asset_dir):
        print("copy assets")
        shutil.copytree(asset_dir, os.path.join(target_dir, os.path.basename(asset_dir)), dirs_exist_ok=True)
    print("copy {}".format(README_FILENAME))    
    shutil.copy(readme_path, target_dir)

def copy_indexfile(src, tgt):
    indexfile_path = os.path.join(src, INDEX_FILENAME)
    if not os.path.exists(indexfile_path):
        return

    target_dir = os.path.join(tgt, src)
    print("mkdir: {}".format(target_dir))
    os.makedirs(target_dir, exist_ok=True)

    print("copy {}".format(INDEX_FILENAME))    
    shutil.copy(indexfile_path, target_dir)

def is_project_root_dir(path):
    if (os.path.exists(os.path.join(path, "project")) 
            and os.path.exists(os.path.join(path, README_FILENAME))):
        return True
    elif (os.path.exists(os.path.join(path, "common")) 
            and os.path.exists(os.path.join(path, README_FILENAME))):
        return True
    else:
        return False


    if (os.path.exists(os.path.join(path, "project")) 
            and os.path.exists(os.path.join(path, README_FILENAME))):
        return True
    else:
        return False


   
def create_index_file(path):
    with os.scandir(path) as entries:
        index_file = []
        for entry in entries:
            if entry.is_dir(follow_symlinks=False):
                if os.path.exists(os.path.join(entry.path, README_FILENAME)):
                    index_file.append("{}/{}".format(entry.name, README_FILENAME))
                else:
                    index_file.append("{}/index.md".format(entry.name))
                    create_index_file(entry.path)
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

def traverse_folder_with_scandir(src, tgt):
    copy_indexfile(src, tgt)
    with os.scandir(src) as entries:
        for entry in entries:
            if entry.is_dir(follow_symlinks=False):
                if is_project_root_dir(entry.path):
                    copyfiles(entry.path, tgt)
                else:
                    traverse_folder_with_scandir(entry.path, tgt)  # 递归遍历子目录
            # elif entry.is_file():
            #     print("File:", entry.path)

def main(source, dest):

    path = source
    target = dest
    curr_path = os.getcwd()
    os.chdir(path)
    if not os.path.isabs(target):
        target = os.path.join(curr_path, target)
    traverse_folder_with_scandir(".", target)
    # os.chdir(target)
    # create_index_file(".")
    os.chdir(curr_path)

def _main():
    if len(sys.argv) < 3:
        print("wrong arg")
        exit(1)

    path = sys.argv[1]
    target = sys.argv[2]
    main(sys.argv[1], sys.argv[2])

if __name__ == "__main__":
    _main()
