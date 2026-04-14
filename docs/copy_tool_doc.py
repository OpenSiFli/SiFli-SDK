# -*- coding: utf-8 -*-
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

import os
import shutil


class ToolDocCopier:
    UM_SUFFIX = "_UM.md"
    UM_EN_SUFFIX = "_UM_EN.md"

    def __init__(self, lang='zh_CN'):
        self.lang = lang

    def find_tool_doc_dirs(self, tools_root):
        """
        扫描 tools/ 顶层子目录，找出含有 doc/*_UM.md 的工具目录。
        返回 list of (tool_name, doc_dir_path, [um_files])
        """
        results = []
        try:
            with os.scandir(tools_root) as entries:
                for entry in entries:
                    if not entry.is_dir(follow_symlinks=False):
                        continue
                    doc_dir = os.path.join(entry.path, 'doc')
                    if not os.path.isdir(doc_dir):
                        continue
                    try:
                        files = os.listdir(doc_dir)
                    except Exception as e:
                        print(f"  Error listing {doc_dir}: {e}")
                        continue
                    um_files = [f for f in files if f.endswith(self.UM_SUFFIX) or f.endswith(self.UM_EN_SUFFIX)]
                    if um_files:
                        print(f"Found tool doc: {entry.name} -> {um_files}")
                        results.append((entry.name, doc_dir, um_files))
        except Exception as e:
            print(f"Error scanning tools root {tools_root}: {e}")
        return results

    def _select_um_file(self, doc_dir, tool_name, um_files):
        """
        根据语言选择合适的 UM 文件。
        英文优先 *_UM_EN.md，无则 fallback 到 *_UM.md。
        中文使用 *_UM.md，无则使用第一个找到的 *_UM*.md。
        """
        zh_candidate = tool_name + self.UM_SUFFIX
        en_candidate = tool_name + self.UM_EN_SUFFIX

        if self.lang == 'en':
            if en_candidate in um_files:
                return en_candidate
            if zh_candidate in um_files:
                return zh_candidate
        else:
            if zh_candidate in um_files:
                return zh_candidate

        # fallback: 找任意 *_UM.md（排除 EN 版本优先）
        zh_files = [f for f in um_files if not f.endswith(self.UM_EN_SUFFIX)]
        if zh_files:
            return zh_files[0]
        if um_files:
            return um_files[0]
        return None

    def copy_tool_docs(self, tools_source, dest):
        """
        主入口：扫描 tools_source 并将工具文档复制到 dest 目录。
        tools_source: 工具根目录路径（相对或绝对）
        dest: 目标 Sphinx 文档目录（相对或绝对）
        """
        curr_path = os.getcwd()

        if not os.path.isabs(tools_source):
            tools_source = os.path.join(curr_path, tools_source)
        if not os.path.isabs(dest):
            dest = os.path.join(curr_path, dest)

        tool_docs = self.find_tool_doc_dirs(tools_source)
        if not tool_docs:
            print("No tool documentation found.")
            return

        os.makedirs(dest, exist_ok=True)

        tool_entries = []  # list of (tool_name, toctree_ref)
        for tool_name, doc_dir, um_files in sorted(tool_docs):
            um_file = self._select_um_file(doc_dir, tool_name, um_files)
            if not um_file:
                print(f"  No suitable UM file found for {tool_name}, skipping.")
                continue

            # 目标子目录：dest/{tool_name}/
            tool_dest = os.path.join(dest, tool_name)
            print(f"mkdir: {tool_dest}")
            os.makedirs(tool_dest, exist_ok=True)

            # 复制 UM 文件，统一输出名为 {tool_name}_UM.md
            src_path = os.path.join(doc_dir, um_file)
            dst_name = tool_name + self.UM_SUFFIX
            dst_path = os.path.join(tool_dest, dst_name)
            print(f"copy {um_file} -> {dst_name}")
            shutil.copy(src_path, dst_path)

            # 复制 png/ 资源目录（如存在）
            png_src = os.path.join(doc_dir, 'png')
            if os.path.isdir(png_src):
                png_dst = os.path.join(tool_dest, 'png')
                print(f"copy png/ -> {png_dst}")
                shutil.copytree(png_src, png_dst, dirs_exist_ok=True)

            # toctree 引用路径（去掉 .md 后缀）
            toctree_ref = f"{tool_name}/{tool_name}_UM"
            tool_entries.append((tool_name, toctree_ref))

        self.create_index_file(dest, tool_entries)

    def create_index_file(self, dest, tool_entries):
        """生成 dest/index.md，列出所有工具文档的 toctree。"""
        title = "工具文档" if self.lang == 'zh_CN' else "Tools Documentation"
        s = f"# {title}\n\n"
        s += "```{toctree}\n"
        s += ":hidden:\n"
        s += "\n"
        for _, toctree_ref in tool_entries:
            s += toctree_ref + "\n"
        s += "\n"
        s += "```\n"

        index_path = os.path.join(dest, "index.md")
        print(f"write index: {index_path}")
        with open(index_path, "w", encoding='utf-8') as f:
            f.write(s)
