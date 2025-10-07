import os
import json
import shlex
from typing import List, Optional

PROJECT_DIR = os.path.join('..')


def vscode_support(env) -> None:
    """
    生成VS Code的c_cpp_properties.json配置文件
    
    参数:
        env: SCons构建环境对象
    """
    # 提取构建环境变量
    env_vars = json.loads(env.Dump(format="json"))
    
    # 准备配置数据
    config = {
        "configurations": [{
            "name": "sf32",
            "includePath": env_vars["CPPPATH"],
            "compilerPath": _find_compiler(env_vars),
            "intelliSenseMode": "gcc-arm",
            "defines": list(env["CPPDEFINES"])
        }]
    }
    
    # 确保.vscode目录存在
    vscode_dir = os.path.join(PROJECT_DIR, '.vscode')
    os.makedirs(vscode_dir, exist_ok=True)
    
    # 写入配置文件
    config_path = os.path.join(vscode_dir, 'c_cpp_properties.json')
    with open(config_path, 'w') as config_file:
        json.dump(config, config_file, indent=4)
        print(f"✓ 已生成VS Code配置文件: {config_path}")


def clion_support(env) -> None:
    """
    生成CLion的compile_commands.json配置文件
    
    参数:
        env: SCons构建环境对象
    """
    # 获取构建环境信息
    env_vars = json.loads(env.Dump(format="json"))
    build_dir = env_vars["BUILD_DIR_FULL_PATH"]
    
    # 定位编译数据库
    db_name = str(env.CompilationDatabase()[0])
    compile_db_path = os.path.join(build_dir, db_name)
    
    # 验证编译器路径
    compiler_path = _find_compiler(env_vars)
    if not compiler_path:
        raise RuntimeError(f"编译器 '{env_vars['CC']}' 在PATH中未找到")
    
    # 处理编译命令
    processed_commands = _process_compile_commands(compile_db_path, compiler_path)
    
    # 写入处理后的编译命令
    output_path = os.path.join(PROJECT_DIR, 'compile_commands.json')
    with open(output_path, 'w') as output_file:
        json.dump(processed_commands, output_file, indent=4)
        print(f"✓ 已生成CLion配置文件: {output_path}")


def _find_compiler(env_vars: dict) -> Optional[str]:
    """在PATH中定位编译器绝对路径"""
    return find_executable_in_path(
        executable_name=env_vars["CC"],
        path_env=env_vars["ENV"]["PATH"]
    )


def _process_compile_commands(db_path: str, compiler_path: str) -> List[dict]:
    """
    处理编译命令数据库
    
    参数:
        db_path: 原始compile_commands.json路径
        compiler_path: 编译器绝对路径
        
    返回:
        处理后的编译命令列表
    """
    with open(db_path, 'r', encoding='utf-8') as db_file:
        commands = json.load(db_file)
    
    processed = []
    for entry in commands:
        # 跳过不存在的源文件
        source_path = _resolve_source_path(entry)
        if not os.path.exists(source_path):
            continue
        
        # 替换编译器路径
        if "command" in entry:
            tokens = shlex.split(entry["command"])
            for i, token in enumerate(tokens):
                if '=' not in token and token.strip():
                    tokens[i] = compiler_path
                    break
            entry["command"] = " ".join(tokens)
        
        processed.append(entry)
    
    return processed


def _resolve_source_path(entry: dict) -> str:
    """解析源文件的绝对路径"""
    source = entry["file"]
    return source if os.path.isabs(source) else os.path.join(entry["directory"], source)


def find_executable_in_path(
    executable_name: str, 
    path_env: Optional[str] = None
) -> Optional[str]:
    """
    在系统PATH中定位可执行文件
    
    参数:
        executable_name: 可执行文件名
        path_env: 可选的PATH环境变量字符串
        
    返回:
        找到的可执行文件完整路径，未找到则返回None
    """
    # 获取系统PATH（如果未提供）
    path_str = path_env or os.environ.get('PATH', '')
    
    # 根据平台确定路径分隔符
    separator = ';' if os.name == 'nt' else ':'
    
    for directory in path_str.split(separator):
        dir_path = directory.strip()
        if not dir_path:
            continue
        
        # 尝试基本路径匹配
        candidate = os.path.join(dir_path, executable_name)
        if os.path.exists(candidate):
            return candidate
        
        # Windows平台尝试常见扩展名
        if os.name == 'nt':
            for ext in ('.exe', '.com', '.bat', '.cmd'):
                if os.path.exists(candidate + ext):
                    return candidate + ext
                    
    return None