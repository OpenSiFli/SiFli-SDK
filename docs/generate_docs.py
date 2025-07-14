import subprocess
import argparse
import os
import shutil
from copy_example_doc import ExampleDocCopier

def run_command(command, cwd=None):
    result = subprocess.run(command, shell=True, cwd=cwd)
    if result.returncode != 0:
        print(f"Error executing {command}")
        exit(result.returncode)

def generate_doxygen_xml(board):
    print(f"Generating Doxygen XML for {board}...")
    doxygen_dir = 'doxygen'
    if not os.path.exists(doxygen_dir):
        os.makedirs(doxygen_dir)
    xml_dir = os.path.join(doxygen_dir, 'xml')
    if os.path.exists(xml_dir):
        shutil.rmtree(xml_dir)
    os.makedirs(xml_dir)
    
    if board == '52x':
        run_command('doxygen Doxyfile_52x.sphinx', cwd=doxygen_dir)
    elif board == '55x':
        run_command('doxygen Doxyfile_55x.sphinx', cwd=doxygen_dir)
    elif board == '56x':
        run_command('doxygen Doxyfile_56x.sphinx', cwd=doxygen_dir)
    elif board == '58x':
        run_command('doxygen Doxyfile_58x.sphinx', cwd=doxygen_dir)

def get_build_dir(chip, lang):
    dir_mapping = \
    {
        "52x" : "52x",
        "55x" : "55x",
        "56x" : "56x",
        "58x" : "58x",
    }
    return os.path.join('build', lang, dir_mapping[chip])


def make_html(chip, lang):
    print(f"Building HTML documentation for {chip}...")
    arg_mapping = \
    {
        "52x" : 
        {
            "tag": "SF32LB52X",
        },
        "55x" : 
        {
            "tag": "SF32LB55X",
        },
        "56x" : 
        {
            "tag": "SF32LB56X",
        },
        "58x" : 
        {
            "tag": "SF32LB58X",
        },
    }
    output_dir = get_build_dir(chip, lang)
    run_command(f'sphinx-build -M html source/{lang} {output_dir} -t {arg_mapping[chip]["tag"]} -j 8')

def copy_templates(chip, lang):
    print(f"Copying templates for {chip}...")
    source_templates_dir = os.path.join('source', '_templates')
    target_templates_dir = os.path.join('source', lang, '_templates')
    
    if not os.path.exists(target_templates_dir):
        os.makedirs(target_templates_dir)
    
    if os.path.exists(source_templates_dir):
        for item in os.listdir(source_templates_dir):
            s = os.path.join(source_templates_dir, item)
            d = os.path.join(target_templates_dir, item)
            if os.path.isdir(s):
                shutil.copytree(s, d, dirs_exist_ok=True)
            else:
                shutil.copy2(s, d)

def main(chip, lang):
    # Step 1: Generate Doxygen XML
    generate_doxygen_xml(chip)

    # Step 2: Copy example documents
    print("Copying example documents...")
    copier = ExampleDocCopier(lang)
    copier.copy_example_docs("../example", f"source/{lang}/example")

    # Step 3: Build HTML documentation
    copy_templates(chip, lang)
    make_html(chip, lang)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate documentation for specified board.')
    parser.add_argument('chip', choices=['52x', '55x', '56x', '58x'], help='Specify the chip (52x or 55x or 56x or 58x)')
    parser.add_argument('--lang', choices=['en', 'zh_CN'], default='zh_CN', help='Specify language(en or zh_CN)')
    args = parser.parse_args()

    main(args.chip, args.lang)