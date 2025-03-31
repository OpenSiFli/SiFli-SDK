import subprocess
import argparse
import os
import shutil
import copy_example_doc

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
        run_command('..\..\\tools\doxygen\\bin\doxygen Doxyfile_52x.sphinx', cwd=doxygen_dir)
    elif board == '55x':
        run_command('..\..\\tools\doxygen\\bin\doxygen Doxyfile_55x.sphinx', cwd=doxygen_dir)
    elif board == '56x':
        run_command('..\..\\tools\doxygen\\bin\doxygen Doxyfile_56x.sphinx', cwd=doxygen_dir)
    elif board == '58x':
        run_command('..\..\\tools\doxygen\\bin\doxygen Doxyfile_58x.sphinx', cwd=doxygen_dir)


def make_html(board):
    print(f"Building HTML documentation for {board}...")
    if board == '52x':
        run_command('sphinx-build -M html source build_52x -t SF32LB52X -j 8')
    elif board == '55x':
        run_command('sphinx-build -M html source build_55x -t SF32LB55X -j 8')
    elif board == '56x':
        run_command('sphinx-build -M html source build_56x -t SF32LB56X -j 8')
    elif board == '58x':
        run_command('sphinx-build -M html source build_58x -t SF32LB58X -j 8')


def copy_to_output(board):
    print(f"Copying HTML documentation for {board} to output directory...")
    if board == '52x':
        output_dir = os.path.join('output', 'sf32lb52x')
        source_dir = os.path.join('build_52x', 'html')
    elif board == '55x':
        output_dir = os.path.join('output', 'sf32lb55x')
        source_dir = os.path.join('build_55x', 'html')
    elif board == '56x':
        output_dir = os.path.join('output', 'sf32lb56x')
        source_dir = os.path.join('build_56x', 'html')
    elif board == '58x':
        output_dir = os.path.join('output', 'sf32lb58x')
        source_dir = os.path.join('build_58x', 'html')


    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    if os.path.exists(source_dir):
        for item in os.listdir(source_dir):
            s = os.path.join(source_dir, item)
            d = os.path.join(output_dir, item)
            if os.path.isdir(s):
                shutil.copytree(s, d, dirs_exist_ok=True)
            else:
                shutil.copy2(s, d)

def main(board):
    # Step 1: Generate Doxygen XML
    generate_doxygen_xml(board)

    # Step 2: Copy example documents
    print("Copying example documents...")
    copy_example_doc.main("../example", "source/example")

    # Step 3: Build HTML documentation
    make_html(board)

    # Step 4: Copy HTML documentation to output directory
    copy_to_output(board)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate documentation for specified board.')
    parser.add_argument('board', choices=['52x', '55x', '56x', '58x'], help='Specify the board (52x or 55x or 56x or 58x)')
    args = parser.parse_args()

    main(args.board)