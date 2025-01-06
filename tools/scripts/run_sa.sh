#!/bin/bash

# Usage:
# run_sa.sh drivers|middleware|example|all

script_path="$(dirname $0)"
reports_path="./reports_$1"
reports_html_path="./reports_html_$1"
rm -f .sconsign.dblite
scons --compiledb cdb
rm -f .sconsign.dblite

if [ -z $2 ];
then 
    CodeChecker analyze compile_database.json -i $script_path/skipfile_$1.txt -o $reports_path --enable sensitive
else
    CodeChecker analyze compile_database.json -i $script_path/skipfile_$1.txt --file *$2 -o $reports_path --enable sensitive
fi
CodeChecker parse --export html -o $reports_html_path $reports_path

