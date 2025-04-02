# How to build the document

## Install toolchain
1. Install Python 3.x
1. Execute `pip install -r requirements.txt` in command line under the `docs` folder. It would install all modules needed by the document building

## Build the document
Run batch file `make_all.bat` in Windows command line. It would build documents for all SoC series. You can also execute command below to build only sf32lb52x document and the generated document locates in `build_52x` folder.
```shell
python generate_docs.py 52x
```


