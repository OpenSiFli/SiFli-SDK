# MicroPython Example
Source path: example/micropython
```{warning}
Not verified
```
## User Guide
### Introduction
The micropython application will verify micropython functionality on the board
flash. The file system uses FAT format. Common file commands can be called in
the UART console, such as:
```
df               - Display free disk space.
mountfs          - Mount a device to the file system.
mkfs             - Format a disk with a file system.
mkdir            - Create a directory.
pwd              - Print the name of the current working directory.
cd               - Change the shell working directory.
rm               - Remove (unlink) files.
cat              - Concatenate and display file content.
mv               - Rename SOURCE to DEST.
cp               - Copy SOURCE to DEST.
ls               - List information about files.
python           - Execute a Python script or start the Python REPL.
```

## Project Description
- Compilation method: Enter the project directory and execute the command `scons
  --board=<board_name> -j8`, where board_name is the board name. For example, to
  compile the eh-lb561 board, the complete command is `scons --board=eh-lb561
  -j8` The compiled image file is stored in the HCPU's build_<board_name>
  directory. For common project usage, refer to <{1}>
- Simulator compilation method: Enter the simulator directory and execute the
  command 'scons -j8'. When running, execute build\bf0_ap.exe in the simulator
  directory

