# MicroPython Example
```{warning}
Not verified
```

## User Guide
### Introduction
The micropython application will verify micropython functionality on the board flash. The file system uses FAT format. Common file commands can be called in the UART console, such as:
```
df               - Disk free
mountfs          - Mount device to file system
mkfs             - Format disk with file system
mkdir            - Create the DIRECTORY.
pwd              - Print the name of the current working directory.
cd               - Change the shell working directory.
rm               - Remove(unlink) the FILE(s).
cat              - Concatenate FILE(s)
mv               - Rename SOURCE to DEST.
cp               - Copy SOURCE to DEST.
ls               - List information about the FILEs.
python           - run python script or start with python REPL.
```
      
## Project Description
- Compilation method: Enter the project directory and execute the command `scons --board=<board_name> -j8`, where board_name is the board name. For example, to compile the eh-lb561 board, the complete command is `scons --board=eh-lb561 -j8`
  The compiled image file is stored in the HCPU's build_<board_name> directory. For common project usage, refer to <<Common Project Build Method>>
- Simulator compilation method: Enter the simulator directory and execute the command 'scons -j8'. When running, execute build\bf0_ap.exe in the simulator directory