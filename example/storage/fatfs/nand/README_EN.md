# FatFs NAND Example
Source path: example/storage/fatfs/nand

## Usage Guide
- sf32lb52-lcd_a128r16
- sf32lb56-lcd_a128r12n1
- sf32lb58-lcd_a128r32n1

## Example Usage Instructions
This example demonstrates the file system capabilities of FatFs using the FAT
format. Standard file system commands can be executed via the UART console,
including:

```
df               - Display free disk space
mountfs          - Mount a device to the file system
mkfs             - Format a disk with a file system
mkdir            - Create a directory
pwd              - Print the name of the current working directory
cd               - Change the shell working directory
rm               - Remove (unlink) files
cat              - Concatenate and display file content
mv               - Rename SOURCE to DEST
cp               - Copy SOURCE to DEST
ls               - List information about files
```
### Hardware Requirements
The example includes a file system performance benchmark, which can be initiated
using the `fs_test` command:
- To run the example, you need to have a development board that supports this
  example
- A USB data cable capable of data transmission
- The benchmark evaluates both write and read speeds.

Expected test results:
```c
TX:fs_test
Creating test file with 1024 KB data...
Written 0 KB...
Written 100 KB...
Written 200 KB...
Written 300 KB...
Written 400 KB...
Written 500 KB...
Written 600 KB...
Written 700 KB...
Written 800 KB...
Written 900 KB...
Written 1000 KB...
Write test completed
Write Speed: 1721799 bytes/sec (1681.44 KB/s)
Write Ops/sec: 1681 ops/sec
Reading test file...
Read 0 KB...
Read 100 KB...
Read 200 KB...
Read 300 KB...
Read 400 KB...
Read 500 KB...
Read 600 KB...
Read 700 KB...
Read 800 KB...
Read 900 KB...
Read 1000 KB...
Read test completed
Read Speed: 5518821 bytes/sec (5389.47 KB/s)
Read Ops/sec: 5389 ops/sec
```

### menuconfig Configuration

![alt text]{1} 2. Use device virtual file system

```
//Execute command
 menuconfig --board=em-lb561
```
## Project Description
### Hardware Requirements
1. Prerequisites for running the routine include a supported development board
and 2. a USB data cable capable of data transmission.
### menuconfig Configuration
```
// Execute command
 menuconfig --board=em-lb561
```
1. Enable the MTD Dhara Nand Flash device in menuconfig.

![alt text](assets/file_system_1.png) 2. Enable the Device Virtual File System
(DFS).

![alt text](assets/file_system_2.png) 3. Select the HAL Assert type.

![alt text](assets/file_system_3.png)

## Example Output Results Display
- Send ls through serial port to view files in root directory.
- Input mkdir test1 to create test1 folder (directory).

## Example Output Results
The following log shows the output of the example running on the development
board. If these logs do not appear, the example failed to run as expected and
troubleshooting is required.
```
Filesystem successfully mounted on flash root.
```
1. Execute `ls` via the serial terminal to list the files in the root directory.

2. Enter `mkdir test1` to create a new directory named `test1`.

3. Use `cd` followed by the directory name to navigate to the specified path.
Execute `pwd` to verify that the current working directory has updated
correctly.

4. You may create another directory within this path using `mkdir test2`. Run
`ls` to confirm the directory was created successfully. ![alt
text](assets/file_system_log_1.png)
### Troubleshooting
If the expected logs or behavior do not appear, please troubleshoot the
following areas:
* Verify that the hardware connections are secure and correct.
* Ensure the USB cable supports data transfer.
* Verify that the menu configuration is correct (specifically the Flash model
  for the board).
