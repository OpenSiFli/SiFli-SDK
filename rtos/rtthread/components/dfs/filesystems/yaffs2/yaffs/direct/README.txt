New Handling for Yaffs Core Code in Yaffs Direct

In the past, the Yaffs core code was joined into the Yaffs direct code by
means of symbolic links. This had to be changed when some Linux
kernel-friendly changes were made.

Now, the Yaffs core code needs to be slightly modified before use in Yaffs
Direct. This is accomplished by a script that copies the code intto the
Yaffs Direct "workspace" while using sed to modify some of the function 
names etc. For example, strlen() is changed to yaffs_strlen() to allow these
functions to be twaeked for use with unicode.

This is accomplished by the script called handle_common.sh.

cd yaffs2/direct
./handle_common.sh copy

