#!/bin/sh
#
# Script to make drag and drop in KDE possible
#set -x
#

if [ $# -lt 2 ]
then
	exit 0
fi

# Determine the temp directory
if [ -d "$TMPDIR" ] && [ -w "$TMPDIR" ]
then
	tmp_dir=$TMPDIR
elif [ -d "$TEMP" ] && [ -w "$TEMP" ]
then
	tmp_dir=$TEMP
else
	tmp_dir="/tmp"
fi                        
out_file=`tempfile -d $tmp_dir` || { echo "$0: Cannot create temporary file" >&2; exit 1;  }
err_file=`tempfile -d $tmp_dir` || { echo "$0: Cannot create temporary file" >&2; exit 1;  }
# Clean up
trap " /bin/rm -f -- \"$out_file\" \"$err_file\"" 0 1 2 3 13 15

# Determine the paper size
paper_size=$1
shift

# Make the PostScript file
antiword -p $paper_size -i 0 "$@" 2>$err_file >$out_file
if [ $? -ne 0 ]
then
	exit 1
fi

# Show the PostScript file
gv $out_file -nocentre -media $paper_size

exit 0
