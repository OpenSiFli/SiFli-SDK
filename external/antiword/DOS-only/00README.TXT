Basic installation instructions for Antiword for DOS
====================================================
Unpack antiword.zip
  pkunzip -d antiword.zip c:\antiword

NOTE: experienced users of DOS may want to edit their autoexec.bat in order
to add the Antiword directory to their PATH variable.

If you don't do this, you will need to go into the Antiword directory before
running the program.


More advanced installation
==========================
These instructions can be followed under all versions of Windows, but are
most useful under Windows NT, 2000 and XP.

(1) Go to the configuration screen and choose system and environment. Make a
    HOME environment variable containing C:\home\yourname or H:\loginname or
    something similar.
(2) Follow the basic installation instructions, but use %HOME%\antiword as
    the Antiword directory.


Problems
========

If get an dpmi error like: "Load error: no DPMI - Get csdpmi*.zip", you can
download the missing component from:
ftp://ftp.fh-koblenz.de/pub/DJGPP/current/v2misc/csdpmi5b.zip


Differences between the Unix version of Antiword and the DOS version
====================================================================

(1)
The Unix version looks for its mapping files in $HOME/.antiword.
The DOS version looks for its mapping files in %HOME%\antiword if HOME is set
and in C:\antiword if HOME is not set.

(2)
The Unix version reads its font information from a file called fontnames.
The DOS version reads its font information from a file called fontname.txt.

(3)
The DOS version expects its mapping files to be in DOS text format. That means
that all lines must end with a carriage return, line feed combination. (CR+NL)

(4)
The Unix version needs mapping file 8859-1.txt for Latin1 text and 8859-2.txt
for Latin2 text.
The DOS version needs mapping file cp850.txt for Latin1 text and cp852.txt for
Latin2 text.

(5)
The DOS version rejects all files with long filenames.
Please use the DOS name (8+3) instead.

(6)
The Antiword executable (antiword.exe) can be placed in any directory. If you
choose a directory from your %PATH%, you will not have to use a full pathname
every time.

(7)
Under DOS the default mapping file does not depend on the locale. Locale is
a POSIX feature. Under DOS the default mapping file depends on the active
codepage.


Examples of use
===============

To get text output
==================

(1) display a Word document in Latin1, one page at a time
antiword -m cp850.txt filename.doc | more

(2) save the text version of a Word document in Latin2 in a file
antiword -m cp852.txt filename.doc > filename.txt

(3) display a Word document in Hebrew, one page at a time
antiword -m cp862.txt filename.doc | more

(4) save the text version of a Word document in Russian Cyrillic in a file
antiword -m cp866.txt filename.doc > filename.txt

To get PostScript output
========================

(1) save the PostScript version of a Word document in Latin1 in a file,
    generate PostScript for printing on European A4 size paper
antiword -p a4 -m 8859-1.txt filename.doc > filename.ps

(2) save the PostScipt version of a Word document in Latin2 in a file,
    generate PostScript for printing on American letter size paper
antiword -p letter -m 8859-2.txt filename.doc > filename.ps

(3) save the PostScript version of a Word document in Cyrillic in a file,
    generate PostScript for printing on European A4 size paper
antiword -p a4 -m 8859-5.txt filename.doc > filename.ps

NOTE: PostScript ouput is only supported for character sets Latin1, Latin2 and
      Cyrillic.

NOTE: You need different mapping files for text and PostScript.

The order of the switches is not important to Antiword. If you specify two or
more settings of the same switch, Antiword will use the rightmost one.


Screen fonts
============

This section is only for the few people who want Antiword output in more than
one character set. 

If you want Antiword to display a character set other than the ones you know
your version of DOS can display already, you must first load a screen font.
This screen font must match the mapping file or files you are using.

You can download the freeware collection of VGA screen fonts and font
loader fntcol16.zip (452 KB) from SimTel:
http://www.simtel.net/pub/pd/49501.html
or
ftp://ftp.simtel.net/pub/simtelnet/msdos/screen/fntcol16.zip
