# libepub
Simple library for parsing EPUB files

##### Aim of the library
The aim of the library is to provide a simple and easy solution to parse and process the data from EPUB files. This library should make it easy for one to develop an EPUB reader.

##### Working of an epub file
An epub file is basically a zip file containing the respective HTML pages and the images in specified directories. The files are usually placed inside directories with varied names but in order to get the name of the root file, the first file that needs to be read will beplaced under META-INF/container.xml
A typical container XML file consists of data of the following type:
```
<?xml version="1.0" encoding="utf-8" standalone="no"?>
<container xmlns="urn:oasis:names:tc:opendocument:xmlns:container" version="1.0">
  <rootfiles>
    <rootfile full-path="OEBPS/content.opf" media-type="application/oebps-package+xml"/>
  </rootfiles>
</container>
```
The `full-path` attribute is the one which is of importance, since this file consists of the table of contents as well as the `id`(s) that are associated with the files(basically filepaths). As an example, a part of the file mentioned in the `full-path` is shown below:
```
<?xml version="1.0" encoding="utf-8" standalone="no"?>
<package xmlns="http://www.idpf.org/2007/opf" version="2.0" unique-identifier="bookid">
  <metadata>
    <dc:identifier xmlns:dc="http://purl.org/dc/elements/1.1/" id="bookid">9781785886195</dc:identifier>
    <dc:title xmlns:dc="http://purl.org/dc/elements/1.1/">Book Title</dc:title>
    <dc:publisher xmlns:dc="http://purl.org/dc/elements/1.1/">Publisher name</dc:publisher>
    <dc:language xmlns:dc="http://purl.org/dc/elements/1.1/">en</dc:language>
    <meta name="cover" content="cover-image"/>
  </metadata>
  <manifest>
    <item id="ncxtoc" media-type="application/x-dtbncx+xml" href="toc.ncx"/>
    <item media-type="text/css" id="css" href="epub.css"/>
    <item id="cover" href="cover.html" media-type="application/xhtml+xml"/>
    <item id="toc" href="toc.html" media-type="application/xhtml+xml"/>
    <item id="id286448481" href="index.html" media-type="application/xhtml+xml"/>
    <item id="cover-image" href="cover/cover.jpg" media-type="image/jpeg"/>
    <item id="id286448497" href="pr01.html" media-type="application/xhtml+xml"/>
    <item id="id286691997" href="graphics/BookInternalAd-30dpi-03.jpg" media-type="image/jpeg"/>
    <item id="id286692006" href="pr02.html" media-type="application/xhtml+xml"/>
    <item id="id286697259" href="pr03.html" media-type="application/xhtml+xml"/>
    <item id="id286697270" href="pr04.html" media-type="application/xhtml+xml"/>
    <item id="id286697310" href="pr05.html" media-type="application/xhtml+xml"/>
    <item id="id286697354" href="pr06.html" media-type="application/xhtml+xml"/>
    <item id="id286697444" href="pr07.html" media-type="application/xhtml+xml"/>
    <item id="id286697635" href="pr07s02.html" media-type="application/xhtml+xml"/>
    <item id="id286697661" href="pr07s03.html" media-type="application/xhtml+xml"/>
    <item id="id286697675" href="pr07s04.html" media-type="application/xhtml+xml"/>
    <item id="id286697744" href="pr07s05.html" media-type="application/xhtml+xml"/>
    <item id="id286697810" href="pr07s06.html" media-type="application/xhtml+xml"/>
    <item id="id286697834" href="pr07s07.html" media-type="application/xhtml+xml"/>
    ...
</manifest>
  <spine toc="ncxtoc">
    <itemref idref="cover" linear="yes"/>
    <itemref idref="toc"/>
    <itemref idref="id286448481"/>
    <itemref idref="id286448497"/>
    <itemref idref="id286692006"/>
    <itemref idref="id286697259"/>
    <itemref idref="id286697270"/>
    <itemref idref="id286697310"/>
    <itemref idref="id286697354"/>
    <itemref idref="id286697444"/>
    <itemref idref="id286697635"/>
    <itemref idref="id286697661"/>
    <itemref idref="id286697675"/>
    <itemref idref="id286697744"/>
    <itemref idref="id286697810"/>
    <itemref idref="id286697834"/>
    <itemref idref="id286698073"/>
    <itemref idref="id286722424"/>
    <itemref idref="id286722849"/>
    <itemref idref="id286723336"/>
    <itemref idref="id286723796"/>
    <itemref idref="id286724235"/>
    <itemref idref="id286750939"/>
    <itemref idref="id286751404"/>
    ...
```

##### The library's task
The aim of the library is to provide appropriate functions to read and parse the content of the files. A sample epub-reader will be made and accordingly the changes to the functions will be done.

##### Dependencies
Currently the project depends on **libzip** and **libxml2**. Installing the development libraries should be enough to get the project up and running.

##### Valgrind Memory status
There is a small problem that is still being investigated. The test program output results in the following still reachable error:
```
==5422== Memcheck, a memory error detector
==5422== Copyright (C) 2002-2017, and GNU GPL'd, by Julian Seward et al.
==5422== Using Valgrind-3.13.0 and LibVEX; rerun with -h for copyright info
==5422== Command: ./epb_test
==5422== Parent PID: 5005
==5422==
==5422==
==5422== HEAP SUMMARY:
==5422==     in use at exit: 1,094 bytes in 20 blocks
==5422==   total heap usage: 6,395 allocs, 6,375 frees, 722,637 bytes allocated
==5422==
==5422== LEAK SUMMARY:
==5422==    definitely lost: 0 bytes in 0 blocks
==5422==    indirectly lost: 0 bytes in 0 blocks
==5422==      possibly lost: 0 bytes in 0 blocks
==5422==    still reachable: 1,094 bytes in 20 blocks
==5422==         suppressed: 0 bytes in 0 blocks
==5422== Reachable blocks (those to which a pointer was found) are not shown.
==5422== To see them, rerun with: --leak-check=full --show-leak-kinds=all
==5422==
==5422== For counts of detected and suppressed errors, rerun with: -v
==5422== ERROR SUMMARY: 0 errors from 0 contexts (suppressed: 0 from 0)
```
The 1094 bytes lost in 20 blocks is because of the document created from the file content in `full-path`(since the file has to be parsed in order to get the table of contents and such other details). This still reachable part most probably is caused due to early off-loading of the library at program exit(But this is just a speculation).

**More information about the library will be updated as the project progresses**
