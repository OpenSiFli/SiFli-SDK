Optional QSort
~~~~~~~~~~~~~~

Yaffs needs a sort function for sorting block indices during scanning.

Most systems already have qsort (or an equivalent that can be used)
as part of the standard library and generally it is best to use that.

For those systems that do not have this, we supply the qsort in this
directory.

This qsort.c file is the only code that is supplied as part of the
Yaffs Direct code that is NOT written, from the ground up, by Aleph One.

The licensing terms are fairly innocuous. Please read the banner at the top
of qsort.c



