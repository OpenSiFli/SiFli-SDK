The differences between the the Unix version of antiword and the NLM version.

(1)
The Unix version expects its mapping files in $HOME/.antiword.
The NLM version expects its mapping files in %HOME%/antiword if HOME is set
and in SYS:/antiword if HOME is not set.

(2)
The Unix version reads its font information from a file called fontnames.
The NLM version reads its font information from a file called fontname.txt.

(3)
The NLM version expects its mapping files to be in DOS text format. (CR+NL)
