########################################################################################
#
# @file reg_xls2h.py 
# 
# @brief Tool to read out register information from excel sheet and generate 
#        source header files
#
# Copyright (C) RivieraWaves 2009-2015
#
########################################################################################

usage_doc = """
Synopsis:
    reg_xls2h.py xlsname
          to test the validity of a register definition

    reg_xls2h.py [--verbose] [--simu] [--short] [--sheet=N] xlsname prefix regaddr outfile
          to generate the header file of a HW block

where:
    --verbose to turn on verbose mode
    --simu means that the file must be generated for simulation environment
    --short means that the file must generate short macros (field name
            not prefixed with register name)
    --sheet=N means that the XLS file provided is in the MODEM register definition
            format and that the sheet N must be parsed to extract the registers
    "xlsname" designates the excel file
    "prefix" designates the prefix to use for all the generated registers
    "regaddr" designates the base address of the registers
    "outfile" designates the file to generate the output to
    ""
"""


import os, sys, getopt
from common.legalexception import *
from reg_xls2h_xls import *
from reg_xls2h_h import *

class reg_xls2h:
    def __init__(self, xlsname, sheetname, access, prefix, verbose=False):
        self.verbose = verbose
        if self.verbose:
            print "  - Excel File Name : " + xlsname

        # check that the input file exists
        if not os.path.isfile(xlsname):
            raise LegalException("Error: excel file %s does not exist" % xlsname, 1)

        # create the entity to read the excel sheet
        self.xls = xls(xlsname, sheetname, access, prefix, self.verbose)

    def read_xls(self):
        self.xls.extract_all()

    def gen_header(self, prefix, regaddr, outfile, forsimu, longname):
        # create a header instance
        self.header = header(prefix, regaddr, outfile)
        # generate the file
        self.header.generate(self.xls, forsimu, longname, False)
    
    def gen_bitMap(self, name):
        #crate a bitMap instance and generate the bitMap string
        bmap = bitMap(name).generate(self.xls)
        return bmap

def usage():
    sys.stdout.write(usage_doc)


def main():
    """
    [--sheet=XLS_sheet_name,] xls_file regaccess regprefix base_address header_file
    """
    # parse the command line
    try:
        opts, args = getopt.getopt(sys.argv[1:], "", ["simu", "short", "verbose", "sheet="])
    except getopt.GetoptError:
        # print help information and exit:
        usage()
        sys.exit(2)

    verbose = False
    sheetname = "Registers"
    forsimu = False
    longname = True

    if ("--simu","") in opts:
        forsimu = True
    if ("--short","") in opts:
        longname = False
    if ("--verbose","") in opts:
        verbose = True

    for opt in opts:
        if opt[0] =="--sheet":
            sheetname = opt[1]

    if len(args) < 1:
        usage()
        sys.exit("Wrong number of arguments")

    # create the main instance
    my_reg_xls2h = reg_xls2h(args[0], sheetname, args[1], args[2], verbose)
    

    # read the excel sheet content
    my_reg_xls2h.read_xls()

    # check the number of parameters is correct
    if len(args) == 5:
        # generate the file only if requested
        my_reg_xls2h.gen_header("", args[3], args[4], forsimu, longname)
    elif len(args) != 1:
        # print help information and exit:
        usage()
        sys.exit("ERROR: unexpected number of parameters")


if __name__ == "__main__":
    try:
        main()
    except LegalException, e:
        print e
    except:
        raise
