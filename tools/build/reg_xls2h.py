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


import os, sys, getopt, re
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


def ext_path(scons_path):
    """Convert an absolute SCons path (with eventual #)
       to an normal path for external tools"""

    if scons_path[0] == '#':
        if len(scons_path) > 1 and scons_path[1] == '/':
            scons_path = scons_path[2:]
        else:
            scons_path = scons_path[1:]

        scons_path = join(Dir('#').abspath, scons_path)
    return os.path.normpath(scons_path)

def build_reg(target, source, regaddr,sheetname,regaccess,regprefix):
    xlsname = os.path.normpath(str(source[0]))
    hname = os.path.normpath(str(target[0]))
    match = re.match("_(.*)", os.path.basename(hname))
    if match != None:
        hname = os.path.join(os.path.dirname(hname), match.group(1))

    print("build reg: %s %s %s %s %s"%(xlsname, hname, sheetname, regprefix, regaddr))

    # parse the XLS name
    try:
        regs = reg_xls2h(xlsname, sheetname, regaccess, regprefix, verbose=False)
        regs.read_xls()
        regs.gen_header("", regaddr, hname, False, True)
    except:
        print "Not able to generate header file reg_%s_%s.h, sheet maybe not exist" % (regprefix.lower(), sheetname.lower())


def build_regs(reg_list, reg_build_dir):
    """Setup the build rules for the firmware or simulation registers from the list"""

    # check if the registers build directory exists, if not create it
    if not os.path.isdir(reg_build_dir):
        os.makedirs(reg_build_dir)

    # loop on all the register definition
    for line in reg_list:
        # check that the format is met : .* blockname 0xXXXXXX (short|long) (sheetname)
        match = re.match("(.+)\s+([^ ]+)\s+(0x[0-9a-fA-F]+)\s+(.+)\s+(.+)\s+(short|long)\s*(\w+)?\s*(trace)?$", line)
        if match == None:
            #print("reglist.txt", 1, "line '%s' does not match format" % (line,))
            continue
        # build the filename
        print(match.group(1))
        xlsname = ext_path(match.group(1)).strip()
        basename, ext = os.path.splitext(os.path.basename(xlsname))
        blockname = match.group(2)
        #print(xlsname)
        # check that the input file exists
        if not os.path.isfile(xlsname):
            # If excel sheet does not exist, command is not added in the list
            # This is to avoid scons error when file is included under disabled
            # preprocessor flag
            continue

        # different cases for RF, modem, and everything else
        sheetname = "Registers"
        hname = os.path.join(reg_build_dir, "reg_" + blockname.lower())
        _hname = os.path.join(reg_build_dir, "_reg_" + blockname.lower())
        if match.group(7) != None:
            sheetname = match.group(7).strip()
            if sheetname != 'RF':
                hname += '_' + sheetname.lower()
                _hname += '_' + sheetname.lower()
        hname += ".h"
        _hname += ".h"
        #if match.group(8) != None and env['TRC'] != 'off':
        #    env['TRC_REGS'].append((xlsname, sheetname, regaccess))

        regaddr = match.group(3)
        regaccess = match.group(4)
        regprefix = match.group(5)
        regformat = match.group(6)
        #for i in range(1,7):
        #    print match.group(i)
        # command to run
        build_reg([hname, _hname], [xlsname], regaddr,sheetname,regaccess,regprefix)

def main():
    """
    [--sheet=XLS_sheet_name,] xls_file regaccess regprefix base_address header_file
    """
    # parse the command line
    try:
        opts, args = getopt.getopt(sys.argv[1:], "", ["simu", "short", "verbose", "sheet=", "list=", "outdir="])
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

    listname=''
    for opt in opts:
        if opt[0] =="--sheet":
            sheetname = opt[1]
        if opt[0] =="--list":
            listname = opt[1]
        if opt[0] =="--outdir":
            outdir = opt[1]
    
    if len(listname)>1:
        listfile=open(listname);
        build_regs(listfile,outdir)
        return
    
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
