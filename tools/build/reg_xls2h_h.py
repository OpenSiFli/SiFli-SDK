########################################################################################
#
# @file reg_xls2h_h.py
#
# @brief translate register information from xls to header
#
# Copyright (C) RivieraWaves 2009-2015
#
########################################################################################

from common.legalexception import *
from reg_xls2h_xls import *
from os.path import basename
from os.path import dirname
from os.path import splitext
from math import ceil

def translate(name):
    # by default, just uppercase
    result = name.upper()

    # if the entire name is already uppercase, do not do anything
    if result != name:
        # first letter has to be uppercase
        result = name[0].upper()
        # loop on the remaining letters
        for i in range(1, len(name)):
            c = name[i]
            # check if current char is uppercase
            if c.isupper():
                # if it is uppercase:
                #   - if previous char is lower, add _ ; example: CamelKase -> _ before K
                #   - if next char is lower, add _ ; example: DTIMKill -> _ before K
                if name[i-1].islower() \
                   or (i < (len(name)-1) and name[i+1].islower()):
                    result += '_'
                result += c
            # check if current char is digit
            elif c.isdigit():
                # add underscore unless previous char is not digit
                if name[i-1].isdigit() == False:
                    # then add underscore before digit
                    result += '_'
                result += c
            # finally, char is not upper and is not digit, just make it upper
            else:
                result += c.upper()
    return result


class header:
    "class of header file"
    def __init__(self, prefix, regaddr, outfile):
        "constructor"

        # save the prefix
        self.prefix = prefix

        # check the values
        try:
            self.regaddr = int(regaddr, 0)
        except ValueError:
            raise LegalException("""ERROR: regaddr "%s", format is not parsable"""%(regaddr, ))

        self.outfile = outfile

    def generate(self, xls, forsimu, longname, gendoc):
        "method to generate the header file"

        # only one block containing all the registers in the xls sheet
        block = xls.blocks[0]

        # open the file
        try:
            fout = open(self.outfile, "wb")
        except:
            raise LegalException("""Error: can not create file "%s" """ %(self.outfile, ), 1)

        # open the size file
        name = basename(self.outfile)
        dirn = dirname(self.outfile)
        filename = dirn + "/_" + name
        try:
            _fout = open(filename, "wb")
        except:
            raise LegalException("""Error: can not create file "%s" """ %(filename, ), 1)

        (fname, fext) = splitext(basename(self.outfile))
        blockname = fname.upper()
        groupname = fname.lower()

        # disclaimer and documentation
        if gendoc:
            fout.write("/**\n")
            fout.write(" * @file %s\n"%(basename(self.outfile), ))
            fout.write(" * @brief Register map and register access functions.\n")
            fout.write(" *\n")
            fout.write(" * This module provides the following definitions and functions.\n")
            fout.write(" *\n")
            fout.write(" * <h3>Register definitions</h3>\n")
            fout.write(" * <h4>{REGNAME}_REG</h4>\n")
            fout.write(" * A dereferenced pointer to the register.  This can be used to read or write the register\n")
            fout.write(" * as if it were a ordinary variable.  Examples:\n")
            fout.write(" * <pre>uint32_t local_reg;\n")
            fout.write("\n")
            fout.write("local_reg = ARB_REG2_REG; // read\n")
            fout.write("local_reg |= 0x1;         // modify\n")
            fout.write("ARB_REG2_REG = local_reg; // write</pre>\n")
            fout.write(" *\n")
            fout.write(" * These registers should not be accessed directly.  Instead the wrapper functions\n")
            fout.write(" * provided herein should be used.\n")
            fout.write(" *\n")
            fout.write(" * <h4>{REGNAME}_ADDR</h4>\n")
            fout.write(" * This is simply the address of the register.  Avoid using this.\n")
            fout.write(" *\n")
            fout.write(" * <h3>Atomic Register Access Functions</h3>\n")
            fout.write(" * Get/set functions are provided for every register which is readable or writable,\n")
            fout.write(" * respectively.\n")
            fout.write(" *\n")
            fout.write(" * <h4>{regname}_get()</h4>\n")
            fout.write(" * These functions return the value of the register.\n")
            fout.write(" *\n")
            fout.write(" * <h4>{regname}_set(value)</h4>\n")
            fout.write(" * These functions simply set the register to an integer value.\n")
            fout.write(" *\n")
            fout.write(" * <h3>Register Packing and Unpacking Functions</h3>\n")
            fout.write(" * <h4>{regname}_pack(f1, f2, ...)</h4>\n")
            fout.write(" * This function takes as arguments a value for each field within the register\n")
            fout.write(" * and write the packed value to the register.  Example usage:\n")
            fout.write(" * <pre>local_val = arb_reg2_pack(1, 0, 3);</pre>\n")
            fout.write(" *\n")
            fout.write(" * <h4>{regname}_unpack(&f1, &f2, ...)</h4>\n")
            fout.write(" * This function takes as arguments pointers to integers; these integers will be populated\n")
            fout.write(" * with the value of their corresponding field from the current value of the register.  Example:\n")
            fout.write(" * <pre>uint8_t fpga_en, override_en, override_cnt;\n")
            fout.write("arb_reg2_unpack(&fpga_en, &override_en, &override_cnt);\n")
            fout.write("\n")
            fout.write("if(fpga_en) { ... }\n")
            fout.write(" * </pre>\n")
            fout.write(" *\n")
            fout.write(" * <h3>Register Field Access Functions</h3>\n")
            if longname:
                fout.write(" * <h4>{regname}_{fieldname}_getf()</h4>\n")
                fout.write(" * This function works similar to the unpack functions except they extract only the value\n")
                fout.write(" * of a single field from a register.  Example:\n")
                fout.write(" *<pre>uint8_t fpga_en = arb_reg2_fpga_en_getf();</pre>\n")
                fout.write(" *\n")
                fout.write(" * <h4>{regname}_{fieldname}_setf(fieldValue)</h4>\n")
                fout.write(" * This function takes a value for a field, read the current value of the\n")
                fout.write(" * register, modify with the new field value, and write the result back\n")
                fout.write(" * to the register.  Example:\n")
                fout.write(" * <pre>arb_reg2_fpga_en_setf(0);</pre>\n")
            else:
                fout.write(" * <h4>{fieldname}_getf()</h4>\n")
                fout.write(" * This function works similar to the unpack functions except they extract only the value\n")
                fout.write(" * of a single field from a register.  Example:\n")
                fout.write(" *<pre>uint8_t fpga_en = fpga_en_getf();</pre>\n")
                fout.write(" *\n")
                fout.write(" * <h4>{fieldname}_setf(fieldValue)</h4>\n")
                fout.write(" * This function takes a value for a field, read the current value of the\n")
                fout.write(" * register, modify with the new field value, and write the result back\n")
                fout.write(" * to the register.  Example:\n")
                fout.write(" * <pre>fpga_en_setf(0);</pre>\n")
            fout.write(" *\n")
            fout.write(" * @defgroup registers Registers\n")
            fout.write(" * @defgroup %s %s\n"%(groupname, groupname))
            fout.write(" * @ingroup registers\n")
            fout.write(" *\n")
            fout.write(" */\n")

        # generate the lines in the file
        define = "_" + basename(self.outfile).upper().replace(".", "_") + "_"
        
        # Size file
        _fout.write("#ifndef _%s\n"%define)
        _fout.write("#define _%s\n\n"%define)

        _fout.write("#define %s_SIZE %d\n\n"%(blockname, block.size))
        _fout.write("#define %s_BASE_ADDR 0x%08X\n\n"%(blockname, self.regaddr))
        _fout.write("\n#endif // _%s\n\n"%define)

        # Normal file
        fout.write("#ifndef %s\n"%define)
        fout.write("#define %s\n\n"%define)
        fout.write("#include <stdint.h>\n")
        fout.write("#include \"_%s\"\n"%basename(self.outfile))
        if not forsimu:
            fout.write("#include \"compiler.h\"\n")
            fout.write("#include \"arch.h\"\n")
            if xls.type in ['em_bt','em_ble','em_common']:
                fout.write("#include \"em_map.h\"\n")
            fout.write("#include \"reg_access.h\"\n")
        
        fout.write("\n")

        if xls.type == 'RF' and not forsimu:
            fout.write("static void phy_reg_write(uint32_t addr, uint32_t ch, uint32_t value);\n\n")

        if gendoc:
            fout.write("/** @brief Number of registers in the %s peripheral.\n"%(blockname, ))
            fout.write(" * @ingroup %s\n"%(groupname, ))
            fout.write(" */\n")
        fout.write("#define %s_COUNT %d\n\n"%(blockname, block.num_regs))
        if gendoc:
            fout.write("/** @brief Decoding mask of the %s peripheral registers from the CPU point of view.\n"%(blockname, ))
            fout.write(" * @ingroup %s\n"%(groupname, ))
            fout.write(" */\n")
        fout.write("#define %s_DECODING_MASK 0x%08X\n\n"%(blockname, block.decode_mask))
        if gendoc:
            fout.write("/** @brief Macro to get the address of a %s element\n"%(blockname, ))
            fout.write(" * @ingroup %s\n"%(groupname, ))
            fout.write(" */\n")
        if xls.type in ['em_bt','em_ble','em_common']:
            fout.write("#define %s_ADDR_GET(idx) (%s_%s_OFFSET + (idx) * %s_SIZE)\n\n"%(blockname, xls.access, xls.sheetname, blockname))
        # loop on all the registers
        for reg in block.registers:
            # if the regname is a noregister do not put it
            if reg.name.upper().find("NOREGISTER") != -1:
                continue
            
            if xls.prefix == "RPL":
                self.inc = 1
            elif xls.prefix == "THT":
                self.inc = 1
            elif xls.prefix in ['CLP_BANK0','CLP_BANK1','CLP_BANK2','CLP_BANK3','CLP_BANK4']:
                self.inc = 1
            else:
                self.inc = reg.width
            
# reg condition removed as it does not affect SW behavior
#            if reg.cond != '':
#                fout.write("#if %s\n"%reg.cond)
            
            if reg.width == 4:
                itype = "uint32_t"
            elif reg.width == 2:
                itype = "uint16_t"
            else:
                itype = "uint8_t"

            # do the initial name translation and creation before generation
            #  - remove register trailing Reg
            regname = re.sub("Reg$", '', reg.name)
            cs_regname = re.sub("Reg$", '', reg.cs_name)
            #  - translate into our naming standard
            cs_regname = translate(cs_regname)
            regname = translate(regname)

            if longname and (len(reg.fields) != 1 or reg.fields[0].name != cs_regname):
                field_prefix = cs_regname + '_'
            else:
                field_prefix = reg.short_name

            #  - generate the names required for the parameters
            for field in reg.fields:
                # in parameter name, remove underscore
                field.h_param = field.name.lower().replace('_', '')
                field.h_name = translate(field.name)

            #  - longest field name (append a zero to prevent problems in no field registers)
            max_fieldhname = max(map(lambda x:len(x.h_name), reg.fields) + [0]) + 6

            # add a potential prefix to the register name
            regname = self.prefix + regname
            cs_regname = self.prefix + cs_regname

            # generate description
            fout.write("/**\n")
            fout.write(" * @brief %s register definition\n"%(regname,))
            if len(reg.fields) != 0:
                fout.write(" * <pre>\n")
                fout.write(" *   Bits           Field Name   Reset Value\n")
                fout.write(" *  -----   ------------------   -----------\n")
                for field in reg.fields:
                    if field.width == 1:
                        fout.write(" *     %02d   %s   %d\n"%\
                                   (field.low_bitpos,
                                    " "*(18 - len(field.name)) + field.name,
                                    field.reset))
                    else:
                        fout.write(" *  %02d:%02d   %s   0x%X\n"%\
                                   (field.high_bitpos, field.low_bitpos,
                                    " "*(18 - len(field.name)) + field.name,
                                    field.reset))

                fout.write(" * </pre>\n")
            if gendoc:
                fout.write(" * @ingroup %s\n"%(groupname,))
            fout.write(" */\n")
            
            if xls.type in ['mdm50','mdmhp']:
                fout.write("#define %s_%s_%s_ADDR   0x%08X\n"%(regname,xls.prefix, regname, (reg.addr + self.regaddr)))
                fout.write("#define %s_%s_%s_OFFSET 0x%08X\n"%(regname,xls.prefix, regname, (reg.addr)))
                fout.write("#define %s_%s_%s_INDEX  0x%08X\n"%(regname,xls.prefix, regname, (reg.addr/self.inc)))
                fout.write("#define %s_%s_%s_RESET  0x%08X\n"%(regname,xls.prefix, regname, (reg.reset)))
            else:                              
                if xls.type in ['em_bt','em_ble','em_common']:
                    fout.write("#define %s_%s_ADDR   (0x%08X + %s_%s_OFFSET)\n"%(xls.prefix, regname, (reg.addr + self.regaddr), xls.access, xls.sheetname))
                else:
                    fout.write("#define %s_%s_ADDR   0x%08X\n"%(xls.prefix, regname, (reg.addr + self.regaddr)))
                    fout.write("#define %s_%s_OFFSET 0x%08X\n"%(xls.prefix, regname, (reg.addr)))
                fout.write("#define %s_%s_INDEX  0x%08X\n"%(xls.prefix, regname, (reg.addr/self.inc)))
                fout.write("#define %s_%s_RESET  0x%08X\n"%(xls.prefix, regname, (reg.reset)))
            
            if reg.multi:
                fout.write("#define %s_%s_COUNT  %d\n\n"%(xls.prefix, regname, reg.max - reg.min + 1))
            else:
                fout.write("\n")

            if xls.type in ['em_bt','em_ble','multi','em_common']:
                if reg.multi:
                    param_beg = "int elt_idx, int reg_idx, "
                    param_one = "int elt_idx, int reg_idx"
                else:
                    param_beg = "int elt_idx, "
                    param_one = "int elt_idx"
            else:
                if reg.multi:
                    param_beg = "int reg_idx, "
                    param_one = "int reg_idx"
                else:
                    param_beg = ""
                    param_one = "void"

            if not forsimu:
                if xls.type in ['em_bt','em_ble','multi','em_common']:
                    if reg.multi:
                        reg_rd = "%s_RD(%s_%s_ADDR + elt_idx * %s_SIZE + reg_idx * %d)"%(xls.access,xls.prefix, regname, blockname, self.inc)
                        reg_wr_beg = "%s_WR(%s_%s_ADDR + elt_idx * %s_SIZE + reg_idx * %d, "%(xls.access,xls.prefix, regname, blockname, self.inc)
                    else:
                        reg_rd = "%s_RD(%s_%s_ADDR + elt_idx * %s_SIZE)"%(xls.access,xls.prefix, regname, blockname)
                        reg_wr_beg = "%s_WR(%s_%s_ADDR + elt_idx * %s_SIZE, "%(xls.access,xls.prefix, regname, blockname)
                else:
                    if reg.multi:
                        reg_rd = "%s_RD(%s_%s_ADDR + reg_idx * %d)"%(xls.access,xls.prefix, regname, self.inc)
                        reg_wr_beg = "%s_WR(%s_%s_ADDR + reg_idx * %d, "%(xls.access,xls.prefix, regname, self.inc)
                    else:
                        if xls.type in ['mdm50','mdmhp']:
                            reg_rd = "%s_RD(%s_%s_%s_ADDR)"%(xls.access, regname, xls.prefix, regname)
                            reg_wr_beg = "%s_WR(%s_%s_%s_ADDR, "%(xls.access, regname, xls.prefix, regname)
                        else:
                            reg_rd = "%s_RD(%s_%s_ADDR)"%(xls.access, xls.prefix, regname)
                            reg_wr_beg = "%s_WR(%s_%s_ADDR, "%(xls.access, xls.prefix, regname)

                reg_wr_end = ")"

            else:
                # simu reg read/write macro setup
                sim_read = "sim_"+fname.lower()+"_rd"
                sim_write = "sim_"+fname.lower()+"_wr"

                if reg.multi:
                    reg_rd = "%s(%s_%s_INDEX + idx)"%(sim_read,xls.prefix,  regname)
                    reg_wr_beg = "%s(%s_%s_INDEX + idx, "%(sim_write,xls.prefix,  regname)
                else:
                    reg_rd = "%s(%s_%s_INDEX)"%(sim_read, xls.prefix, regname)
                    reg_wr_beg = "%s(%s_%s_INDEX, "%(sim_write, xls.prefix, regname)

                reg_wr_end = ")"


            if reg.multi:
                func_beg = "{\n    ASSERT_ERR(reg_idx <= %d);\n" % (reg.max)
            else:
                func_beg = "{\n"
            func_end = "}\n\n"

            if reg.sw == 'C':
                func_set_name = "clear"
            else:
                func_set_name = "set"

            # REG_get function
            if reg.sw != 'W' and reg.gen_read and not reg.rf and not forsimu:
                if gendoc:
                    fout.write("/**\n")
                    fout.write(" * @brief Returns the current value of the %s register.\n"%(regname,))
                    fout.write(" * The %s register will be read and its value returned.\n"%(regname,))
                    fout.write(" * @return The current value of the %s register.\n"%(regname,))
                    fout.write(" */\n")
                fout.write("__INLINE %s %s_%s_get(%s)\n"%(itype, xls.prefix.lower(), cs_regname.lower(), param_one))
                fout.write(func_beg)
                fout.write("    return %s;\n" % (reg_rd))
                fout.write(func_end)

            # REG_set function
            if reg.sw != 'R' and not forsimu:
                if gendoc:
                    fout.write("/**\n")
                    fout.write(" * @brief Sets the %s register to a value.\n"%(regname,))
                    fout.write(" * The %s register will be written.\n"%(regname,))
                    fout.write(" * @param value - The value to write.\n")
                    fout.write(" */\n")

                fout.write("__INLINE void %s_%s_%s(%s%s value)\n"%(xls.prefix.lower(), cs_regname.lower(), func_set_name, param_beg, itype))
                fout.write(func_beg)
                fout.write("    %svalue%s;\n"%(reg_wr_beg, reg_wr_end))
                fout.write(func_end)

            if len(reg.fields) != 0:
                if reg.gen_field:
                    fout.write("// field definitions\n")
                    
                    prefix="%s_"%xls.prefix
                    if xls.type in ['mdm50','mdmhp']:
                        str_def_hex = "#define %s_%s_%-" + str(max_fieldhname) + "s  ((%s)0x%08X)\n"
                        str_def_dec = "#define %s_%s_%-" + str(max_fieldhname) + "s  %d\n"
                        for field in reg.fields:
                            if field.width == 1:
                                fout.write(str_def_hex % (regname,xls.prefix,field.h_name + "_BIT  ", itype, field.mask)) 
                                fout.write(str_def_dec % (regname,xls.prefix,field.h_name + "_POS  ", field.low_bitpos))
                            else:
                                fout.write(str_def_hex % (regname,xls.prefix,field.h_name + "_MASK ",  itype, field.mask))
                                fout.write(str_def_dec % (regname,xls.prefix,field.h_name + "_LSB  ", field.low_bitpos))
                                fout.write(str_def_hex % (regname,xls.prefix,field.h_name + "_WIDTH", itype, field.width))
    
                        fout.write("\n")
    
                        str_def_hex = "#define %s_%s_%-" + str(max_fieldhname) + "s  0x%X\n"
                        for field in reg.fields:
                            fout.write(str_def_hex % (regname,xls.prefix,field.h_name + "_RST", field.reset))
                    else:
                        str_def_hex = "#define %s_%-" + str(max_fieldhname) + "s  ((%s)0x%08X)\n"
                        str_def_dec = "#define %s_%-" + str(max_fieldhname) + "s  %d\n"

                        for field in reg.fields:
                            if field.width == 1:
                                fout.write(str_def_hex % (xls.prefix,field.h_name + "_BIT  ", itype, field.mask)) 
                                fout.write(str_def_dec % (xls.prefix,field.h_name + "_POS  ", field.low_bitpos))
                            else:
                                fout.write(str_def_hex % (xls.prefix,field.h_name + "_MASK ",  itype, field.mask))
                                fout.write(str_def_dec % (xls.prefix,field.h_name + "_LSB  ", field.low_bitpos))
                                fout.write(str_def_hex % (xls.prefix,field.h_name + "_WIDTH", itype, field.width))
    
                        fout.write("\n")
    
                        str_def_hex = "#define %s_%-" + str(max_fieldhname) + "s  0x%X\n"
                        for field in reg.fields:
                            fout.write(str_def_hex % (xls.prefix,field.h_name + "_RST", field.reset))

                    fout.write("\n")
                else:
                    fout.write("// fields defined in symmetrical set/clear register\n")

                # only provide pack writable registers that have a writable field
                if reg.writable and len(reg.writable_fields) > 1 and not forsimu:

                    # REG_pack function
                    if gendoc:
                        fout.write("/**\n")
                        fout.write(" * @brief Constructs a value for the %s register given values for its fields\n"%(regname, ))
                        fout.write(" * and writes the value to the register.\n")
                        fout.write(" *\n")
                        for field in reg.writable_fields:
                            fout.write(" * @param[in] %s - The value to use for the %s field.\n"%(field.h_param, field.name))
                        fout.write(" */\n")
                    fout.write("__INLINE void %s_%s_pack(%s"%(xls.prefix.lower() ,regname.lower(),param_beg))
                    for field in reg.writable_fields:
                        fout.write("%s %s"%(field.type, field.h_param))
                        # check if it is the last element
                        if field == reg.writable_fields[-1]:
                            fout.write(")\n")
                        else:
                            fout.write(", ")
                    fout.write(func_beg)
                    for field in reg.writable_fields:
                        fout.write("    ASSERT_ERR((((%s)%s << %d) & ~((%s)0x%08X)) == 0);\n"%(itype, field.h_param, field.low_bitpos, itype, field.mask))

                    fout.write("    %s"%(reg_wr_beg))

                    for field in reg.writable_fields:
                        fout.write(" ((%s)%s << %d)"%(itype, field.h_param, field.low_bitpos))
                        # check if it is the last element
                        if field != reg.writable_fields[-1]:
                            fout.write(" |")
                        else:
                            fout.write("%s;\n" % reg_wr_end)

                    fout.write(func_end)

                # REG_unpack function
                if reg.sw != 'W' and reg.gen_read and len(reg.fields) > 1 and not forsimu:
                    if gendoc:
                        fout.write("/**\n")
                        fout.write(" * @brief Unpacks %s's fields from current value of the %s register.\n"%(regname, regname))
                        fout.write(" *\n")
                        fout.write(" * Reads the %s register and populates all the _field variables with the corresponding\n"%(regname,))
                        fout.write(" * values from the register.\n")
                        fout.write(" *\n")
                        for field in reg.fields:
                            fout.write(" * @param[out] %s - Will be populated with the current value of this field from the register.\n"%(field.h_param,))
                        fout.write(" */\n")

                    fout.write("__INLINE void %s_%s_unpack(%s"%(xls.prefix.lower(), cs_regname.lower(), param_beg))
                    for field in reg.fields:
                        fout.write("%s* %s"%(field.type, field.h_param))
                        # check if it is the last element
                        if reg.fields.index(field) == len(reg.fields)-1:
                            fout.write(")\n")
                        else:
                            fout.write(", ")
                    fout.write(func_beg)
                    fout.write("    %s localVal = %s;\n"%(itype, reg_rd))
                    fout.write("\n")
                    for field in reg.fields:
                        if len(reg.fields) == 1:
                            fout.write("    ASSERT_ERR((localVal & ~((%s)0x%08X)) == 0);\n"%(itype, field.mask,))
                            fout.write("    *%s = localVal >> %d;\n"%(field.h_param, field.low_bitpos))
                        else:
                            fout.write("    *%s = (localVal & ((%s)0x%08X)) >> %d;\n"%(field.h_param, itype, field.mask, field.low_bitpos))
                    fout.write(func_end)

            # loop on all the fields to generate the field related
            for field in reg.fields:
                if field.sw != 'W' and reg.gen_read and not reg.rf and not forsimu:
                    if gendoc:
                        fout.write("/**\n")
                        fout.write(" * @brief Returns the current value of the %s field in the %s register.\n"%(field.name, regname))
                        fout.write(" *\n")
                        fout.write(" * The %s register will be read and the %s field's value will be returned.\n"%(regname, field.name))
                        fout.write(" *\n")
                        fout.write(" * @return The current value of the %s field in the %s register.\n"%(field.name, regname))
                        fout.write(" */\n")
                    fout.write("__INLINE %s %s_%s%s_getf(%s)\n"%(field.type,xls.prefix.lower(),  field_prefix.lower(), field.h_name.lower(), param_one ))
                    fout.write(func_beg)
                    fout.write("    %s localVal = %s;\n"%(itype, reg_rd))

                    # do not mask for single field
                    if len(reg.fields) == 1:
                        fout.write("    ASSERT_ERR((localVal & ~((%s)0x%08X)) == 0);\n"%(itype, field.mask,))
                        fout.write("    return (localVal >> %d);\n"%(field.low_bitpos,))
                    else:
                        fout.write("    return ((localVal & ((%s)0x%08X)) >> %d);\n"%(itype, field.mask, field.low_bitpos))
                    fout.write(func_end)


                # only provide set functionality to writable fields
                if field.writable and not reg.rf and not forsimu:
                    if gendoc:
                        fout.write("/**\n")
                        fout.write(" * @brief Sets the %s field of the %s register.\n"%(field.name, regname))
                        fout.write(" *\n")
                        fout.write(" * The %s register will be read, modified to contain the new field value, and written.\n"%(regname, ))
                        fout.write(" *\n")
                        fout.write(" * @param[in] %s - The value to set the field to.\n"%(field.h_param,))
                        fout.write(" */\n")
                    fout.write(  "__INLINE void %s_%s%s_%sf(%s%s %s)\n"
                               % (xls.prefix.lower(), field_prefix.lower(), field.h_name.lower(), func_set_name,
                                  param_beg, field.type, field.h_param))
                    fout.write(func_beg)
                    fout.write("    ASSERT_ERR((((%s)%s << %d) & ~((%s)0x%08X)) == 0);\n"%(itype, field.h_param, field.low_bitpos, itype, field.mask))

                    # generate direct accesses for set/clear registers or single writable field field
                    if (reg.sw in ('S', 'C')) or (len(reg.writable_fields) == 1):
                        fout.write("    %s(%s)%s << %d%s;\n"%(reg_wr_beg, itype, field.h_param, field.low_bitpos, reg_wr_end))
                    else:
                        fout.write(  "    %s(%s & ~((%s)0x%08X)) | ((%s)%s << %d)%s;\n"
                                   % (reg_wr_beg, reg_rd, itype, field.mask, itype, field.h_param, field.low_bitpos, reg_wr_end))
                    fout.write(func_end)

# reg condition removed as it does not affect SW behavior
#            if reg.cond != '':
#                fout.write("#endif // %s\n"%reg.cond)

        fout.write("\n#endif // %s\n\n"%define)

        fout.close()
        _fout.close()

class bitMap():
    def __init__(self, instance_name):
        self._instance_name = instance_name
        self._bit_map_type = "BitMap"
        
    def generate(self, xls):
        bMap = "class %s (%s):\n"%(self._instance_name, self._bit_map_type)
        padding = ' '*(4)

        # only one block containing all the registers in the xls sheet
        block = xls.blocks[0]

        #write type and size
        bMap += "%s_type_   = uint%d_t\n"%(padding, block.width * 8)
        bMap += "%s_length_ = %d\n"%(padding, block.size/block.width)
        bMap += "%s_fields_ = {\n"%(padding)
        
        padding = ' '*(16)

        # loop on all the registers
        for reg in block.registers:
            # if the regname is a noregister do not put it
            if reg.name.upper().find("NOREGISTER") != -1:
                continue

            self.inc = reg.width

            # do the initial name translation and creation before generation
            #  - remove register trailing Reg
            regname = re.sub("Reg$", '', reg.name)
            entry_txt = "%s'%s' : (0x%02x, "%(padding, regname, reg.addr)
            if reg.multi:
                entry_txt += ("%d, "%(reg.max - reg.min + 1))
            entry_txt += "[\n"
            
            bMap += entry_txt
            if len(reg.fields) != 0:
                padding_entry = ' '*len(entry_txt)
                for field in reg.fields:
                    mask = hex(field.mask)
                    if reg.multi:
                        hex_num  = mask[2:]
                        hex_num *= reg.max - reg.min
                        mask    += hex_num
                    bMap += "%s('%s', 0x%02x, %s),\n"%(padding_entry, field.name, field.low_bitpos, mask)
                bMap += "%s]),\n"%(padding_entry)
            else:
                raise Exception("Number of fields = 0. Case to handle")
        bMap += "%s}"%(padding)
        return bMap
