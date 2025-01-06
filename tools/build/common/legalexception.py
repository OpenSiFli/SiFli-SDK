########################################################################################
#
# @file legalexception.py
#
# @brief Common functions and classes
#
# Copyright (C) RivieraWaves 2009-2015
#
########################################################################################

class LegalException(Exception):
    "class representing an exception handled by the tool"
    def __init__(self, value, errornumber=0):
        # convert the value parameter into a string for print
        if isinstance(value, str):
            self.value = value
        else:
            self.value = repr(value)
        # record the errornumber to return to the OS
        self.errornumber = errornumber
    def __str__(self):
        "convert the exception into a string"
        return self.value



    
