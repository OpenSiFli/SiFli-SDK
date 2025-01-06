#!/bin/bash

export SIFLI_SDK="$(pwd)/"
export PYTHONPATH="$PYTHONPATH:$SIFLI_SDK/tools/build:$SIFLI_SDK/tools/build/default"
export PATH="$PATH:$SIFLI_SDK/tools/scripts"

export RTT_CC="keil"
export RTT_EXEC_PATH="/c/Keil_v5"
