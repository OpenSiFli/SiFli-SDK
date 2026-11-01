# Subband Codec (SBC)

The SBC codec is the mandotory codec for Bluetooth audio over A2DP Profile

The technical specification of the codec is covered in Appendix B of [_Advanced Audio Distribution_](https://www.bluetooth.org/docman/handlers/downloaddoc.ashx?doc_id=457083)

mSBC extension is covered by Appendix A of [_Hands-Free Profile_](https://www.bluetooth.org/DocMan/handlers/DownloadDoc.ashx?doc_id=489628)

## Overview

The directory layout is as follows :
- include:      Library interface
- src:          Source files
- tools:        Standalone encoder/decoder tools
- build:        Building outputs
- bin:          Compilation output

## How to build

The default toolchain used is GCC. Invoke `make` to build the library.

```sh
$ make -j
```

Compiled library `libsbc.a` will be found in `bin` directory.

#### Cross compilation

The cc, as, ld and ar can be selected with respective Makefile variables `CC`,
`AS`, `LD` and `AR`. The `AS` and `LD` selections are optionnal, and fallback
to `CC` selection when not defined.

```sh
$ make -j CC=path_to_toolchain/bin/toolchain-prefix-gcc CFLAGS="..."
```

Compiled library will be found in `bin` directory.

#### Enabling assembly

Assembly code is available for armv7-em architecture. Enabling assembly code
is done with the `ARCH` Makefile variable. Following example enable assembly
optimization for target CPU ARM Cortex M4 :

```sh
$ make -j CC=path_to_arm_toolchain/bin/toolchain-prefix-gcc \
    CFLAGS="-mcpu=cortex-m4 -mthumb" ARCH="arm-v7em"
```
## Tools

Tools can be all compiled, while involking `make` as follows :

```sh
$ make tools
```

The standalone encoder `esbc` take a `wave` file as input and encode it
according given parameter. The standalone decoder `dsbc` do the inverse
operation.

Refer to `esbc -h` or `dsbc -h` for options.

Note that `esbc` output bitstream to standard output when output file is
omitted. On the other side `dsbc` read from standard input when input output
file are omitted.
In such way you can easly test encoding / decoding loop with :

```sh
$ ./esbc <in.wav> -b <bitpool> | ./dsbc > <out.wav>
```

Adding Linux `aplay` tools, you are able to instant hear the result :

```sh
$ ./esbc <in.wav> -b <bitpool> | ./dsbc | aplay
```
