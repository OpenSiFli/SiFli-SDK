#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

/* fuzz target entry point, works without libFuzzer */

extern int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size);

int
main(int argc, char **argv) {
    ZIP_FILE *f = NULL;
    char *buf = NULL;
    long siz_buf;

    if (argc < 2) {
        fprintf(stderr, "no input file\n");
        goto err;
    }

    f = libzip_fopen(argv[1], "rb");
    if (f == NULL) {
        fprintf(stderr, "error opening input file %s\n", argv[1]);
        goto err;
    }

    libzip_fseek(f, 0, SEEK_END);

    siz_buf = libzip_ftell(f);
    rewind(f);

    if (siz_buf < 1) {
        fprintf(stderr, "zero-byte file not supported\n");
        goto err;
    }

    buf = (char *)zip_alloc(siz_buf);
    if (buf == NULL) {
        fprintf(stderr, "zip_alloc() failed\n");
        goto err;
    }

    if (libzip_fread(buf, siz_buf, 1, f) != 1) {
        fprintf(stderr, "libzip_fread() failed\n");
        goto err;
    }
    libzip_fclose(f);
    f = NULL;

    (void)LLVMFuzzerTestOneInput((uint8_t *)buf, siz_buf);

 err:
    if (f) {
        libzip_fclose(f);
    }
    zip_free(buf);

    return 0;
}
