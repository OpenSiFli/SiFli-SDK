# CMock — vendored, currently INERT

SiFli SDK vendoring of the CMock mock/stub generator.

## Provenance

| Item | Value |
| --- | --- |
| Upstream project | CMock — Automatic Mock Generation for C (ThrowTheSwitch.org) |
| Upstream URL | https://github.com/ThrowTheSwitch/CMock |
| Vendored version | 2.7.0 (released July 2026) |
| Source archive | `CMock-2.7.0.zip` (official ThrowTheSwitch release zip) |
| License | MIT (see `LICENSE`) |

## This is a host-side generator, not firmware code

CMock is a **Ruby program that runs on a host PC**. It parses a C header and
writes a `mock_<header>.c` / `mock_<header>.h` pair. Only those *generated*
files are C source that a firmware build could ever compile; the Ruby under
`lib/` is never compiled into firmware and never runs on the target.

Vendored layout:

- `lib/` — the Ruby generator (parser, plugin manager, plugins, file writer).
- `config/` — the Ruby environment helpers the generator loads.
- `scripts/` — `create_makefile.rb`, a host-side makefile helper.
- `src/` — the CMock **C runtime** (`cmock.c`, `cmock.h`, `cmock_internals.h`)
  that generated mocks call into. This is the only part that would ever be
  compiled into a test binary.
- `docs/`, `examples/`, `README.md` — upstream documentation, kept verbatim.
- `LICENSE` — the upstream `LICENSE.txt`, copied unchanged (MIT).

Omitted from the upstream release: `.github/`, `.gitignore`, `.gitattributes`,
`.gitmodules`, `Gemfile`, `cmock.gemspec`, `test/` and `vendor/`. The
`.gitmodules` file was dropped because it declares the `vendor/unity` and
`vendor/c_exception` submodules, which are deliberately not vendored here and
would otherwise be a dangling reference.

## Intended workflow (later phase)

1. On a host with Ruby, run the generator once to produce the mock sources for
   a header, driven by a `cmock_config.yml` in the test project:

   ```
   ruby external/cmock/lib/cmock.rb -o cmock_config.yml path/to/header.h
   ```

2. **Commit the generated `mock_*.c` / `mock_*.h` files** into the test project.
   Generating them ahead of time means the firmware build itself needs no Ruby
   and no generator run — SCons just compiles the checked-in C files.

3. Register the generated sources and `external/cmock/src/*.c` in the test
   project's own `SConscript`, gated on that project's config symbol.

## Status: INERT — deliberately not wired into anything

Mocking is deferred to a later phase. This directory is therefore **not**
referenced by any build or configuration:

- **There is no `external/cmock/SConscript` — on purpose.** `external/SConscript`
  discovers components by looking for `<dir>/SConscript`, so the absence of that
  file is what keeps this directory out of the build. Do not add one until the
  mocking phase starts.
- **There is no `external/cmock/Kconfig`, and no `source` line for it in
  `external/Kconfig` — on purpose.** CMock must not appear in menuconfig.

Adding either file is a deliberate act that belongs to the later mocking phase.
