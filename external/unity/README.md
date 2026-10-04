# Unity

SiFli SDK vendoring of the Unity C unit-test framework.

## Provenance

| Item | Value |
| --- | --- |
| Upstream project | Unity Test (ThrowTheSwitch.org) |
| Upstream URL | https://github.com/ThrowTheSwitch/Unity |
| Vendored version | 2.7.0 (released July 2026) |
| Source archive | `Unity-2.7.0.zip` (official ThrowTheSwitch release zip) |
| License | MIT (see `LICENSE`) |

## Vendored files

Taken verbatim from the 2.7.0 release:

- `src/unity.c`
- `src/unity.h`
- `src/unity_internals.h`
- `auto/type_sanitizer.rb`

`LICENSE` is the upstream `LICENSE.txt` copied unchanged.

`auto/type_sanitizer.rb` is **not** Unity runtime code. It is a host-side Ruby
helper that CMock pulls in while generating mocks: `cmock_generator.rb` looks
for it at `external/cmock/lib/../../unity/auto/` and aborts with `Failed to
find an instance of Unity to pull in type_sanitizer module!` when it is
missing. It is kept here so CMock works out of the box; the rest of `auto/` is
not. `UNITY_DIR=<path>` points CMock at a different Unity checkout instead.

## Deliberately omitted

Only the files needed to compile Unity into a firmware test binary are kept. The
following upstream content was intentionally left out to keep the tree small and
the license-scan surface minimal:

- `auto/` — the Ruby helper scripts, **except** `type_sanitizer.rb` (see
  above): `generate_test_runner.rb`, the `unity_test_summary.rb` /
  `stylize_as_junit.rb` report parsers and the `*.yml` build-target
  descriptions.
- `examples/` — the five host-side example projects.
- `docs/` — the user guides, assertion reference and the change log.
- `test/` — Unity's own self-test suite.
- CMake / Meson / PlatformIO build files (`CMakeLists.txt`, `meson.build`,
  `unityConfig.cmake`, `library.json`, `platformio-build.py`).

Rationale:

- This SDK uses **explicit `RUN_TEST()` registration** in a hand-written test
  runner (`main()` calling each test function), so the Ruby test-runner
  generator in `auto/` is not used and Ruby is not required to build or run the
  tests. Mock generation is the one place Ruby is still needed: CMock itself is
  a Ruby program, but it runs on the host once and its output is committed, so
  a firmware build or a test run still needs no Ruby.
- The build is driven by **SCons**, so the upstream CMake / Meson build files
  would never be evaluated and are dead weight.

## Build integration

`SConscript` registers a single group named `unity`, built only when
`RT_USING_UNITY` is selected in menuconfig (`Kconfig` in this directory).

## License

MIT. See `LICENSE` for the full text. The copyright holder is
Mike Karlesky, Mark VanderVoord and Greg Williams.
