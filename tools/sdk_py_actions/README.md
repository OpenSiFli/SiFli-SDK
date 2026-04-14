# sdk.py extensions

Python modules in this directory named `*_ext.py` are auto-loaded as sdk.py extensions.

## Extension API

Each extension module must provide:

- `EXTENSION_ID` (string)
- `EXTENSION_VERSION` (string)
- `EXTENSION_API_VERSION = 2`
- `MIN_SDK_VERSION` (string or `None`)
- `register(registry)` function

Example:

```python
from sdk_py_actions.cli.registry import CommandRegistry

EXTENSION_ID = "demo"
EXTENSION_VERSION = "1.0.0"
EXTENSION_API_VERSION = 2
MIN_SDK_VERSION = None


def register(registry: CommandRegistry) -> None:
    registry.group(path="demo", help="Demo commands")
    registry.command(
        path="demo/hello",
        callback=hello,
        help="Print hello",
        options=[{"names": ["--name"], "default": "world"}],
    )


def hello(sdk_ctx, name: str) -> None:
    print(f"hello {name}")
```

## External extension path

Use `SIFLI_SDK_EXTRA_ACTIONS_PATH` to provide extra extension directories.
Multiple directories should be separated by `;`.

Built-in extension load failures are fatal.
External extension load failures are warnings by default, and fatal with `--strict-extensions`.
