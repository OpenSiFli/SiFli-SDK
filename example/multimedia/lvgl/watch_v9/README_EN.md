# Watch Interface Example
(For a general overview of examples and their usage, please refer to the `README.md` file in the parent "examples" directory.)

Source Code Path: example/multimedia/lvgl/watch_v9

This example implements a smartwatch interface using LVGL v9, including the following main functional interfaces:
- Honeycomb main menu
- Watch face
- Cube rotation (not supported on SF32lb55x series chips)

## Usage

The following sections provide only absolutely necessary information. For complete steps on configuring SiFli-SDK and using it to build and run projects, please refer to the [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html).

### Supported Development Boards
```{note}
- 520-hdk is not supported
```

### Hardware Requirements
This example does not require special hardware and can run directly on supported development boards.

### Configure Project

#### Specify Fonts
Refer to `src/resource/fonts/SConscript`, add font .c files, and use extern declarations where needed

## Troubleshooting
Users may encounter the following compatibility issues when using this example:
- Running on 520-hdk development board will fail
- SF32lb55x series chips do not support cube rotation function

For any technical questions, please submit an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.