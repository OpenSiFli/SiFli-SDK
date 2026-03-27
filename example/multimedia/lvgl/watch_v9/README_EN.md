# Watch Interface Example

(For a general overview of examples and their usage, please refer to the
`README.md` file in the parent "examples" directory.)

Source Code Path: example/multimedia/lvgl/watch_v9
- Honeycomb main menu
- Watch face
- Cube rotation (not supported on SF32lb55x series chips)

## Usage

The following sections provide only absolutely necessary information. For
complete steps on configuring SiFli-SDK and using it to build and run projects,
please refer to the [SiFli-SDK Quick
Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html).

### Supported Development Boards
+ sf32lb52-lcd series
+ sf32lb56-lcd series
+ sf32lb58-lcd series

```{note}
- 520-hdk is not supported
```

## Troubleshooting
Users may encounter the following compatibility issues when using this example:
- Running on 520-hdk development board will fail
- SF32lb55x series chips do not support cube rotation function
    - `font_file`: Path to the TTF file
    - `DroidSansFallback`: C file name

## Batch convert multiple TTF files to C files

Add the following code to the SConscript:

```python
font_objs1 = Env.ConvertFont(font_file1, 'DroidSansFallback')
font_objs2 = Env.ConvertFont(font_file2, 'DroidSansFallback_other_name')
font_objs = font_objs1 + font_objs2
```
