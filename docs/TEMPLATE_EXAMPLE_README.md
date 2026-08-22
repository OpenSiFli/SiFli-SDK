_Note that this is a template for an SiFli-SDK example README.md file. When using this template, replace all the emphasized placeholders (in italics) with example-specific content._

_While creating your example, use this template in conjunction with the guidelines given in [Creating Examples](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/contribute/creating-examples.html)._

# _Example Title_

_Example Title should use the word "example" instead of "demo", "test" or any similar words._

(For general overview of examples and their usage, see the `README.md` file in the upper level 'examples' directory.)

_Give example source code path in the following format_
Source code path: example/xxxxx

_What is this example? What does it do?_

_What feature(s) of SiFli-SDK does this example use? Briefly introduce them and give a link to the API reference(s)._

_What could someone create based on this example? Think of applications, use cases, etc._

_If there are any acronyms or SiFli-only words used here, explain them or mention where in the hardware or software documentation this information can be found._


## Usage

The subsections below give only absolutely necessary information. For full steps to configure SiFli-SDK and use it to build and run projects, see [SiFli-SDK Getting Started](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html).

## Supported Boards

_This example can run on the following development boards:_
- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp
...

### Hardware Required

_If possible, the examples should be able to run on the development boards mentioned above. Otherwise, please indicate which specific hardware should be used._

_For beginner-friendly examples, provide a more detailed list of the required hardware. For more advanced examples, omit self-evident information._

_If any other items (server, BLE device, app, second chip, whatever) are needed, mention them here. Include the links if applicable. Explain how to set them up._

_For examples where a chip is connected to some other hardware, include a table or schematics with connection details._

### Software Required _(optional)_

_If any additional software is needed, mention it here and include the links. Mention the software version required if applicable. Explain how to set it up._


### Configure the Project

_Here you need to explain which menuconfig options need to be turned on by the routine, preferably with a screenshot_

```bash
sdk.py menuconfig --board=<board_name>
```

## Example Output

_For different reasons, an example's console output might not be informative enough for the user to realize that something went wrong. To allow users to check that the example is running correctly on their side, be sure to include here the expected console output as given below._

If you see the following console output, your example should be running correctly:

```
Paste console output as a code snippet like this
```

_If the user is supposed to interact with the example at this point, mention it here (read/write GATT attribute, press button, etc.)_


## Example Breakdown _(optional)_

_If the example source code is lengthy, complex, or cannot be easily understood, use this section to break down and explain the source code. This can be done by breaking down the execution path step by step, or explaining what each major function/task/source file does. Add sub titles if necessary. Remove this section for very simple examples where the source code is self explanatory._


## Troubleshooting

_Remove this section for very simple examples where nothing is likely to go wrong._

_There might be typical issues or errors that the user is likely to encounter while working with this example. If the issue is related to the SiFli-SDK feature in general, mention it in Troubleshooting of this feature's API reference. If the issue is related to this example only, mention such issue here._

For any technical queries, please open an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.


## Reference

_Provide references to important documents_
