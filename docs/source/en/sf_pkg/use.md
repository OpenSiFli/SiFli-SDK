# Using SiFli Package Registry in Your Project

The following steps explain how to install and use SiFli Package Registry dependencies in an existing project.

```{note}
Some screenshots on this page were captured with the old CLI format. If a screenshot differs from the text, follow the current `sdk.py sf-pkg ...` commands in the document.
```

## Initialize Dependencies (sf-pkg init)

```bash
sdk.py sf-pkg init
```

Upon successful execution, a `conanfile.py` file will be generated in the `project` directory.

`conanfile.py` is the Conan configuration file used to define the dependencies required by the project. For detailed information, refer to the [Conan Official Documentation](https://docs.conan.io/en/latest/reference/conanfile.html). For now, you can ignore most of the content and focus only on the `requires` field.

Generally, the auto-generated `conanfile.py` file contains a `requires` field similar to the following:

```python
requires = (
        # "core-lib/1.0.0",
    )
```

This indicates that no dependencies have been added. You can manually add the required packages. For example, to add an SHT30 sensor driver package:

```python
requires = (
    "sht30/0.0.4@caisong123",
)
```

The format for each dependency is `<package_name>/<version>@<username>`.

## Search for Available Packages

If you are unsure about the package name or version, you can search for it:

```bash
sdk.py sf-pkg search <package_name>
```

Example:

```bash
sdk.py sf-pkg search sht30
```
You can also search directly on the official website of the SiFli Package Registry: Click here to visit [SiFli Package Registry](https://packages.sifli.com/)


## Install Dependencies (sf-pkg install)

Execute the following command in the `project` directory of your project:

```bash
sdk.py sf-pkg install
```

![Install Dependencies](./assert/sf-pkg-install.png)

After successful installation, an `sf-pkgs` folder will be generated in the `project` directory, containing the installed packages.

## Using the Driver

Once installation is complete, you can use the driver directly:

- You can compile immediately
- When including header files, there is no need to specify absolute paths—Conan will automatically handle path configuration

### Kconfig Configuration Notes

- `menuconfig` will automatically integrate all `Kconfig` files under the `sf-pkgs` folder
- These configuration options will appear in the **SiFli External Components** menu within `menuconfig`
