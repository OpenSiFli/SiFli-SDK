# Creating and Uploading SiFli Package Registry Packages

This section describes how to log in, create, build, and upload your own SiFli
Package Registry packages.

```{note}
Some screenshots on this page were captured with the old CLI format. If a screenshot differs from the text, follow the current `sdk.py sf-pkg ...` commands in the document.
```

## Obtaining an Access Token

1. Open <https://packages.sifli.com/zh> and log in using your GitHub account.
   Your username will be your GitHub username (all lowercase).
2. After logging in, navigate to **Profile**.
3. In your profile center, apply for an access token (Token) and save it
   securely. This token will be used for the `sdk.py sf-pkg login` command.

![Log in to the website](./assert/log_in_to_the_website.png) ![Navigate to
Profile](./assert/enter_profile.png) ![Create Token](./assert/create_token.png)
![Obtain Token](./assert/get_token.png)

```{note}
Each user only needs to log in once per computer. Credentials are encrypted locally, and multiple users can be stored on the same machine.
```

## Log in to SiFli Package Registry

```bash
sdk.py sf-pkg login -u <lowercase_GitHub_username> -t <obtained_token>
```

![Login Successful](./assert/sdk-pkg-login.png)

```{warning}
The -u parameter must be your lowercase GitHub username! Otherwise, the upload will fail.
```

## Multi-user Management and User Selection

List locally stored users:

```bash
sdk.py sf-pkg users
```

Switch active user:

```bash
sdk.py sf-pkg use --name <namespace>
```

Show current active user:

```bash
sdk.py sf-pkg current-user
```

Notes:

- The selected user is mapped to a Conan remote with the same name, and remote
  operations use `-r=<namespace>`.
- If no user is selected, remote commands will ask you to login or switch user
  first.
- If the same-name remote is missing, has a mismatched URL, or is authenticated
  as another user, local credentials for that user are cleared and you will be
  asked to login again.
- `sf-pkg logout --name <namespace>` also clears the same-name Conan remote for
  that user.

To override user for a single command, use the `sf-pkg` group `--user` option:

```bash
sdk.py sf-pkg --user <namespace> upload --name <package_name>/<version>@<namespace>
```

## Create Package Configuration (sf-pkg new)

After preparing the driver folder, navigate to that directory in the terminal
and execute:

```bash
sdk.py sf-pkg new --name <package_name>
```

By default, this uses the active user. To temporarily select another user:

```bash
sdk.py sf-pkg --user <namespace> new --name <package_name>
```

Optional parameters:

- `--version`: Package version number
- `--license`: License declaration
- `--author`: Author
- `--support-sdk-version`: Supported SiFli-SDK version

Example (with version and author information):

```bash
sdk.py sf-pkg new --name <package_name> --version 1.0.0 --author yourname
```

Upon successful execution, a `conanfile.py` file will be generated. For detailed
information, refer to the [Conan Official
Documentation](https://docs.conan.io/en/latest/reference/conanfile.html).

Generally, the default content of the `conanfile.py` file is as follows:

```python
from conan import ConanFile

class Example_AddRecipe(ConanFile):
    name = "example_add"
    version = "0.1.0"

    license = "Apache-2.0"
    user = "halfsweet"
    author = "halfsweet"
    url = "<Package recipe repository url here, for issues about the package>"
    homepage = "<Package homepage here>"
    description = "<Description of hello package here>"
    topics = ("<Put some tag here>", "<here>", "<and here>")

    support_sdk_version = "^2.4"

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = "*"

    python_requires = "sf-pkg-base/[^1.0]@sifli"
    python_requires_extend = "sf-pkg-base.SourceOnlyBase"


    def requirements(self):
        # add your package dependencies here, for example:
        # self.requires("fmt/8.1.1")
        pass
```

Pay close attention to the following parameters:
- `name`: Package name, required. Used as a unique identifier on
  `https://packages.sifli.com`.
- `version`: Package version number. It is recommended to use semantic
  versioning, such as `0.0.1`, `1.0.0`, etc.
- `user`: Your username (also called `namespace`) on
  `https://packages.sifli.com`, required.
- `license`: Open-source license. Common options include `Apache-2.0`, `MIT`,
  `GPL-3.0`, `BSD-3-Clause`, etc. The default is `Apache-2.0`.
- `author`: Package author/maintainer name, optional.
- `url`: Package repository URL (GitHub/GitLab, etc.), optional.
- `description`: Detailed description of the package, optional.
- `topics`: Package tags/topics for easier searching and categorization. Use
  tuple format, optional.
- `support_sdk_version`: Supported SiFli-SDK version, required. Format follows
  [Semantic Versioning range](https://semver.org/).

The combination of `user` and `name` serves as the unique identifier for the
package.

The `requirements` method is used to add package dependencies:

```python
def requirements(self):
    # add your package dependencies here, for example:
    # self.requires("fmt/8.1.1")
    pass
```

This indicates that no dependencies have been added. You can add required
packages in the `requirements` method. For example:

```python
def requirements(self):
    self.requires("sht30/0.0.4@caisong123")
```

`self.requires` is used to specify dependencies in the format
`package_name/version@username`.

## Build Package (sf-pkg build)

Execute the following command in the driver folder:

```bash
sdk.py sf-pkg build --version <version_number>
```

![Build Package](./assert/sf-pkg-build.png)

> It is recommended to use semantic versioning for the version number, such as
> `0.0.1`, `1.0.0`, etc.

## Upload Package (sf-pkg upload)

```bash
sdk.py sf-pkg upload --name <package_name>/<version>@<username>
```

By default, this uses the active user. To temporarily select another user:

```bash
sdk.py sf-pkg --user <namespace> upload --name <package_name>/<version>@<namespace>
```

![Upload Package](./assert/sf-pkg-upload.png)

Command format explanation:

- `package_name`: The package name defined in `conanfile.py`
- `version`: The version number specified during build
- `username`: Your GitHub username

### Handling Upload Failures

1. Clear local cache:
   ```bash
   sdk.py sf-pkg remove --name <package_name>
   ```
2. (Optional) Remove the package from the remote repository:
   ```bash
   sdk.py sf-pkg remove --name <package_name>/<version>@<username> --remote
   ```
   Note: `--remote` removes the package from the selected user's same-name
   remote (`-r=<namespace>`). Please login first and ensure user state is valid.
3. Rebuild:
   ```bash
   sdk.py sf-pkg build --version <version_number>
   ```
4. Upload again:
   ```bash
   sdk.py sf-pkg upload --name <package_name>/<version>@<username>
   ```

### Verify Upload Result

After a successful upload, you can view the uploaded package on the server
website:

![Package on Server](./assert/pkg_in_website.png)


## 作为组织使用

组织用于对多个用户共同维护的组件包进行统一管理，是面向团队和企业级使用场景的核心功能。

组织可包含多个成员，不同成员可共享同一组织下的组件包与发布流程。

作为组织和作为个人使用时，大多数步骤均相同，只有令牌的获取方式和命名空间的不同。

### 创建组织令牌

1. 打开<https://packages.sifli.com/zh>，使用 GitHub 账号登录。
2. 登陆后进入组织管理。
3. 找到想要管理的组织，点击管理
4. 在组织管理申请访问令牌（Token），并妥善保存，后续用于 `sdk.py sf-pkg login` 命令。

![进入组织管理](./assert/enter_org.png) ![管理组织](./assert/check_org.png)
![创建token](./assert/create_org_token.png) ![获取token](./assert/get_org_token.png)

```{note}
组织 token 和个人 token 可以同时保存在本地，可通过 `sf-pkg use` 或 `sf-pkg --user` 在不同命名空间间切换。
```

### 有关命名空间的说明

1. When logging in as an organization, the namespace corresponds to the
   organization name. Therefore, use the following command to log in via the
   terminal:

    ```bash
    sdk.py sf-pkg login -u <org_name> -t </org_name>
    ```

   where `org_name` represents the name of the organization.

2. When creating a package configuration, the `author` field is independent of
   the `namespace` and represents the specific author within the organization.
   Ensure that the `user` field matches the organization name; **otherwise, the
   component package cannot be uploaded successfully**.
