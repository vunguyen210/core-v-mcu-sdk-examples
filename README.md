# core-v-mcu-sdk-examples

Example SDK applications for DevKit

## Cloning this repository

This repo uses [Git Submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules) to bring in dependent components.

**Note:** If you download the ZIP file provided by the GitHub UI, you will not get the contents of the submodules. (The ZIP file is also not a valid git repository)

If using Windows, because this repository and its submodules contain symbolic links, set `core.symlinks` to true with the following command:

```
git config --global core.symlinks true
```

In addition to this, either enable [Developer Mode](https://docs.microsoft.com/en-us/windows/apps/get-started/enable-your-device-for-development) or, whenever using a git command that writes to the system (e.g. `git pull`, `git clone`, and `git submodule update --init --recursive`), use a console elevated as administrator so that git can properly create symbolic links for this repository. Otherwise, symbolic links will be written as normal files with the symbolic links' paths in them as text. [This](https://blogs.windows.com/windowsdeveloper/2016/12/02/symlinks-windows-10/) gives more explanation.

To clone using HTTPS:

```shell
$ git clone https://github.com/openhwgroup/core-v-mcu-sdk-examples.git --recurse-submodules
```

Using SSH:

```shell
$ git clone git@github.com:openhwgroup/core-v-mcu-sdk-examples.git --recurse-submodules
```

If you have downloaded the repo without using the `--recurse-submodules` argument, you need to run:

```shell
$ git submodule update --init --recursive
```

## Importing and running a demo

All the demos within this repository are aimed to run on the [CORE-V-SDK IDE](https://github.com/openhwgroup/core-v-sdk). Onces you have installed the SDK, you will need to import the common dependency folders, `commio` and `FreeRTOS-Kernel` to the workspace. After that you can import your aimed sample demo from the sample folder and hit build after selectin the `main.c` file. Onces done it should look like the below image.

![image](demo_import.png)

You may add more demos within the same workspace or create a new workspace for other demos.

> Note: Always remember to import the common dependency project folder to the currently working workspace.

## Formatting

The file can be formatted by just running the below command at the root of the repository.

```shell
$ nix fmt
```

You may also use any other formmatter that support clang format with the given `.clang-format` file at the root of the repository.
