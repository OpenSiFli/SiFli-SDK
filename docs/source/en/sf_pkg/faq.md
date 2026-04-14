# Frequently Asked Questions

## What should I do if the upload fails?

1. Check if the network connection is working properly
2. Verify that the token is valid
3. Clear the local cache and then rebuild and upload again

## What should I do if dependency installation fails?

1. Check if the package name, version number, and username format in `conanfile.py` are correct
2. Use `sdk.py sf-pkg search <package_name>` to confirm the package exists
3. Ensure you have executed `.\export.ps1` (or `./export.sh`) to initialize the environment

## How do I update installed dependencies?

1. Modify the version number in `conanfile.py`
2. Re-execute the `sdk.py sf-pkg install` command
