# This script should be sourced, not executed.

# Emergency backup option to use previous export.sh (export_legacy.sh) if the new export approach fails.
# To use it, set environmental variable like: export SIFLI_SDK_LEGACY_EXPORT=1
if [ -n "${SIFLI_SDK_LEGACY_EXPORT-}" ]; then
    . ./tools/legacy_exports/export_legacy.sh
    return $?
fi

# shellcheck disable=SC2128,SC2169,SC2039,SC3054 # ignore array expansion warning
if [ -n "${BASH_SOURCE-}" ] && [ "${BASH_SOURCE[0]}" = "${0}" ]
then
    echo "This script should be sourced, not executed:"
    # shellcheck disable=SC2039,SC3054  # reachable only with bash
    echo ". ${BASH_SOURCE[0]}"
    exit 1
fi

# Attempt to identify the SIFLI-SDK directory
sdk_path="."

shell_type="detect"

# shellcheck disable=SC2128,SC2169,SC2039,SC3054,SC3028 # ignore array expansion warning
if [ -n "${BASH_SOURCE-}" ]
then
    # shellcheck disable=SC3028,SC3054 # unreachable with 'dash'
    sdk_path=$(dirname "${BASH_SOURCE[0]}")
    shell_type="bash"
elif [ -n "${ZSH_VERSION-}" ]
then
    # shellcheck disable=SC2296  # ignore parameter starts with '{' because it's zsh
    sdk_path=$(dirname "${(%):-%x}")
    shell_type="zsh"
elif [ -n "${SIFLI_SDK_PATH-}" ]
then
    if [ -f "/.dockerenv" ]
    then
        echo "Using the SIFLI_SDK_PATH found in the environment as docker environment detected."
        sdk_path=$SIFLI_SDK_PATH
    elif [ -n "${SIFLI_SDK_PATH_FORCE-}" ]
    then
        echo "Using the forced SIFLI_SDK_PATH found in the environment."
        sdk_path=$SIFLI_SDK_PATH
    fi
fi

if [ ! -f "${sdk_path}/tools/sifli_sdk_tools.py" ] ||
   [ ! -f "${sdk_path}/tools/activate.py" ]
then
    echo "Could not automatically detect SIFLI_SDK_PATH from script location. Please navigate to your SIFLI-SDK directory and run:"
    echo ". ./export.sh"
    if [ -n "${SIFLI_SDK_PATH-}" ]
    then
        echo
        echo "To use the SIFLI_SDK_PATH set in the environment, you can enforce it by setting 'export SIFLI_SDK_PATH_FORCE=1'"
    fi
    unset sdk_path
    return 1
fi

. "${sdk_path}/tools/detect_python.sh"

# Evaluate the SIFLI-SDK environment set up by the activate.py script.
sdk_exports=$("$SIFLI_PYTHON" "${sdk_path}/tools/activate.py" --export --shell $shell_type)
eval "${sdk_exports}"
unset sdk_path
return 0

# SDK 2.x

