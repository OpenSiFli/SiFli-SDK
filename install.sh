#!/usr/bin/env bash

set -e
# set -u

basedir=$(dirname "$0")
SIFLI_SDK_PATH=$(cd "${basedir}"; pwd -P)
export SIFLI_SDK_PATH

echo "INFO: Using SIFLI_SDK_PATH '${SIFLI_SDK_PATH}' for installation."

echo "Detecting the Python interpreter"
. "${SIFLI_SDK_PATH}/tools/detect_python.sh"

echo "Checking Python compatibility"
"${SIFLI_PYTHON}" "${SIFLI_SDK_PATH}/tools/python_version_checker.py"

while getopts ":h" option; do
    case $option in
        h)
            "${SIFLI_PYTHON}" "${SIFLI_SDK_PATH}/tools/install_util.py" print_help sh
            exit;;
        \?)
            ;;
    esac
done

FEATURES=$("${SIFLI_PYTHON}" "${SIFLI_SDK_PATH}/tools/install_util.py" extract features "$@")

echo "Installing Python environment and packages"
"${SIFLI_PYTHON}" "-m" "pip" "install" "requests"
"${SIFLI_PYTHON}" "${SIFLI_SDK_PATH}/tools/sifli_sdk_tools.py" install-python-env --features="${FEATURES}"

python_venv_path=$("${SIFLI_PYTHON}" "${SIFLI_SDK_PATH}/tools/sifli_sdk_tools.py" "get-install-python-env")

if [[ "$python_venv_path" == Error* ]]; then
    echo $python_venv_path
    exit 1
fi

TARGETS=$("${python_venv_path}/bin/python" "${SIFLI_SDK_PATH}/tools/install_util.py" extract targets "$@")

echo "Installing SiFli-SDK tools"
"${python_venv_path}/bin/python" "${SIFLI_SDK_PATH}/tools/sifli_sdk_tools.py" install --targets="${TARGETS}"

echo "All done! You can now run:"
echo ""
echo "  . ${basedir}/export.sh"
echo ""
