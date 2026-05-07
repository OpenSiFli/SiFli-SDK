# This script should be sourced, not executed.

# shellcheck disable=SC2128,SC2169,SC2039,SC3054 # ignore array expansion warning
if [ -n "${BASH_SOURCE-}" ] && [ "${BASH_SOURCE[0]}" = "${0}" ]
then
    echo "This script should be sourced, not executed:"
    # shellcheck disable=SC2039,SC3054  # reachable only with bash
    echo ". ${BASH_SOURCE[0]}"
    exit 1
fi

sdk_path="."
shell_type="sh"

# shellcheck disable=SC2128,SC2169,SC2039,SC3054,SC3028
if [ -n "${BASH_SOURCE-}" ]
then
    # shellcheck disable=SC3028,SC3054
    sdk_path=$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd -P)
    shell_type="bash"
elif [ -n "${ZSH_VERSION-}" ]
then
    sdk_path=$(cd "$(dirname "${(%):-%x}")"; pwd -P)
    shell_type="zsh"
fi

export SIFLI_SDK_PATH="${sdk_path}"

show_help() {
    cat <<'EOF'
usage: . ./export.sh [--profile PROFILE] [-t TOOLCHAIN]

Activate the installed SiFli-SDK environment in the current shell.

options:
  --profile PROFILE      profile to export, defaults to "default"
  -t, --toolchain TOOLCHAIN
                         toolchain to export: gcc (default); Keil is Windows-only
  -h, --help             show this help message and exit
EOF
}

profile="default"
prev=""
for arg in "$@"
do
    if [ "${prev}" = "--profile" ]
    then
        profile="${arg}"
        prev=""
        continue
    fi
    if [ "${prev}" = "--toolchain" ]
    then
        prev=""
        continue
    fi
    case "${arg}" in
        -h|--help)
            show_help
            unset sdk_path
            return 0
            ;;
        --profile)
            prev="--profile"
            ;;
        --profile=*)
            profile="${arg#--profile=}"
            ;;
        -t|--toolchain)
            prev="--toolchain"
            ;;
        --toolchain=*)
            ;;
    esac
done

if [ "${prev}" = "--profile" ]
then
    echo "ERROR: --profile requires a value." >&2
    unset sdk_path
    return 1
fi
if [ "${prev}" = "--toolchain" ]
then
    echo "ERROR: --toolchain requires a value." >&2
    unset sdk_path
    return 1
fi

mirror_china_normalized=$(printf '%s' "${SIFLI_SDK_MIRROR_CHINA:-}" | tr '[:upper:]' '[:lower:]')
case "${mirror_china_normalized}" in
    1|true|yes|on)
        export SIFLI_SDK_GITHUB_ASSETS="https://downloads.sifli.com/github_assets"
        export SIFLI_SDK_PYPI_DEFAULT_INDEX="https://mirrors.ustc.edu.cn/pypi/simple"
        export UV_PYTHON_DOWNLOADS_JSON_URL="https://uv.agentsmirror.com/metadata/python-downloads.json"
        export UV_PYPY_INSTALL_MIRROR="https://uv.agentsmirror.com/pypy"
        ;;
esac

if ! command -v uv >/dev/null 2>&1
then
    echo "ERROR: uv was not found in PATH. Please install uv before running export.sh." >&2
    unset sdk_path
    return 1
fi

export_output=$(uv run --with rich --with tomli_w --python 3.13.11 --no-project "${sdk_path}/tools/sdk_env.py" export --shell "${shell_type}" "$@")
export_status=$?
if [ ${export_status} -ne 0 ]
then
    unset sdk_path
    return ${export_status}
fi

script_path=$(printf '%s\n' "${export_output}" | tail -n 1)
if [ ! -f "${script_path}" ]
then
    echo "ERROR: export helper did not return a valid script path." >&2
    unset sdk_path
    return 1
fi

. "${script_path}"
unset sdk_path
