#!/bin/bash
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
# Mac/Linux version of init_commit_env.bat

GIT_DIR=$(git rev-parse --git-dir 2>/dev/null)
if [ -z "$GIT_DIR" ]; then
    echo "Error: not in a git repository"
    exit 1
fi

set -e

SRC_DIR="$(dirname "$0")/tools/autotest/commitEnvScripts"

echo
echo "    Initial Commit Env"
echo
echo "  1  Gitlab Repo URL"
echo "  2  Gerrit Repo URL"
echo "  3  Submodule Gerrit Repo URL"
echo "  0  Cancel"
echo

read -r -p "select an option [2]: " choice
choice=${choice:-2}

case "$choice" in
    1)
        cp "$SRC_DIR/pre-commit" "$GIT_DIR/hooks/"
        chmod +x "$GIT_DIR/hooks/pre-commit"
        echo "gitlab hooks installed"
        ;;
    2|3)
        cp "$SRC_DIR/commit-msg" "$GIT_DIR/hooks/"
        chmod +x "$GIT_DIR/hooks/commit-msg"
        cp "$SRC_DIR/pre-commit" "$GIT_DIR/hooks/"
        chmod +x "$GIT_DIR/hooks/pre-commit"
        echo "gerrit hooks installed"
        ;;
    *)
        echo "cancelled"
        exit 0
        ;;
esac

git config commit.template .commit_template
echo "commit.template set to .commit_template"
