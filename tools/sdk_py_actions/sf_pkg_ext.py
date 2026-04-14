# -*- coding:utf-8 -*-
# SPDX-FileCopyrightText: 2025-2026 SiFli
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import json
import re
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

from sdk_py_actions.cli.context import SdkContext
from sdk_py_actions.cli.registry import CommandRegistry
from sdk_py_actions.errors import AuthError
from sdk_py_actions.errors import CommandExecutionError
from sdk_py_actions.errors import UsageError
from sdk_py_actions.sf_pkg_auth import SfPkgAuthError
from sdk_py_actions.sf_pkg_auth import clear_users
from sdk_py_actions.sf_pkg_auth import delete_user
from sdk_py_actions.sf_pkg_auth import get_active_user
from sdk_py_actions.sf_pkg_auth import list_users
from sdk_py_actions.sf_pkg_auth import normalize_user
from sdk_py_actions.sf_pkg_auth import resolve_credentials
from sdk_py_actions.sf_pkg_auth import set_active_user
from sdk_py_actions.sf_pkg_auth import upsert_user
from sdk_py_actions.tools import print_warning

EXTENSION_ID = "sf-pkg"
EXTENSION_VERSION = "2.0.0"
EXTENSION_API_VERSION = 2
MIN_SDK_VERSION = None
SF_PKG_REMOTE_URL = "https://jfrog.sifli.com/artifactory/api/conan/conan-local"
SF_PKG_PUBLIC_REMOTE_NAME = "artifactory"
_REMOTE_NAME_RE = re.compile(r"^[a-z0-9][a-z0-9._-]*$")


def _extract_namespace(package_ref: str) -> Optional[str]:
    if "@" not in package_ref:
        return None

    namespace = package_ref.rsplit("@", 1)[1].strip()
    if not namespace:
        return None

    namespace = namespace.split("/", 1)[0].strip()
    if not namespace:
        return None

    return normalize_user(namespace)


def _group_user(sdk_ctx: SdkContext) -> Optional[str]:
    group_options = sdk_ctx.config.group_options.get("sf-pkg", {})
    group_user = group_options.get("sf_pkg_user")
    if not isinstance(group_user, str):
        return None

    group_user = group_user.strip()
    return group_user if group_user else None


def _to_auth_error(exc: Exception) -> AuthError:
    return AuthError(str(exc))


def _resolve_login_user(sdk_ctx: SdkContext, action_user: Optional[str]) -> str:
    global_user = _group_user(sdk_ctx)

    if action_user and global_user and normalize_user(action_user) != normalize_user(global_user):
        raise UsageError(
            'Conflicting users provided. Please use either "sf-pkg --user ... login" '
            'or "sf-pkg login --user ...", or keep them identical.'
        )

    selected_user = action_user or global_user
    if not selected_user:
        raise UsageError(
            'Missing user for login. Use "sdk.py sf-pkg --user <user> login -t <token>" '
            'or "sdk.py sf-pkg login -u <user> -t <token>".'
        )

    return normalize_user(selected_user)


def _remote_name_for_user(user: str) -> str:
    remote_name = normalize_user(user)
    if not _REMOTE_NAME_RE.fullmatch(remote_name):
        raise UsageError(
            f'User "{user}" cannot be mapped to a Conan remote name. '
            "Use lowercase letters/digits and separators ._- only."
        )
    return remote_name


def _normalize_remote_url(url: str) -> str:
    return url.strip().rstrip("/")


def _load_remote_list_data(sdk_ctx: SdkContext) -> List[Dict[str, Any]]:
    try:
        result = sdk_ctx.runner.run(
            ["conan", "remote", "list", "-f", "json"],
            cwd=sdk_ctx.project_dir,
            capture_output=True,
        )
    except CommandExecutionError as exc:
        raise CommandExecutionError("Failed to inspect Conan remotes.") from exc
    output = result.stdout or "[]"
    try:
        data = json.loads(output)
    except json.JSONDecodeError as exc:
        raise CommandExecutionError(f"Failed to parse conan remote list output: {exc}") from exc

    if not isinstance(data, list):
        raise CommandExecutionError("Unexpected conan remote list payload")

    return [item for item in data if isinstance(item, dict)]


def _load_remote_list(sdk_ctx: SdkContext) -> List[Dict[str, Any]]:
    try:
        return _load_remote_list_data(sdk_ctx)
    except CommandExecutionError as exc:
        raise AuthError("Failed to inspect Conan remotes. Please login again.") from exc


def _load_remote_users(sdk_ctx: SdkContext) -> List[Dict[str, Any]]:
    try:
        result = sdk_ctx.runner.run(
            ["conan", "remote", "list-users", "-f", "json"],
            cwd=sdk_ctx.project_dir,
            capture_output=True,
        )
    except CommandExecutionError as exc:
        raise AuthError("Failed to inspect Conan remote authentication status. Please login again.") from exc
    output = result.stdout or "[]"
    try:
        data = json.loads(output)
    except json.JSONDecodeError as exc:
        raise AuthError(f"Failed to parse conan remote list-users output: {exc}") from exc

    if not isinstance(data, list):
        raise AuthError("Unexpected conan remote list-users payload")

    return [item for item in data if isinstance(item, dict)]


def _clear_stale_user_and_raise(user: str, reason: str) -> None:
    try:
        delete_user(user)
    except SfPkgAuthError:
        pass

    raise AuthError(
        f"{reason} Local credentials for user '{user}' were removed. "
        'Please login again using "sdk.py sf-pkg login".'
    )


def _ensure_remote_matches_user(sdk_ctx: SdkContext, user: str) -> str:
    remote_name = _remote_name_for_user(user)
    expected_url = _normalize_remote_url(SF_PKG_REMOTE_URL)

    remotes = _load_remote_list(sdk_ctx)
    remote_info = next((item for item in remotes if item.get("name") == remote_name), None)
    if remote_info is None:
        _clear_stale_user_and_raise(
            user,
            f'Conan remote "{remote_name}" is missing for user "{user}".',
        )

    remote_url = str(remote_info.get("url") or "")
    if _normalize_remote_url(remote_url) != expected_url:
        _clear_stale_user_and_raise(
            user,
            f'Conan remote "{remote_name}" URL does not match SiFli package registry.',
        )

    remote_users = _load_remote_users(sdk_ctx)
    remote_user_info = next((item for item in remote_users if item.get("name") == remote_name), None)
    if remote_user_info is None:
        _clear_stale_user_and_raise(
            user,
            f'Conan remote "{remote_name}" has no authentication record.',
        )

    authenticated = bool(remote_user_info.get("authenticated"))
    username = normalize_user(str(remote_user_info.get("user_name") or ""))
    if not authenticated or username != user:
        _clear_stale_user_and_raise(
            user,
            f'Conan remote "{remote_name}" is not authenticated as user "{user}".',
        )

    return remote_name


def _configure_and_login_remote(sdk_ctx: SdkContext, user: str, token: str) -> str:
    remote_name = _remote_name_for_user(user)

    try:
        sdk_ctx.runner.run(
            ["conan", "remote", "add", remote_name, SF_PKG_REMOTE_URL, "--force"],
            cwd=sdk_ctx.project_dir,
        )
        sdk_ctx.runner.run(
            ["conan", "remote", "login", "-p", token, remote_name, user],
            cwd=sdk_ctx.project_dir,
        )
    except CommandExecutionError as exc:
        raise AuthError(f"Failed to login remote '{remote_name}' as {user}") from exc

    return remote_name


def _cleanup_user_remote(sdk_ctx: SdkContext, user: str) -> None:
    try:
        remote_name = _remote_name_for_user(user)
    except UsageError as exc:
        print_warning(f"WARNING: skip remote cleanup for user '{user}': {exc}")
        return

    sdk_ctx.runner.run(["conan", "remote", "logout", remote_name], cwd=sdk_ctx.project_dir, check=False)
    sdk_ctx.runner.run(["conan", "remote", "remove", remote_name], cwd=sdk_ctx.project_dir, check=False)


def _resolve_auth_credentials(sdk_ctx: SdkContext, required: bool) -> Optional[Tuple[str, str]]:
    try:
        return resolve_credentials(_group_user(sdk_ctx), required=required)
    except SfPkgAuthError as exc:
        raise _to_auth_error(exc) from exc


def _ensure_remote_access(sdk_ctx: SdkContext, required: bool) -> Optional[Tuple[str, str, str]]:
    credentials = _resolve_auth_credentials(sdk_ctx, required=required)
    if credentials is None:
        return None

    user, token = credentials
    remote_name = _ensure_remote_matches_user(sdk_ctx, user)
    return user, token, remote_name


def _ensure_public_remote(sdk_ctx: SdkContext) -> str:
    expected_url = _normalize_remote_url(SF_PKG_REMOTE_URL)
    remotes = _load_remote_list_data(sdk_ctx)
    remote_info = next((item for item in remotes if item.get("name") == SF_PKG_PUBLIC_REMOTE_NAME), None)

    remote_url = str(remote_info.get("url") or "") if remote_info else ""
    if _normalize_remote_url(remote_url) != expected_url:
        sdk_ctx.runner.run(
            ["conan", "remote", "add", SF_PKG_PUBLIC_REMOTE_NAME, SF_PKG_REMOTE_URL, "--force"],
            cwd=sdk_ctx.project_dir,
        )

    return SF_PKG_PUBLIC_REMOTE_NAME


def init_callback(sdk_ctx: SdkContext) -> None:
    result = sdk_ctx.runner.run(["conan", "new", "sf-pkg-project"], cwd=sdk_ctx.project_dir, check=False)
    if result.returncode != 0:
        raise CommandExecutionError("Failed to create dependency file")
    print("You can now add dependent packages in conanfile.py")


def install_callback(sdk_ctx: SdkContext) -> None:
    remote_name = _ensure_public_remote(sdk_ctx)
    sdk_ctx.runner.run(
        [
            "conan",
            "install",
            ".",
            "--output-folder=sf-pkgs",
            "--deployer=full_deploy",
            "--envs-generation=false",
            f"-r={remote_name}",
        ],
        cwd=sdk_ctx.project_dir,
    )
    print("Packages installed successfully")


def new_callback(
    sdk_ctx: SdkContext,
    name: str = "mypackage",
    version: Optional[str] = None,
    license: Optional[str] = None,
    author: Optional[str] = None,
    support_sdk_version: Optional[str] = None,
) -> None:
    credentials = _resolve_auth_credentials(sdk_ctx, required=True)
    if not credentials:
        raise AuthError("No available user credentials. Please login first.")

    user, _token = credentials
    cmd = [
        "conan",
        "new",
        "sf-pkg-package",
        "-d",
        f"name={name}",
        "-d",
        f"user={user}",
    ]

    if version:
        cmd.extend(["-d", f"version={version}"])
    if license:
        cmd.extend(["-d", f"license={license}"])
    if author:
        cmd.extend(["-d", f"author={author}"])
    if support_sdk_version:
        cmd.extend(["-d", f"support_sdk_version={support_sdk_version}"])

    sdk_ctx.runner.run(cmd, cwd=sdk_ctx.project_dir)
    print(f"Created new package: {name}")


def build_callback(sdk_ctx: SdkContext, version: str) -> None:
    credentials = _ensure_remote_access(sdk_ctx, required=True)
    if not credentials:
        raise AuthError("No available user credentials. Please login first.")

    _user, _token, remote_name = credentials
    sdk_ctx.runner.run(["conan", "create", "--version", version, ".", "-r", remote_name], cwd=sdk_ctx.project_dir)
    print(f"Package built successfully with version: {version}")


def search_callback(sdk_ctx: SdkContext, name: str) -> None:
    remote_name = _ensure_public_remote(sdk_ctx)
    search_pattern = name if "*" in name else f"{name}/*"
    sdk_ctx.runner.run(["conan", "search", search_pattern, f"-r={remote_name}"], cwd=sdk_ctx.project_dir)
    print(f"Search completed for: {search_pattern}")


def upload_callback(sdk_ctx: SdkContext, name: str, keep: bool = False) -> None:
    credentials = _ensure_remote_access(sdk_ctx, required=True)
    if not credentials:
        raise AuthError("No available user credentials. Please login first.")

    user, token, remote_name = credentials
    package_user = _extract_namespace(name)
    if package_user and package_user != user:
        raise UsageError(
            f'Package namespace "{package_user}" does not match selected user "{user}". '
            "Use matching --user or update --name."
        )

    sdk_ctx.runner.run(["conan", "upload", name, f"-r={remote_name}"], cwd=sdk_ctx.project_dir)
    print(f"Package {name} uploaded successfully")

    if not keep:
        sdk_ctx.runner.run(["conan", "remove", name, "-c"], cwd=sdk_ctx.project_dir, check=False)
        print(f"Package {name} removed from local cache")

    try:
        import requests

        sync_url = "https://packages.sifli.com/api/v1/sync"
        headers = {
            "Authorization": f"Bearer {token}",
            "Content-Type": "application/json",
        }

        print("Syncing package to public repository...")
        response = requests.post(sync_url, json={}, headers=headers, timeout=30)

        if response.status_code == 200:
            print("Package synced to public repository successfully")
        else:
            print_warning(f"WARNING: Sync failed with status code {response.status_code}")
            print_warning(f"WARNING: Response: {response.text}")

    except ImportError:
        print_warning("WARNING: requests library not available. Skipping sync to public repository.")
    except Exception as exc:
        print_warning(f"WARNING: Failed to sync to public repository: {exc}")


def remove_callback(sdk_ctx: SdkContext, name: str, remote: bool = False) -> None:
    if remote:
        credentials = _ensure_remote_access(sdk_ctx, required=True)
        if not credentials:
            raise AuthError("No available user credentials. Please login first.")

        _user, _token, remote_name = credentials
        sdk_ctx.runner.run(["conan", "remove", name, f"-r={remote_name}", "-c"], cwd=sdk_ctx.project_dir)
        print(f"Package {name} removed from remote repository: {remote_name}")
        return

    sdk_ctx.runner.run(["conan", "remove", name, "-c"], cwd=sdk_ctx.project_dir)
    print(f"Package {name} removed from local cache")


def login_callback(sdk_ctx: SdkContext, token: str, user: Optional[str] = None) -> None:
    selected_user = _resolve_login_user(sdk_ctx, user)
    remote_name = _configure_and_login_remote(sdk_ctx, selected_user, token)

    try:
        stored_user = upsert_user(selected_user, token)
    except SfPkgAuthError as exc:
        raise _to_auth_error(exc) from exc

    print("Logged in to SiFli package registry")
    print(f"Credentials stored for user: {stored_user}")
    print(f"Active user set to: {stored_user}")
    print(f"Conan remote ready: {remote_name}")


def logout_callback(sdk_ctx: SdkContext, name: Optional[str] = None) -> None:
    target_user = name or _group_user(sdk_ctx)

    if target_user:
        selected_user = normalize_user(target_user)
        try:
            removed = delete_user(selected_user)
        except SfPkgAuthError as exc:
            raise _to_auth_error(exc) from exc

        if removed:
            _cleanup_user_remote(sdk_ctx, selected_user)
            print(f"Logged out user: {selected_user}")
            current = get_active_user()
            if current:
                print(f"Active user switched to: {current}")
            return

        print(f"User {selected_user} not found in stored credentials")
        return

    users = list_users()
    for selected_user in users:
        _cleanup_user_remote(sdk_ctx, selected_user)
    clear_users()
    print("Logged out all users and cleared credentials")


def users_callback(sdk_ctx: SdkContext) -> None:
    users = list_users()
    active = get_active_user()
    if not users:
        print('No stored sf-pkg users. Please login using "sdk.py sf-pkg login".')
        return

    print("Stored sf-pkg users:")
    for user in users:
        marker = "*" if user == active else " "
        print(f" {marker} {user}")
    if active:
        print(f"Active user: {active}")


def use_callback(sdk_ctx: SdkContext, name: str) -> None:
    try:
        selected_user = set_active_user(name)
    except SfPkgAuthError as exc:
        raise _to_auth_error(exc) from exc

    print(f"Active sf-pkg user set to: {selected_user}")


def current_user_callback(sdk_ctx: SdkContext) -> None:
    active = get_active_user()
    if not active:
        print('No active sf-pkg user. Use "sdk.py sf-pkg login" or "sdk.py sf-pkg use" first.')
        return

    print(active)


def register(registry: CommandRegistry) -> None:
    registry.group(
        path="sf-pkg",
        help="SiFli package management.",
        options=[
            {
                "names": ["--user", "-u", "sf_pkg_user"],
                "help": "Select sf-pkg user for this group.",
                "default": None,
            }
        ],
    )

    registry.command(
        path="sf-pkg/login",
        callback=login_callback,
        help="Login to SiFli package registry and store credentials.",
        options=[
            {
                "names": ["--user", "-u"],
                "help": "Username for login. If omitted, group --user is used.",
                "default": None,
            },
            {
                "names": ["--token", "-t"],
                "help": "API token for login.",
                "required": True,
            },
        ],
    )

    registry.command(
        path="sf-pkg/logout",
        callback=logout_callback,
        help="Logout from SiFli package registry and clear credentials.",
        options=[
            {
                "names": ["--name", "-n"],
                "help": "Username to logout. If omitted, uses group --user or logs out all users.",
                "default": None,
            }
        ],
    )

    registry.command(path="sf-pkg/users", callback=users_callback, help="List local sf-pkg users and active user.")

    registry.command(
        path="sf-pkg/use",
        callback=use_callback,
        help="Switch active sf-pkg user.",
        options=[
            {
                "names": ["--name", "-n"],
                "help": "Username to set as active user.",
                "required": True,
            }
        ],
    )

    registry.command(
        path="sf-pkg/current-user",
        callback=current_user_callback,
        help="Show current active sf-pkg user.",
    )

    registry.command(path="sf-pkg/init", callback=init_callback, help="Initialize project dependencies.")
    registry.command(path="sf-pkg/install", callback=install_callback, help="Install SiFli-SDK packages.")

    registry.command(
        path="sf-pkg/new",
        callback=new_callback,
        help="Create a new SiFli-SDK package.",
        options=[
            {"names": ["--name", "-n"], "help": "Package name.", "default": "mypackage", "required": True},
            {"names": ["--version"], "help": "Package version.", "default": None},
            {"names": ["--license"], "help": "Package license.", "default": None},
            {"names": ["--author"], "help": "Package author.", "default": None},
            {
                "names": ["--support-sdk-version"],
                "help": "Supported SiFli-SDK version.",
                "default": None,
            },
        ],
    )

    registry.command(
        path="sf-pkg/build",
        callback=build_callback,
        help="Build the package for upload.",
        options=[
            {
                "names": ["--version", "-v"],
                "help": "Version to be built.",
                "required": True,
            }
        ],
    )

    registry.command(
        path="sf-pkg/upload",
        callback=upload_callback,
        help="Upload the specified package.",
        options=[
            {
                "names": ["--name", "-n"],
                "help": "Name of package to be uploaded.",
                "required": True,
            },
            {
                "names": ["--keep"],
                "help": "Keep local cache after upload.",
                "is_flag": True,
                "default": False,
            },
        ],
    )

    registry.command(
        path="sf-pkg/remove",
        callback=remove_callback,
        help="Remove package from local cache or from the selected user remote.",
        options=[
            {
                "names": ["--name"],
                "help": "Name of package to be removed.",
                "required": True,
            },
            {
                "names": ["--remote"],
                "help": "Remove package from selected user remote instead of local cache.",
                "is_flag": True,
                "default": False,
            },
        ],
    )

    registry.command(
        path="sf-pkg/search",
        callback=search_callback,
        help="Search for packages in SiFli package registry.",
        arguments=[
            {
                "names": ["name"],
                "required": True,
            }
        ],
    )
