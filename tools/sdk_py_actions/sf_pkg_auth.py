# -*- coding:utf-8 -*-
# SPDX-FileCopyrightText: 2026 SiFli
# SPDX-License-Identifier: Apache-2.0

import base64
import json
import os
import platform
import secrets
import socket
import tempfile
import uuid
from datetime import datetime
from datetime import timezone
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

from cryptography.fernet import Fernet
from cryptography.hazmat.primitives.kdf.scrypt import Scrypt

from sdk_py_actions.tools import print_warning


class SfPkgAuthError(Exception):
    pass


sdk_tools_path = os.environ.get('SIFLI_SDK_TOOLS_PATH') or os.path.expanduser(os.path.join('~', '.sifli'))
CREDENTIALS_FILE = os.path.join(sdk_tools_path, '.sf-pkg')
MASTER_KEY_FILE = os.path.join(sdk_tools_path, '.sf-pkg.key')

STORE_VERSION = 2
KEY_VERSION = 1


def normalize_user(user: str) -> str:
    return user.strip().lower()


def _now_utc_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace('+00:00', 'Z')


def _safe_chmod(path: str, mode: int) -> None:
    try:
        os.chmod(path, mode)
    except OSError:
        # chmod may be unsupported on some systems, do not fail authentication flow.
        pass


def _ensure_storage_dir() -> None:
    os.makedirs(sdk_tools_path, exist_ok=True)
    _safe_chmod(sdk_tools_path, 0o700)


def _atomic_write_text(path: str, content: str) -> None:
    _ensure_storage_dir()
    fd, temp_path = tempfile.mkstemp(prefix='.sf_pkg_tmp_', dir=sdk_tools_path, text=True)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            f.write(content)
        _safe_chmod(temp_path, 0o600)
        os.replace(temp_path, path)
        _safe_chmod(path, 0o600)
    finally:
        if os.path.exists(temp_path):
            os.remove(temp_path)


def _backup_file(path: str, reason: str) -> Optional[str]:
    if not os.path.exists(path):
        return None

    timestamp = datetime.now(timezone.utc).strftime('%Y%m%d%H%M%S')
    backup_path = f'{path}.bak.{reason}.{timestamp}'
    index = 1
    while os.path.exists(backup_path):
        index += 1
        backup_path = f'{path}.bak.{reason}.{timestamp}.{index}'

    os.replace(path, backup_path)
    return backup_path


def _backup_store(reason: str) -> List[str]:
    backups: List[str] = []
    for path in (CREDENTIALS_FILE, MASTER_KEY_FILE):
        backup_path = _backup_file(path, reason)
        if backup_path:
            backups.append(backup_path)
    return backups


def _machine_identifier() -> bytes:
    candidates = []

    try:
        uname = os.uname()
        candidates.extend([uname.nodename, uname.machine, uname.sysname])
    except AttributeError:
        pass

    env_host = os.environ.get('COMPUTERNAME') or os.environ.get('HOSTNAME')
    if env_host:
        candidates.append(env_host)

    node = platform.node()
    if node:
        candidates.append(node)

    try:
        hostname = socket.gethostname()
        if hostname:
            candidates.append(hostname)
    except OSError:
        pass

    mac = uuid.getnode()
    if mac:
        candidates.append(hex(mac))

    unique_parts = []
    seen = set()
    for part in candidates:
        part = str(part).strip()
        if part and part not in seen:
            seen.add(part)
            unique_parts.append(part)

    identifier = '::'.join(unique_parts) if unique_parts else 'sifli-default'
    return identifier.encode('utf-8')


def _derive_wrapping_key(salt: bytes) -> bytes:
    kdf = Scrypt(
        salt=salt,
        length=32,
        n=2**14,
        r=8,
        p=1,
    )
    key = kdf.derive(_machine_identifier())
    return base64.urlsafe_b64encode(key)


def _create_master_key() -> bytes:
    master_key = Fernet.generate_key()
    salt = secrets.token_bytes(16)
    wrapping_key = _derive_wrapping_key(salt)
    wrapped_master = Fernet(wrapping_key).encrypt(master_key).decode('utf-8')

    key_bundle = {
        'version': KEY_VERSION,
        'kdf': 'scrypt',
        'salt': base64.b64encode(salt).decode('utf-8'),
        'wrapped_key': wrapped_master,
    }
    _atomic_write_text(MASTER_KEY_FILE, json.dumps(key_bundle, separators=(',', ':')))
    return master_key


def _load_master_key() -> Optional[bytes]:
    if not os.path.exists(MASTER_KEY_FILE):
        return None

    with open(MASTER_KEY_FILE, 'r', encoding='utf-8') as f:
        key_bundle = json.load(f)

    if key_bundle.get('version') != KEY_VERSION:
        raise SfPkgAuthError(f'Unsupported sf-pkg key version: {key_bundle.get("version")}')

    if key_bundle.get('kdf') != 'scrypt':
        raise SfPkgAuthError(f'Unsupported sf-pkg key derivation: {key_bundle.get("kdf")}')

    wrapped_key = key_bundle.get('wrapped_key')
    if not isinstance(wrapped_key, str) or not wrapped_key:
        raise SfPkgAuthError('Invalid wrapped sf-pkg key payload')

    encoded_salt = key_bundle.get('salt')
    if not isinstance(encoded_salt, str) or not encoded_salt:
        raise SfPkgAuthError('Invalid sf-pkg key salt')

    salt = base64.b64decode(encoded_salt.encode('utf-8'))
    wrapping_key = _derive_wrapping_key(salt)
    master_key = Fernet(wrapping_key).decrypt(wrapped_key.encode('utf-8'))

    # Validate key format.
    Fernet(master_key)
    return master_key


def _warn_relogin(reason: str, backups: List[str]) -> None:
    backup_msg = f' Backups: {", ".join(backups)}.' if backups else ''
    print_warning(
        f'WARNING: {reason}. Local sf-pkg credentials were reset.{backup_msg} '
        'Please login again using "sdk.py sf-pkg login".'
    )


def _default_store() -> Dict[str, Any]:
    return {
        'version': STORE_VERSION,
        'active_user': None,
        'users': {},
    }


def _normalize_store(store: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(store, dict):
        raise SfPkgAuthError('sf-pkg credential store is not a JSON object')

    raw_users = store.get('users', {})
    if not isinstance(raw_users, dict):
        raise SfPkgAuthError('sf-pkg credential store "users" must be an object')

    users: Dict[str, Dict[str, str]] = {}
    now = _now_utc_iso()

    for raw_user, raw_record in raw_users.items():
        if not isinstance(raw_user, str):
            continue

        user = normalize_user(raw_user)
        if not user:
            continue

        token = ''
        created_at = now
        updated_at = now

        if isinstance(raw_record, str):
            token = raw_record.strip()
        elif isinstance(raw_record, dict):
            raw_token = raw_record.get('token')
            if isinstance(raw_token, str):
                token = raw_token.strip()
            if isinstance(raw_record.get('created_at'), str) and raw_record.get('created_at'):
                created_at = raw_record['created_at']
            if isinstance(raw_record.get('updated_at'), str) and raw_record.get('updated_at'):
                updated_at = raw_record['updated_at']

        if not token:
            continue

        users[user] = {
            'token': token,
            'created_at': created_at,
            'updated_at': updated_at,
        }

    raw_active = store.get('active_user')
    active_user = normalize_user(raw_active) if isinstance(raw_active, str) else None
    if active_user and active_user not in users:
        active_user = None

    if not active_user and users:
        active_user = sorted(users.keys())[0]

    return {
        'version': STORE_VERSION,
        'active_user': active_user,
        'users': users,
    }


def load_store() -> Dict[str, Any]:
    has_vault = os.path.exists(CREDENTIALS_FILE)
    has_key = os.path.exists(MASTER_KEY_FILE)

    if not has_vault and not has_key:
        return _default_store()

    if has_vault != has_key:
        backups = _backup_store('incomplete')
        _warn_relogin('Incompatible sf-pkg credential files detected', backups)
        return _default_store()

    try:
        master_key = _load_master_key()
        if not master_key:
            return _default_store()

        with open(CREDENTIALS_FILE, 'r', encoding='utf-8') as f:
            encrypted_content = f.read().strip()

        if not encrypted_content:
            return _default_store()

        decrypted_content = Fernet(master_key).decrypt(encrypted_content.encode('utf-8')).decode('utf-8')
        store = json.loads(decrypted_content)
        return _normalize_store(store)
    except Exception:
        backups = _backup_store('corrupt')
        _warn_relogin('Unable to decrypt sf-pkg credential files', backups)
        return _default_store()


def save_store(store: Dict[str, Any]) -> None:
    normalized_store = _normalize_store(store)
    _ensure_storage_dir()

    master_key: Optional[bytes] = None
    try:
        master_key = _load_master_key()
    except Exception:
        backups = _backup_store('invalid-key')
        _warn_relogin('sf-pkg key file is invalid', backups)

    if master_key is None:
        master_key = _create_master_key()

    encrypted_content = Fernet(master_key).encrypt(
        json.dumps(normalized_store, separators=(',', ':')).encode('utf-8')
    ).decode('utf-8')

    _atomic_write_text(CREDENTIALS_FILE, encrypted_content)


def remove_store() -> None:
    for path in (CREDENTIALS_FILE, MASTER_KEY_FILE):
        if os.path.exists(path):
            os.remove(path)


def list_users() -> List[str]:
    store = load_store()
    return sorted(store['users'].keys())


def get_active_user() -> Optional[str]:
    store = load_store()
    active_user = store.get('active_user')
    return active_user if isinstance(active_user, str) else None


def upsert_user(user: str, token: str) -> str:
    normalized_user = normalize_user(user)
    token = token.strip()

    if not normalized_user:
        raise SfPkgAuthError('User cannot be empty')

    if not token:
        raise SfPkgAuthError('Token cannot be empty')

    store = load_store()
    now = _now_utc_iso()
    old_record = store['users'].get(normalized_user, {})
    created_at = old_record.get('created_at', now)

    store['users'][normalized_user] = {
        'token': token,
        'created_at': created_at,
        'updated_at': now,
    }
    store['active_user'] = normalized_user
    save_store(store)

    return normalized_user


def delete_user(user: str) -> bool:
    normalized_user = normalize_user(user)
    store = load_store()

    if normalized_user not in store['users']:
        return False

    del store['users'][normalized_user]

    if not store['users']:
        remove_store()
        return True

    active_user = store.get('active_user')
    if active_user == normalized_user or active_user not in store['users']:
        store['active_user'] = sorted(store['users'].keys())[0]

    save_store(store)
    return True


def clear_users() -> None:
    remove_store()


def set_active_user(user: str) -> str:
    normalized_user = normalize_user(user)
    store = load_store()

    if normalized_user not in store['users']:
        raise SfPkgAuthError(f'User "{normalized_user}" is not logged in')

    store['active_user'] = normalized_user
    save_store(store)
    return normalized_user


def resolve_credentials(cli_user: Optional[str], required: bool=True) -> Optional[Tuple[str, str]]:
    store = load_store()

    selected_user = normalize_user(cli_user) if cli_user else store.get('active_user')

    if not selected_user:
        if required:
            raise SfPkgAuthError(
                'No sf-pkg user selected. Login first with "sdk.py sf-pkg login -u <user> -t <token>" '
                'or pass group "--user <user>" to "sdk.py sf-pkg ...".'
            )
        return None

    record = store['users'].get(selected_user)
    if not record:
        raise SfPkgAuthError(
            f'User "{selected_user}" is not logged in locally. '
            'Please run "sdk.py sf-pkg login -u <user> -t <token>".'
        )

    token = record.get('token', '').strip()
    if not token:
        raise SfPkgAuthError(f'User "{selected_user}" has empty token, please login again.')

    return selected_user, token
