#!/usr/bin/env python3

# Copyright (c) 2018 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

import argparse
import sys
import json
import yaml
import fnmatch


def normalize_rule_paths(rule):
    paths = rule.get("path")
    if not paths:
        return []
    if isinstance(paths, list):
        return paths
    return [paths]


def analyze_file(config_file, scancode_file, scanned_files_dir):

    with open(config_file, 'r') as f:
        config = yaml.safe_load(f.read())

    report = ""

    exclude = config.get("exclude")
    if exclude:
        never_check_ext =  exclude.get("extensions", [])
        never_check_langs = exclude.get("langs", [])
        never_check_file = exclude.get("special_file", [])
    else:
        never_check_ext = []
        never_check_langs = []
        never_check_file = []

    copyrights = config.get("copyright", {})
    check_copyright = copyrights.get("check", False)
    lic_config = config.get("license")
    lic_main = lic_config.get("main")
    # lic_cat = lic_config.get("category")
    report_invalid_license = lic_config.get("report_invalid", False)
    report_unknown_license = lic_config.get("report_unknown", False)
    report_missing_license = lic_config.get("report_missing", False)
    extensions_config = config.get("extensions", [])
    extension_rules = []
    for rule in extensions_config:
        for path in normalize_rule_paths(rule):
            extension_rules.append((path, rule))
    # more_cat = []
    # more_cat.append(lic_cat)
    more_lic = lic_config.get('additional', [])
    more_lic.append(lic_main)
    # Scancode may report 'unknown-license-reference' if there are lines
    # containing the word 'license' in the source files, so ignore these for
    # now.
    # more_cat.append('Unstated License')
    if not report_unknown_license:
        more_lic.append('unknown-license-reference')

    if check_copyright:
        print("Will check for missing copyrights...")

    check_langs = []
    with open(scancode_file, 'r') as json_fp:
        scancode_results = json.load(json_fp)
        for file in scancode_results['files']:
            if file['type'] == 'directory':
                continue

            orig_path = str(file['path']).replace(scanned_files_dir, '')
            licenses = file['license_detections']
            detected_license_spdx = file['detected_license_expression_spdx']
            detected_expr = detected_license_spdx or ""
            #licenses.append(file['detected_license_expression'])
            file_type = file.get("file_type")
            kconfig = "Kconfig" in orig_path and file_type in ['ASCII text']
            check = False
            if kconfig:
                check = True
            elif file.get("programming_language") in check_langs:
                check = True
            elif file.get("is_script"):
                check = True
            elif file.get("is_source"):
                check = True

            if check:
                path_to_check = orig_path.replace('\\', '/')
                if file.get("extension")[1:] in never_check_ext or \
                    file.get("programming_language") in never_check_langs or \
                    file.get("name") in never_check_file or \
                    any(fnmatch.fnmatch(path_to_check, pattern) for pattern in never_check_file):
                    check = False


            if check:
                matched_rules = [(path, rule) for path, rule in extension_rules if path in orig_path]
                matched_rule = {}
                if matched_rules:
                    # Use the longest match to avoid less-specific parent paths winning.
                    matched_path, matched_rule = max(matched_rules, key=lambda item: len(item[0]))

                ignore_license = matched_rule.get("ignore_license", False)
                ignore_copyright = matched_rule.get("ignore_copyright", False)
                matched_license = matched_rule.get("license")

                if not ignore_license:
                    if not licenses and report_missing_license:
                        report += ("* {} missing license.\n".format(orig_path))
                    else:
                        base_licenses = more_lic if isinstance(more_lic, list) else [more_lic]
                        if matched_license:
                            matched_licenses = matched_license if isinstance(matched_license, list) else [matched_license]
                            # Path-specific rules extend allowed licenses instead of replacing global defaults.
                            allowed_licenses = list(dict.fromkeys(base_licenses + matched_licenses))
                        else:
                            allowed_licenses = base_licenses
                        if not any(allowed and allowed in detected_expr for allowed in allowed_licenses) and report_invalid_license:
                            report += ("* {} has invalid license: {}\n".format(orig_path, detected_expr))
                        # for lic in licenses:
                        #     if lic['license_expression_spdx'] not in more_lic and report_invalid_license:
                        #         report += ("* {} has invalid license: {}\n".format(orig_path, lic['license_expression_spdx']))
                        #     if lic['license_expression_spdx'] == 'unknown-spdx' and report_unknown_license:
                        #         report += ("* {} has unknown SPDX: {}\n".format(orig_path, lic['license_expression_spdx']))
                if check_copyright and not ignore_copyright and not file['copyrights'] and \
                        file.get("programming_language") != 'CMake':
                    report += ("* {} missing copyright.\n".format(orig_path))


    return report


def parse_args():
    parser = argparse.ArgumentParser(
        description="Analyze licenses...")
    parser.add_argument('-s', '--scancode-output',
                        help='''JSON output from scancode-toolkit''')
    parser.add_argument('-f', '--scanned_files',
                        help="Directory with scanned files")
    parser.add_argument('-c', '--config_file',
                        help="Configuration file")
    parser.add_argument('-o', '--output_file',
                        help="Output report file")
    return parser.parse_args()

if __name__ == "__main__":

    args = parse_args()
    if args.scancode_output and args.scanned_files and args.config_file:
        report = analyze_file(args.config_file, args.scancode_output, args.scanned_files)
        if report:
            with open(args.output_file, "w") as fp:
                fp.write(report)
            print("Copyright check FAILED.")
            sys.exit(1)
        else:
            print("Copyright check PASSED.")
            sys.exit(0)
    else:
        sys.exit("Provide files to analyze")
