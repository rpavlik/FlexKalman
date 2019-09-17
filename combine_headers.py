#!/usr/bin/env python3
# Copyright 2019 Collabora, Ltd
# SPDX-License-Identifier: BSL-1.0
# Author: Ryan Pavlik <ryan.pavlik@collabora.com>

from pathlib import Path

from HeaderMerger import HeaderMergerBase

REPO_ROOT = Path(__file__).resolve().parent
INCLUDE_DIR = REPO_ROOT / 'inc' / 'FlexKalman'
FULL_MERGE_OUTFILE = REPO_ROOT / 'generated' / 'FlexKalman.h'

META_HEADER = "FlexibleKalmanMeta.h"

NAMESPACE_STRING = "namespace flexkalman"
START_NAMESPACE = NAMESPACE_STRING + " {"
END_NAMESPACE = "} // " + NAMESPACE_STRING

HEADER_SUFFIXES = set(('.h', '.hpp'))

GENERATED_WARNING = """
    NOTE: This is a generated single-file version of FlexKalman - do not edit directly!
    Instead, edit the individual source files and regenerate this with combine_headers.py.
"""


def get_headers(base_dir):
    for fn in base_dir.iterdir():
        if not fn.is_file():
            continue
        if fn.suffix in HEADER_SUFFIXES:
            yield fn


def join_lines(lines):
    return '\n'.join(lines) + '\n'


def get_template_parts(include_dir):
    prefix = []
    between_includes_and_content = []
    after_content = (END_NAMESPACE,)
    with open(include_dir / META_HEADER, 'r', encoding='utf-8') as f:
        step = 1

        for line in f:
            line = line.rstrip()
            if step == 1:
                # Before the includes

                if "Header including" in line:
                    # Adjust the @brief comment line
                    line = line.replace(
                        "Header including", "Generated single-file header containing")

                if "@date" in line:
                    # Stick the warning in here
                    prefix.append(GENERATED_WARNING)

                prefix.append(line)
                # looking for the pragma - the last line in this segment
                if "#pragma once" in line:
                    step = 2
            elif step == 2:
                # Between the includes and the open of the namespace
                if START_NAMESPACE in line:
                    between_includes_and_content.append(START_NAMESPACE)
                    break
                between_includes_and_content.append(line)

    return (prefix, between_includes_and_content, after_content)


class FlexKalmanMerger(HeaderMergerBase):
    def __init__(self, include_dir):
        self.include_dir = include_dir
        #self.files = list(get_headers(include_dir))
        self.files = [fn for fn in get_headers(include_dir)
                      if fn.name != META_HEADER]
        self.known_filenames = set((fn.name for fn in self.files))
        super().__init__()

    def get_external_include_from_match(self, match):
        """Identify external includes by checking if this file is in our group of files to merge."""
        if match.group("name") not in self.known_filenames:
            # Regenerate with no extra spaces, so the set can de-duplicate nicely.
            return "#include {}".format(match.group("quoted_name"))
        return None

    def is_start_marker(self, line):
        return line == START_NAMESPACE

    def is_end_marker(self, line):
        return line == END_NAMESPACE

    def parse_all(self):
        self.parse_files(self.files)

    def write(self, fn):
        sorted_names = self.get_sorted_deps()
        prefix, between_includes_and_content, after_content = get_template_parts(
            self.include_dir)
        lines = prefix
        lines.extend(sorted(self.get_other_includes_of(sorted_names)))
        lines.extend(between_includes_and_content)
        lines.extend(self.get_contents_of(sorted_names))
        lines.extend(after_content)
        with open(fn, 'w', encoding='utf-8') as f:
            f.write(join_lines(lines))


if __name__ == "__main__":
    print(INCLUDE_DIR)
    full_merge = FlexKalmanMerger(INCLUDE_DIR)
    full_merge.parse_all()
    full_merge.write(str(FULL_MERGE_OUTFILE))
