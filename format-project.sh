#!/bin/sh
# Copyright 2019, 2021, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0
set -e
(
    cd "$(dirname $0)"
    find inc tests \( -name "*.cpp" -o -name "*.h" \) -exec clang-format -i -style=file \{\} +
)
