#!/usr/bin/env python3
# Copyright 2019, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0
# Author: Ryan Pavlik
"""Script to verify and adjust includes for minimal but consistent header usage."""

import argparse
import enum
import json
import re
import subprocess
import sys
from itertools import chain
from pathlib import Path
from pprint import pprint

import networkx as nx
from networkx.readwrite import json_graph


class IncludeType(enum.Enum):
    QUOTES = '"'
    ANGLE_BRACKETS = "<"

    @staticmethod
    def from_string(s):
        if s.startswith('<') or s.startswith('>'):
            return IncludeType.ANGLE_BRACKETS
        if s.startswith('"'):
            return IncludeType.QUOTES
        raise RuntimeError("didn't look like quotes or angle brackets")

    def wrap(self, s):
        end = self.value
        if self.value == "<":
            end = ">"
        return ''.join((self.value, s, end))


class FileMigrationBase:
    def __init__(self, dirs, extra_search_dirs, include_dirs):
        self.graph = nx.DiGraph()
        self.include_re = re.compile(
            r'^ *# *include *(?P<quoted_name>(?P<open>["<])(?P<name>[^">]*)[">])')
        self.dirs = set(dirs)
        self.all_dirs = self.dirs.union(extra_search_dirs)
        self.include_dirs = include_dirs
        self._reversed = None
        for d in self.dirs:
            self.parse_dir(d)

    def find_in_dir_list(self, rel_path, dirs=None, try_drop_leading_path=False):
        if dirs is None:
            dirs = self.all_dirs
        candidates = (d/rel_path for d in dirs)
        found = [p for p in candidates if p.is_file()]
        if not found:
            # Check to see if there's a leading directory we can strip
            rel_path = Path(rel_path)
            if try_drop_leading_path and rel_path != rel_path.name:
                return self.find_in_dir_list(rel_path.name, dirs=dirs)
            return rel_path, False

        if len(found) > 1:
            raise RuntimeError("Got more than one resolution:", found)

        return found[0], True

    def file_known(self, resolved_path):
        return self.graph.nodes[str(resolved_path)].get('known', False)

    def process_file(self, fn):
        """Add a file to the graph, and read and parse its includes into the graph."""
        if not fn.is_file():
            return
        self._reversed = None
        G = self.graph
        current_file_node = str(fn)
        G.add_node(current_file_node)
        G.nodes[current_file_node]['known'] = True
        with open(str(fn), 'r', encoding='utf') as fp:
            for line in fp:
                line = line.rstrip()
                match = self.include_re.match(line)
                if not match:
                    continue

                include_name = match.group('name')
                path, resolved = self.find_in_dir_list(
                    include_name, try_drop_leading_path=True)
                if not resolved:
                    continue

                angle_include = (match.group('open') == "<")
                node_name = str(path)
                G.add_edge(current_file_node, node_name,
                           include_path=include_name,
                           angle_include=angle_include)

    @property
    def reversed(self):
        if self._reversed is None:
            self._reversed = self.graph.reverse(copy=True)
        return self._reversed

    def parse_dir(self, d, **kwargs):
        """Process all source and header files in a directory."""
        for fn in d.glob('*.h'):
            self.process_file(fn, **kwargs)
        for fn in d.glob('*.cpp'):
            self.process_file(fn, **kwargs)

    def make_fn_absolute(self, fn):
        """Make a file absolute by searching by name."""
        fn = Path(fn)
        if not fn.is_absolute():
            new_fn, resolved = self.find_in_dir_list(fn)
            if not resolved:
                raise RuntimeError("Could not find file " + str(new_fn))
            fn = new_fn
        return str(fn)

    def get_transitive_includers(self, fn):
        """Get a set of all files that (transitively) include fn."""
        fn = self.make_fn_absolute(fn)
        if fn not in self.graph:
            raise RuntimeError(str(fn) + " not in graph")
        return set(nx.dfs_preorder_nodes(self.reversed, fn))

    def get_transitive_includes(self, fn):
        """Get a generator of all files transitively included by fn."""
        fn = self.make_fn_absolute(fn)
        if fn not in self.graph:
            raise RuntimeError(str(fn) + " not in graph")
        return nx.dfs_preorder_nodes(self.graph, fn)

    def get_transitive_includes_for_all(self, collection):
        """Return a set including all members of collection
        and all files transitively included by them."""
        ret = set(collection)
        for fn in collection:
            includes = set(self.get_transitive_includes(fn))
            ret = ret.union(includes)
        return ret

    def get_include_search_path_for(self, includer, angle_include=False):
        search_path = []
        if not angle_include:
            includer = Path(includer)
            assert(includer.is_absolute())
            includer_dir = includer.parent
            search_path = [includer_dir]
        search_path.extend(self.include_dirs)
        return search_path

    def resolve_include(self, includer, path_included, angle_include=False):
        search_path = self.get_include_search_path_for(
            includer, angle_include=angle_include)
        full_include, resolved = self.find_in_dir_list(
            path_included, search_path)
        if not resolved:
            return None
        return full_include

    def make_most_concise_include(self, includer, resolved_include, angle_include=False):
        resolved_include = Path(resolved_include)
        assert(resolved_include.is_absolute())
        search_path = self.get_include_search_path_for(
            includer, angle_include=angle_include)
        for d in search_path:
            try:
                relative = resolved_include.relative_to(d)
                return str(relative)
            except ValueError:
                continue
        return None


APPS_BY_MODULE = {
    'videotrackershared': ('BlobExtractionDemo',),
    'unifiedvideoinertial': ('camera-latency-testing',
                             'OfflineProcessing',
                             'ParameterFinder',
                             'ViewTrackingCamera',)
}

TESTS_BY_MODULE = {
    'FlexKalman': ('Kalman', 'Util'),
    'unifiedvideoinertial': ('unifiedvideoinertial',)
}


def dir_list_from_modules(root, inc, src, modules):
    dirs_to_process = [inc / m for m in modules]
    dirs_to_process.extend((src / m for m in modules))
    for mod in modules:
        apps = APPS_BY_MODULE.get(str(mod))
        if apps:
            dirs_to_process.extend((root/'apps'/mod/app for app in apps))
        tests = TESTS_BY_MODULE.get(str(mod))
        if tests:
            dirs_to_process.extend((root/'tests'/'cplusplus'/t for t in tests))
    return dirs_to_process


class UVBIFileMigration(FileMigrationBase):
    def __init__(self, root, active_modules, inactive_modules):
        self.root = Path(root).resolve()
        self.inc = self.root / 'inc'
        self.inc_dir_name = str(self.inc)
        self.src = self.root / 'src'

        self.banned_dirs = dir_list_from_modules(
            self.root, self.inc, self.src, inactive_modules)

        dirs = dir_list_from_modules(
            self.root, self.inc, self.src, active_modules)

        extra_dirs = [self.inc, self.src] + self.banned_dirs
        super().__init__(
            dirs=dirs,
            extra_search_dirs=extra_dirs,
            include_dirs=(self.inc,))

    def is_public_header(self, fn):
        return str(fn).startswith(self.inc_dir_name)

    def get_public_headers(self):
        return (f for f in self.graph if self.is_public_header(f))

    def make_headers_public(self, headers):
        """Move these headers from the source directory to the public include directory."""
        for fn in headers:
            fn = Path(fn)
            relative = fn.relative_to(self.src)
            dest = self.inc / relative
            # Make dest directory before move, if required.
            try:
                dest.parent.mkdir()
            except FileExistsError:
                pass
            print("Moving", self.shorten(fn), "to", self.shorten(dest))
            fn.rename(dest)

    def fix_public_transitive_headers(self):
        """Make sure all transitive includes of public header are also public."""
        public_headers = set(self.get_public_headers())
        print("Public headers:", len(public_headers))
        transitive = self.get_transitive_includes_for_all(public_headers)
        print("Transitive includes of public headers:", len(transitive))
        must_move = transitive.difference(public_headers)
        if not must_move:
            # Don't need to move anything
            return False
        self.make_headers_public(must_move)
        return True

    def shorten(self, fn):
        try:
            return str(Path(fn).relative_to(self.root))
        except ValueError:
            return str(fn)

    def modify_include(self, fn,
                       old_include,
                       new_include,
                       old_style=IncludeType.QUOTES,
                       new_style=IncludeType.QUOTES):
        old_quoted = old_style.wrap(old_include)
        new_quoted = new_style.wrap(new_include)
        pattern = 's:{}:{}:'.format(
            # Escape periods
            old_quoted.replace(".", r"[.]"),
            new_quoted)
        cmd = ['sed', '-i', pattern, str(fn)]
        print("Fixing include in", self.shorten(fn),
              "by applying substitution:", pattern)
        subprocess.check_call(cmd)

    def fix_includes(self):
        """Correct and simplify include lines, and find files that should be public headers."""
        G = self.graph
        possible_public = set()
        include_fixes = []
        for includer, included, path_used in G.edges(data='include_path'):
            if not self.file_known(included):
                # Skip system/external dep includes
                continue

            angle_include = G.edges[includer, included]['angle_include']
            if angle_include:
                print(self.shorten(includer), self.shorten(included))
            # Make sure includes can be found,
            # and shorten up quote includes
            best_include = self.make_most_concise_include(
                includer,
                included,
                angle_include=angle_include
            )
            if not best_include:
                print("Couldn't figure out how to refer to",
                      self.shorten(included), "when including from", self.shorten(includer))
                possible_public.add(included)
                continue

            if angle_include:
                # This replaces in-project usage of angle brackets with quotes.
                include_fixes.append(
                    (includer, path_used, best_include,
                     IncludeType.ANGLE_BRACKETS, IncludeType.QUOTES))
                continue
            if best_include != path_used:
                include_fixes.append((includer, path_used, best_include))

        # If there are headers to make public, do that first
        # and skip include fixes, since they'll be different after.
        if possible_public:
            print("These headers might need to be made public:")
            for fn in possible_public:
                print("  -", self.shorten(fn))
            return True, possible_public

        # Only include fixes - do them.
        if include_fixes:
            for args in include_fixes:
                self.modify_include(*args)
            return True, None

        # No changes
        return False, None

    def check_for_banned_dirs(self):
        problems = []
        for includer, included, _path_used in self.graph.edges(data='include_path'):
            included = Path(included)
            if not included.is_absolute():
                included, resolved = self.find_in_dir_list(included)
                if not resolved:
                    print("Could not resolve this file, skipping ban check", included)
                    continue
            for d in self.banned_dirs:
                try:
                    _ = included.relative_to(d)
                    problems.append((includer, included))
                    break
                except ValueError:
                    continue
        if problems:
            print("Found these banned cross-module includes:")
            for includer, included in problems:
                print(self.shorten(includer),
                      "tries to include", self.shorten(included))
            raise RuntimeError("Banned cross-module includes found")


# G = migration.graph

# with open('includes.json', 'w', encoding='utf-8') as fp:
#     json.dump(json_graph.node_link_data(migration.graph), fp, indent=4)

def do_fixup_loop(root, active_modules, inactive_modules):
    print()
    print("Running fixup loop")
    print("    Active modules:", active_modules)
    print("    Inactive modules:", inactive_modules)
    while True:
        migration = UVBIFileMigration(root, active_modules, inactive_modules)

        print("Ensuring includes of public headers are public...")
        if migration.fix_public_transitive_headers():
            print("Some headers moved!")
            continue
            # migration = UVBIFileMigration(ROOT)

        print("Correcting include lines")
        changes, possible_public = migration.fix_includes()
        if not changes:
            print("Processing done!")
            break

        if possible_public:
            print("Moving those headers to become public")
            migration.make_headers_public(possible_public)

    migration.check_for_banned_dirs()


modules = [
    'FlexKalman',
    'videotrackershared',
    Path('videotrackershared')/'ImageSources',
    'folly',
    'unifiedvideoinertial'
]


def process(root):
    for i in range(len(modules)):

        active_modules = modules[:i+1]
        inactive_modules = modules[i+1:]
        print()
        print("------------------------------------")
        print("    Active modules:", active_modules)
        print("    Inactive modules:", inactive_modules)

        migration = UVBIFileMigration(root, active_modules, inactive_modules)

        print("Ensuring includes of public headers are public...")
        if migration.fix_public_transitive_headers():
            print("Some headers moved!")
            return True

        print("Correcting include lines")
        changes, possible_public = migration.fix_includes()

        if possible_public:
            print("Moving those headers to become public")
            migration.make_headers_public(possible_public)

        if changes:
            return True

    migration.check_for_banned_dirs()


def make_uvbi_migration(root):
    return UVBIFileMigration(root, modules, ())


def get_root_based_on_file():
    return Path(__file__).resolve().parent


def normalize():
    ROOT = get_root_based_on_file()
    while True:
        if process(ROOT):
            print("Changes made, repeating")
        else:
            break


def alt_main():
    ROOT = get_root_based_on_file()
    NOT_IMPORTED = set(("AbsoluteOrientationMeasurement.h",
                        "AbsolutePositionMeasurement.h",
                        "AngularVelocityMeasurement.h",
                        "AugmentedProcessModel.h",
                        "AugmentedState.h",
                        "ConstantProcess.h",
                        "EigenQuatExponentialMap.h",
                        "ExternalQuaternion.h",
                        "FlexibleKalmanBase.h",
                        "FlexibleKalmanCorrect.h",
                        "FlexibleKalmanFilter.h",
                        "MatrixExponentialMap.h",
                        "OrientationConstantVelocity.h",
                        "OrientationState.h",
                        "PoseConstantVelocity.h",
                        "PoseDampedConstantVelocity.h",
                        "PoseSeparatelyDampedConstantVelocity.h",
                        "PoseState.h",
                        "PoseStateExponentialMap.h",
                        "PureVectorState.h",
                        "SigmaPointGenerator.h",
                        ))
    missing_orig = set()
    m = make_uvbi_migration(ROOT)
    pub_headers = set(m.get_public_headers())
    flexkalman = (h for h in pub_headers if 'FlexKalman' in h)
    added = (h for h in flexkalman if Path(h).name not in NOT_IMPORTED)
    from pprint import pprint
    for h in added:
        print(h)
        public_includers = set(
            m.get_transitive_includers(h)).intersection(pub_headers)
        pprint(public_includers)
    #     for includer in m.get_transitive_includers(h):
    #         name = Path(includer).name
    #         if Path(includer).name in NOT_IMPORTED:
    #             missing_orig.add(Path(h).name)
    # pprint(missing_orig)


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--includers",
        help="Show the (transitive) includers of the file(s).",
        action='store_true')
    parser.add_argument(
        "--includes",
        help="Show the (transitive) includes of the file(s).",
        action='store_true')
    parser.add_argument(
        "file",
        help="The filenames to show includers or includes of. If all args are omitted, the full normalization proceeds.",
        nargs="*")

    args = parser.parse_args()

    if args.includes or args.includers:
        m = make_uvbi_migration(get_root_based_on_file())
        for f in args.file:
            print()
            if args.includes:
                print(f, "includes the following files (transitively):")
                for other in sorted(set(m.get_transitive_includes(f))):
                    print("    -", m.shorten(other))
            if args.includers:
                print(f, "is included by the following files (transitively):")
                for other in sorted(set(m.get_transitive_includers(f))):
                    print("    -", m.shorten(other))
    else:
        normalize()


if __name__ == "__main__":
    main()
    # alt_main()
