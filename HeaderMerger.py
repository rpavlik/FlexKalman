#!/usr/bin/env python3 -i
# Copyright 2019 Collabora, Ltd
# SPDX-License-Identifier: BSL-1.0
# Author: Ryan Pavlik <ryan.pavlik@collabora.com>

import re

import networkx as nx
from networkx.algorithms.dag import topological_sort


class HeaderMergerBase:
    def __init__(self):
        self.graph = nx.DiGraph()
        # self.file_contents = {}
        # self.file_includes = {}
        self.paths = {}
        self.include_re = re.compile(
            r'^ *# *include *(?P<quoted_name>(?P<open>["<])(?P<name>[^">]*)[">])')

    def is_start_marker(self, line):
        """Return True if lines following this one should be treated as "contents".

        You must implement this."""
        raise NotImplementedError

    def is_end_marker(self, line):
        """Return True if processing of this file should stop.

        You must implement this."""
        raise NotImplementedError

    def get_external_include_from_match(self, match):
        """Return the string that represents an external include, or None if the match represents an internal header include that should be replaced.

        The basic implementation here assumes that angle-brackets means external include.

        A match will be transformed into a dependency in the graph if this returns True.
        It will be kept as an external include if this returns False."""
        if self.is_match_angle_brackets(match):
            # Regenerate with no extra spaces, so the set can de-duplicate nicely.
            return "#include {}".format(match.group("quoted_name"))
        return None

    def is_match_angle_brackets(self, match):
        return match.group("open") == "<"

    def parse_files(self, files):
        for fn in files:
            self.parse_file(fn)

    def parse_file(self, fn):
        self.paths[fn.name] = fn
        with open(str(fn), 'r', encoding='utf') as f:
            self.parse_from_stream(fn.name, f)

    def parse_from_stream(self, name, stream):
        # locals
        G = self.graph
        lines = []
        other_includes = set()

        # Add graph node and attributes
        G.add_node(name)
        G.nodes[name]['contents'] = lines
        G.nodes[name]['includes'] = other_includes
        # self.file_contents[name] = lines
        # self.file_includes[name] = other_includes
        started = False

        for line in stream:
            line = line.rstrip()
            match = self.include_re.match(line)
            if match:
                # An include line
                external_include = self.get_external_include_from_match(match)
                if external_include:
                    other_includes.add(external_include)
                else:
                    # the current name depends on the included name
                    included_name = match.group("name")
                    G.add_edge(name, included_name)
                continue  # with next line

            # Not an include line - what kind of line is it?

            # Is it the marker that indicates the next line is the first to keep as "content"?
            if not started and self.is_start_marker(line):
                started = True
                continue  # with next line

            # Is it the marker that indicates that there is no more content?
            if started and self.is_end_marker(line):
                return

            # If we're in a "content" line, append it to the list
            if started:
                lines.append(line)

    def get_sorted_deps(self, name=None):
        """Returns an appropriately-sorted list of filenames that are name or depended-on by name.

        Each file appears earlier in the list than any file including it.

        If name is None, gets all sorted deps."""
        if name:
            nodes = [name]
            nodes.extend(self.graph.successors(name))
            graph = self.graph.subgraph(nodes)
        else:
            graph = self.graph
        return list(reversed(list(topological_sort(graph))))

    def get_other_includes_of(self, names):
        "Return the set of other includes from the given file names."
        return set().union(*(self.graph.nodes[name]["includes"] for name in names))

    def get_contents_of(self, names):
        "Return a list of lines from the given file names, in the given order."
        contents = []
        for name in names:
            contents.extend(self.graph.nodes[name]['contents'])
        return contents
