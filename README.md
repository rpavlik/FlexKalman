# UVBI Video+Inertial Tracking

This is the core of the Kalman filter framework code from OSVR-Core,
plus the newer "unified" video tracker plugin,
modified to build without OSVR.

## License and Vendored Projects

This project: Licensed under the Apache License, Version 2.0.

Some directories under `/vendor` are in fact not external libraries vendored-in, but "internally-vendored" - developed as a part of OSVR-Core, and likewise licensed under the Apache License 2.0, but are logically distinct from the Core in their functionality and are thus kept separate in anticipation of potential splitting off into a separate project at some future point. These include `/vendor/comutils`.

- `/cmake` - Git subtree from <https://github.com/rpavlik/cmake-modules> used at compile-time only. Primarily BSL 1.0, some items under the license used by CMake (BSD-style)
- `/vendor/com_smart_pointer` - Header (for integrating `boost::intrusive_ptr` with MS COM) extracted from the following project and file: <https://github.com/rengeln/nyx/blob/master/src/Prefix.h>, modified as needed for compatibility. MIT license.
- `/vendor/eigen` - Unpacked release from <http://eigen.tuxfamily.org/> - header-only library under the MPL2 (a file-level copyleft, compatible with proprietary software), define `EIGEN_MPL2_ONLY` to exclude modules with other license from the build (done in OSVR-Core build system).
- `/vendor/FloatExceptions` - Modified code ([original author: Bruce Dawson](http://randomascii.wordpress.com/2012/04/21/exceptional-floating-point/) for development use only, usage should not be committed or merged into master. MIT licensed.
- `/vendor/folly` - Submodule of [C++11 components originally developed and widely used at Facebook](https://github.com/facebook/folly). Apache License, Version 2.0.
- `/vendor/util-headers` - Subset of headers installed from <https://github.com/rpavlik/util-headers>. BSL 1.0.
