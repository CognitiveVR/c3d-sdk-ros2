# Third-party notices

The Cognitive3D ROS 2 SDK is licensed under the Cognitive3D SDK Software License (`LICENSE`). It
incorporates the third-party components below, each under its own license. This file is installed
with the `cognitive3d_ros` and `cognitive3d_ros_nav2` packages, in `share/<package>/`.

## Compiled into the SDK

### nlohmann/json

A header-only JSON library. It is compiled into `libcognitive3d_ros_component.so` from the
distribution's `nlohmann-json3-dev` package: 3.10.5 on Ubuntu 22.04, 3.11.3 on Ubuntu 24.04.

```text
MIT License

Copyright (c) 2013-2022 Niels Lohmann (3.10.5)
Copyright (c) 2013-2023 Niels Lohmann (3.11.3)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

### Cap'n Proto and KJ

The serialization library for the camera and lidar payloads. The SDK links the distribution's
Cap'n Proto runtime dynamically (0.8.0 on Ubuntu 22.04, 1.0.1 on Ubuntu 24.04). Code from its
headers, and the C++ that its `capnpc-c++` compiler generates from the schemas in `core/capnp/`,
is compiled into `libcognitive3d_ros_component.so`.

```text
MIT License

Copyright (c) 2013-2017 Sandstorm Development Group, Inc. and contributors
Copyright (c) 2017-2023 Cloudflare, Inc. and contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```

## Included in this repository

### `core/capnp/java.capnp`

The annotation schema that the other schemas in `core/capnp/` import. It is an unmodified copy of
`compiler/src/main/schema/capnp/java.capnp` from
[capnproto-java](https://github.com/capnproto/capnproto-java), and carries its license header.

```text
Copyright (c) 2013-2015 Sandstorm Development Group, Inc. and contributors
Licensed under the MIT License:

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```

## Runtime dependencies

These are linked dynamically and are not redistributed with the SDK. The package manager installs
them from their own distributions, under their own licenses.

| component | license | reached through |
| -- | -- | -- |
| libcurl | curl (MIT-style) | the upload transport |
| zstd | BSD-3-Clause or GPL-2.0 | point cloud compression |
| Cap'n Proto runtime | MIT | the camera and lidar payloads |
| OpenCV | Apache-2.0 | `cv_bridge`, and decoding and scaling camera frames before they reach `ffmpeg` |
| ROS 2 (rclcpp, tf2, cv_bridge, message packages, Nav2 messages) | Apache-2.0 and BSD | the node |

## Runtime programs

These are executed as a separate process, from the distribution's own packages, and are never
linked into or redistributed with the SDK. The camera encoder runs the `ffmpeg` binary as a child,
writes raw frames to its standard input and reads the encoded stream from its standard output; no
code from either program is compiled into any Cognitive3D binary, and the SDK ships neither.

| program | license | reached through |
| -- | -- | -- |
| FFmpeg (`ffmpeg`) | GPL-2.0-or-later, as distributed by Ubuntu; GPL-3.0-or-later where Debian or Ubuntu's `libavcodec-extra` replaces the stock codec library, since that build enables the version 3 licenses | the camera H.264 encoder, as a child process |
| x264 (`libx264`, loaded by `ffmpeg`) | GPL-2.0-or-later | that same child process; the SDK never links it |
