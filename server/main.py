#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#

import argparse
from .server import mcp

try:
    import extensions  # noqa

    print("Private extensions loaded successfully")
except ImportError as e:
    print(f"{e}.\nRunning with public tools only")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t",
        "--transport",
        type=str,
        default="stdio",
        choices=["stdio", "sse"],
        help="Transport being use in MCP server",
    )
    args, _ = parser.parse_known_args()
    transport: str = args.transport
    print(f'Starting MCP - ROS 2 server using "{transport}" transport')

    mcp.run(transport=transport)


if __name__ == "__main__":
    main()
