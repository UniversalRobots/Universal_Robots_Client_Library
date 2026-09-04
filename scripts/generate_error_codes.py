#!/usr/bin/env python3
# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2026 Universal Robots A/S
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# -- END LICENSE BLOCK ------------------------------------------------

"""Generate or verify the error_code_texts.h header from UR's ErrorCodes JSON.

Usage
-----
Generate / update the header::

    generate_error_codes.py [--input URL_or_path] [--overlay path] --output path

Check whether the committed header is still in sync with the upstream JSON::

    generate_error_codes.py --check [--input URL_or_path] --header path

Exit code is non-zero when --check detects a version mismatch.

Key encoding
------------
Each (code, arg) pair is packed into a uint64_t:

    key = (uint32_t(code) << 32) | uint32_t(arg)

Entries in the JSON that have no "arg" field use the sentinel 0xFFFFFFFF so
that they can serve as code-only fallbacks in the C++ lookup.  The C++ cast
of int32_t(-1) to uint32_t also produces 0xFFFFFFFF, so a message_argument_
of -1 matches those entries directly in the first lookup step.
"""

import argparse
import json
import re
import sys
import urllib.request
from pathlib import Path
from typing import Optional

UPSTREAM_URL = (
    "https://www.universal-robots.com/manuals/EN/PDF/Shared/ErrorCodes/ErrorCodes.json"
)

# Sentinel used for JSON entries that carry no "arg" field.
NO_ARG_SENTINEL = 0xFFFFFFFF


def load_json(source: str) -> dict:
    """Load JSON from a URL or a local file path."""
    if source.startswith("http://") or source.startswith("https://"):
        with urllib.request.urlopen(source) as response:
            return json.loads(response.read().decode("utf-8"))
    else:
        with open(source, encoding="utf-8") as f:
            return json.load(f)


def escape_c_string(text: str) -> str:
    """Escape a string so it is safe to embed as a C string literal.

    - Named escapes are used for common control characters.
    - All remaining C0 control characters (< 0x20) and DEL (0x7F) are
      emitted as three-digit octal sequences to avoid misinterpretation.
    - Characters above U+007F are encoded as individual UTF-8 bytes, each
      represented by a three-digit octal sequence.  This guarantees the
      generated header is valid on compilers that do not default to UTF-8
      (e.g. MSVC without /utf-8).
    """
    named = {
        "\\": "\\\\",
        '"': '\\"',
        "\n": "\\n",
        "\r": "\\r",
        "\t": "\\t",
        "\a": "\\a",
        "\b": "\\b",
        "\f": "\\f",
        "\v": "\\v",
    }
    result = []
    for char in text:
        if char in named:
            result.append(named[char])
        elif ord(char) < 0x20 or ord(char) == 0x7F:
            # Remaining C0 control characters and DEL
            result.append(f"\\{ord(char):03o}")
        elif ord(char) > 0x7F:
            # Non-ASCII: emit each UTF-8 byte as an octal escape so the
            # literal is portable across compiler/platform encoding settings.
            for byte in char.encode("utf-8"):
                result.append(f"\\{byte:03o}")
        else:
            result.append(char)
    return "".join(result)


def build_map(ur_data: dict, overlay_data: Optional[dict]) -> dict:
    """Return an ordered dict mapping uint64 keys to (code, arg_or_none, text).

    Overlay entries take precedence over UR entries on key collision.
    """
    entries: dict[int, tuple[int, Optional[int], str]] = {}

    def add_entries(data: dict) -> None:
        for entry in data.get("codes", []):
            if "text" not in entry:
                continue  # skip entries with no text (e.g. placeholder codes)
            code = int(entry["code"])
            arg = int(entry["arg"]) if "arg" in entry else None
            arg_key = NO_ARG_SENTINEL if arg is None else (arg & 0xFFFFFFFF)
            key = (code & 0xFFFFFFFF) << 32 | arg_key
            entries[key] = (code, arg, entry["text"])

    add_entries(ur_data)
    if overlay_data:
        add_entries(overlay_data)  # overlay wins on collision

    return entries


def read_version_from_header(header_path: Path) -> Optional[str]:
    """Extract the version string from a previously generated header."""
    content = header_path.read_text(encoding="utf-8")
    m = re.search(r'ERROR_CODE_JSON_VERSION\s*=\s*"([^"]+)"', content)
    return m.group(1) if m else None


def generate(args: argparse.Namespace) -> int:
    source = args.input or UPSTREAM_URL
    print(f"Loading UR ErrorCodes JSON from: {source}")
    ur_data = load_json(source)

    overlay_data = None
    if args.overlay:
        print(f"Loading overlay JSON from: {args.overlay}")
        overlay_data = load_json(args.overlay)

    version = ur_data.get("version", "unknown")
    entries = build_map(ur_data, overlay_data)

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    lines = []
    lines.append("// -- BEGIN LICENSE BLOCK " + "-" * 46)
    lines.append("// Copyright 2026 Universal Robots A/S")
    lines.append("//")
    lines.append("// Redistribution and use in source and binary forms, with or without")
    lines.append("// modification, are permitted provided that the following conditions are met:")
    lines.append("//")
    lines.append("//    * Redistributions of source code must retain the above copyright")
    lines.append("//      notice, this list of conditions and the following disclaimer.")
    lines.append("//")
    lines.append("//    * Redistributions in binary form must reproduce the above copyright")
    lines.append("//      notice, this list of conditions and the following disclaimer in the")
    lines.append("//      documentation and/or other materials provided with the distribution.")
    lines.append("//")
    lines.append("//    * Neither the name of the copyright holder nor the names of its")
    lines.append("//      contributors may be used to endorse or promote products derived from")
    lines.append("//      this software without specific prior written permission.")
    lines.append("//")
    lines.append('// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"')
    lines.append("// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE")
    lines.append("// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE")
    lines.append("// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE")
    lines.append("// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR")
    lines.append("// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF")
    lines.append("// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS")
    lines.append("// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN")
    lines.append("// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)")
    lines.append("// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE")
    lines.append("// POSSIBILITY OF SUCH DAMAGE.")
    lines.append("// -- END LICENSE BLOCK " + "-" * 48)
    lines.append("")
    lines.append("//----------------------------------------------------------------------")
    lines.append("/*!")
    lines.append(" * \\file")
    lines.append(" *")
    lines.append(" * AUTO-GENERATED FILE — DO NOT EDIT MANUALLY.")
    lines.append(" * Re-generate with:")
    lines.append(" *   cmake --build <build-dir> --target generate_error_codes")
    lines.append(" * or directly:")
    lines.append(" *   scripts/generate_error_codes.py --overlay scripts/error_code_overrides.json --output <this-file>")
    lines.append(f" *")
    lines.append(f" * Source: UR ErrorCodes JSON v{version}")
    lines.append(" */")
    lines.append("//----------------------------------------------------------------------")
    lines.append("")
    lines.append("#pragma once")
    lines.append("")
    lines.append("#include <cstdint>")
    lines.append("#include <string_view>")
    lines.append("#include <unordered_map>")
    lines.append("")
    lines.append("namespace urcl")
    lines.append("{")
    lines.append("namespace primary_interface")
    lines.append("{")
    lines.append("")
    lines.append("/// Version of the UR ErrorCodes JSON this header was generated from.")
    lines.append(f'constexpr std::string_view ERROR_CODE_JSON_VERSION = "{version}";')
    lines.append("")
    lines.append("/// Returns a static map from packed (code, arg) keys to human-readable")
    lines.append("/// error texts sourced from the UR ErrorCodes JSON.")
    lines.append("///")
    lines.append("/// Key encoding: upper 32 bits = error code, lower 32 bits = argument.")
    lines.append("/// Entries without an argument in the source JSON use 0xFFFFFFFF as the")
    lines.append("/// lower 32 bits (matches message_argument_ == -1 when cast to uint32_t).")
    lines.append("inline const std::unordered_map<uint64_t, const char*>& getErrorCodeTexts()")
    lines.append("{")
    lines.append("  // clang-format off")
    lines.append(
        "  static const std::unordered_map<uint64_t, const char*> error_code_map = {"
    )

    for key, (code, arg, text) in sorted(entries.items()):
        arg_comment = f"arg={arg}" if arg is not None else "no arg"
        escaped = escape_c_string(text)
        lines.append(
            f'    {{ UINT64_C(0x{key:016X}), "{escaped}" }},  // C{code} {arg_comment}'
        )

    lines.append("  };")
    lines.append("  // clang-format on")
    lines.append("  return error_code_map;")
    lines.append("}")
    lines.append("")
    lines.append("}  // namespace primary_interface")
    lines.append("}  // namespace urcl")
    lines.append("")

    output_path.write_text("\n".join(lines), encoding="utf-8")
    print(
        f"Written {len(entries)} entries (UR JSON v{version}) to {output_path}"
    )
    return 0


def check(args: argparse.Namespace) -> int:
    header_path = Path(args.header)
    if not header_path.exists():
        print(f"ERROR: Header not found: {header_path}", file=sys.stderr)
        return 1

    committed_version = read_version_from_header(header_path)
    if committed_version is None:
        print(
            f"ERROR: Could not extract ERROR_CODE_JSON_VERSION from {header_path}",
            file=sys.stderr,
        )
        return 1

    source = args.input or UPSTREAM_URL
    print(f"Loading UR ErrorCodes JSON from: {source}")
    ur_data = load_json(source)
    upstream_version = ur_data.get("version", "unknown")

    if committed_version == upstream_version:
        print(
            f"OK: Header version ({committed_version}) matches upstream JSON version ({upstream_version})."
        )
        return 0
    else:
        print(
            f"MISMATCH: Header version is '{committed_version}' but upstream JSON version is "
            f"'{upstream_version}'.",
            file=sys.stderr,
        )
        print(
            "Run 'cmake --build <build-dir> --target generate_error_codes' to update.",
            file=sys.stderr,
        )
        return 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate or verify error_code_texts.h from UR ErrorCodes JSON."
    )
    parser.add_argument(
        "--input",
        metavar="URL_or_path",
        default=None,
        help=f"Source of UR ErrorCodes JSON (default: {UPSTREAM_URL})",
    )

    subparsers = parser.add_subparsers(dest="command")

    # --- generate (default) sub-command ---
    gen_parser = subparsers.add_parser("generate", help="Generate the header (default).")
    gen_parser.add_argument(
        "--overlay",
        metavar="path",
        default=None,
        help="Optional JSON overlay/extension file (same format as UR JSON).",
    )
    gen_parser.add_argument(
        "--output",
        metavar="path",
        required=True,
        help="Destination path for the generated header.",
    )

    # --- check sub-command ---
    check_parser = subparsers.add_parser(
        "check", help="Check that the committed header matches the upstream JSON version."
    )
    check_parser.add_argument(
        "--header",
        metavar="path",
        required=True,
        help="Path to the existing generated header.",
    )

    # Support old-style flat invocation for backwards compat and CMake ease:
    # generate_error_codes.py --output foo  (no sub-command)
    # generate_error_codes.py --check --header foo
    parser.add_argument("--check", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--overlay", metavar="path", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--output", metavar="path", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--header", metavar="path", default=None, help=argparse.SUPPRESS)

    args = parser.parse_args()

    # Resolve flat invocation (no sub-command)
    if args.command is None:
        if args.check:
            if not args.header:
                parser.error("--check requires --header")
            return check(args)
        elif args.output:
            return generate(args)
        else:
            parser.print_help()
            return 1

    if args.command == "generate":
        # Merge top-level --input into sub-command namespace
        if not hasattr(args, "input") or args.input is None:
            pass  # already None from subparser
        return generate(args)

    if args.command == "check":
        return check(args)

    parser.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
