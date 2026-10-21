# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2025 Universal Robots A/S
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
#    * Neither the name of the {copyright_holder} nor the names of its
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

import pathlib
import re

import pandas as pd

URCL_PATH = pathlib.Path(__file__).resolve().parents[2]
DOCS_URL = "https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html"
MINIMUM_FIELDS = 101
INPUT_OUTPUT_MARKER = "  // INPUT / OUTPUT\n"
UNDOCUMENTED_MARKER = "  // NOT IN OFFICIAL DOCS\n"
DATA_TYPES = {
    "BOOL", "UINT8", "UINT32", "UINT64", "INT32", "DOUBLE",
    "VECTOR3D", "VECTOR6D", "VECTOR6INT32", "VECTOR6UINT32",
}
NAME = r"[a-z][A-Za-z0-9_]*"
# Match a C++ { "field_name", "TYPE" }, entry, allowing spaces and tabs around punctuation.
ENTRY = re.compile(rf'''
    \s*\{{\s*                   # opening brace
    "(?P<name>{NAME})"          # field name
    \s*,\s*
    "(?P<type>[A-Z0-9]+)"       # type
    \s*\}},\s*                  # closing brace and comma
''', re.VERBOSE)


def validate_and_add(fields, name, data_type) -> None:
    """Validate a field definition, then add it to the supplied dictionary."""
    if not isinstance(name, str) or not re.fullmatch(NAME, name):
        raise ValueError(f"Invalid RTDE field name: {name!r}")
    if data_type not in DATA_TYPES:
        raise ValueError(f"Unknown RTDE type for {name}: {data_type!r}")
    if name in fields:
        raise ValueError(f"Duplicate RTDE field: {name}")
    fields[name] = data_type


def extract_fields(document) -> dict[str, str]:
    """Read the second table: controller inputs come first, outputs second."""
    page = pd.read_html(document, flavor="lxml")
    if len(page) < 2:
        raise ValueError(f"Expected at least two tables on the RTDE docs page, found {len(page)}")
    table = page[1]
    required = {"Name", "Type", "Comment"}
    if not required.issubset(table.columns):
        raise ValueError(f"Output table is missing columns: {sorted(required - set(table.columns))}")

    fields = {}
    for _, row in table.iterrows():
        name, data_type = row["Name"], row["Type"]
        if isinstance(name, str) and name.endswith("_X"):
            numbers = re.findall(r'\d+', row["Comment"])
            if len(numbers) < 2:
                raise ValueError(f"Missing register count/start index for {name}")
            amount, base_addr = int(numbers[0]), int(numbers[1])
            for index in range(amount):
                validate_and_add(fields, name[:-1] + str(base_addr + index), data_type)
        else:
            validate_and_add(fields, name, data_type)
    if len(fields) < MINIMUM_FIELDS:
        raise ValueError(f"Expected at least {MINIMUM_FIELDS} output fields, found {len(fields)}")
    return fields


def update_server(source, documented) -> tuple[str, dict[str, str], list[str]]:
    """Read output entries line by line, updating types and retaining missing fields."""
    before, section = source.split(INPUT_OUTPUT_MARKER)
    section, after = section.split(UNDOCUMENTED_MARKER)
    if section.count("  // OUTPUTS\n") != 1:
        raise ValueError("Expected exactly one OUTPUTS section")
    if "};" not in after:
        raise ValueError("Missing RTDE map terminator")

    outputs = {}
    changes = []
    lines = []
    for line in section.splitlines(keepends=True):
        if line.strip() and not line.lstrip().startswith("//"):
            match = ENTRY.fullmatch(line)
            if not match:
                raise ValueError(f"Unexpected RTDE map entry: {line.strip()}")
            name, old_type = match["name"], match["type"]
            data_type = documented.get(name, old_type)
            validate_and_add(outputs, name, data_type)
            if data_type != old_type:
                line = line.replace(f'"{old_type}"', f'"{data_type}"')
                changes.append(f"Updated {name}: {old_type} -> {data_type}")
        lines.append(line)

    # Do not duplicate names already maintained outside the output sections.
    protected = set(re.findall(r'\{\s*"(' + NAME + r')"\s*,', before + after))
    added = sorted(documented.keys() - outputs.keys())
    if protected.intersection(added):
        raise ValueError("New documented outputs collide with protected entries")
    additions = []
    for name in added:
        data_type = documented[name]
        validate_and_add(outputs, name, data_type)
        additions.append(f'  {{ "{name}", "{data_type}" }},\n')
        changes.append(f"Added {name}: {data_type}")

    updated = "".join(lines)
    if additions:
        updated = updated.rstrip("\n") + "\n" + "".join(additions) + "\n"
    updated_source = before + INPUT_OUTPUT_MARKER + updated + UNDOCUMENTED_MARKER + after
    return updated_source, outputs, changes


def update_resources(document, server_path, recipe_path, docs_recipe_path) -> list[str]:
    """Validate and render all resources before writing any of them."""
    documented = extract_fields(document)
    source = server_path.read_text(encoding="utf-8")
    updated, outputs, changes = update_server(source, documented)
    generated_files = {
        server_path: updated,
        recipe_path: "".join(name + "\n" for name in outputs),
        docs_recipe_path: "".join(name + "\n" for name in sorted(documented)),
    }
    for path, text in generated_files.items():
        if not path.exists() or path.read_text(encoding="utf-8") != text:
            path.write_text(text, encoding="utf-8")
    return changes


def main() -> None:
    resources_dir = URCL_PATH / "tests" / "resources"
    changes = update_resources(
        DOCS_URL,
        URCL_PATH / "tests" / "fake_rtde_server.cpp",
        resources_dir / "exhaustive_rtde_output_recipe.txt",
        resources_dir / "docs_rtde_output_recipe.txt",
    )
    for change in changes:
        print(change)
    print(f"RTDE resources updated: {len(changes)} field additions/type changes.")


if __name__ == "__main__":
    main()
