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

"""Generate independent library and documentation RTDE output lists.

The exhaustive recipe remains plain field names extracted from the C++ source.
The documentation output is TEST ONLY TSV, without a header:
    name<TAB>PS5 availability<TAB>PSX availability
Availability is '*' (unrestricted), '-' (unavailable), or major.minor.patch.
Do not pass this metadata file to the production RTDE recipe reader.
"""

import pathlib
import re

import pandas as pd

URCL_PATH = pathlib.Path(__file__).parent.parent.parent.resolve()
OUTPUT_PATH = pathlib.Path(__file__).parent / "exhaustive_rtde_output_recipe.txt"
WEB_OUTPUT_PATH = pathlib.Path(__file__).parent / "docs_rtde_output_recipe.txt"
DOCS_URL = "https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html"
VERSION_COLUMN = "Introduced in Version"
REQUIRED_COLUMNS = {"Name", "Type", "Comment", VERSION_COLUMN}
BASELINES = {5: (5, 9, 0), 10: (10, 10, 0)}
VERSION_PATTERN = r"[0-9]+\.[0-9]+(?:\.[0-9]+)?"
REGISTER_PATTERN = r"(?:input|output)_(?:bit|int|double)_register_X"


def cell_text(value):
    """Normalize empty pandas cells and documentation whitespace."""
    if pd.isna(value):
        return ""
    if not isinstance(value, str):
        raise ValueError(f"Expected a documentation text cell, got {value!r}")
    return " ".join(value.split())


def parse_availability(name, annotation):
    """Parse field introductions, resolving family presence before cutoffs."""
    text = cell_text(annotation)
    # This exact documented annotation introduces a new bit, NOT the field.
    # Keep the exception narrow so future field/bit annotations need review.
    if (name == "safety_status_bits"
            and text == "5.25.0 / 10.12.0 Is 3PE input active"):
        return "*", "*"

    # The upper half of these registers was introduced before our PS5 baseline.
    # Do not apply range-specific thresholds to every register: only this known
    # legacy annotation is safe to discard. Other ranges/versions fail below.
    if (re.fullmatch(r"(?:input|output)_(?:int|double)_register_X", name)
            and re.fullmatch(r"\[\s*24\s*\.\.\s*47\s*\]\s*5\.3\.0", text)):
        return "*", "*"
    if "[" in text or "]" in text:
        raise ValueError(f"{name}: unsupported range-specific version annotation {text!r}")
    if not text:
        return "*", "*"
    if not re.fullmatch(rf"{VERSION_PATTERN}(?:\s*/\s*{VERSION_PATTERN})*", text):
        raise ValueError(f"{name}: malformed version annotation {text!r}")

    versions = {}
    for token in text.split("/"):
        components = tuple(int(part) for part in token.strip().split("."))
        version = components if len(components) == 3 else (*components, 0)
        major = version[0]
        if major not in (3, 5, 10):
            raise ValueError(f"{name}: unsupported version family in {text!r}")
        if major in versions:
            raise ValueError(f"{name}: duplicate version family in {text!r}")
        versions[major] = version

    availability = []
    for major in (5, 10):
        version = versions.get(major)
        if version is None:
            availability.append("-" if major == 5 and 10 in versions else "*")
        elif version < BASELINES[major]:
            availability.append("*")
        else:
            availability.append(".".join(str(part) for part in version))
    return tuple(availability)


def expand_register_names(name, comment):
    """Expand the documented count from the first register address, in order."""
    if not re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*", name):
        raise ValueError(f"Invalid output field name {name!r}")
    if not name.endswith("_X"):
        return [name]
    if not re.fullmatch(REGISTER_PATTERN, name):
        raise ValueError(f"Unsupported register placeholder {name!r}")
    text = cell_text(comment)
    count = re.match(r"([0-9]+)\s+general purpose\b", text)
    ranges = re.findall(r"\[\s*([0-9]+)\s*\.\.\s*([0-9]+)\s*\]", text)
    if count is None or not ranges:
        raise ValueError(f"{name}: malformed register description {text!r}")
    amount = int(count.group(1))
    base = int(ranges[0][0])
    next_address = base
    for start, end in ranges:
        if int(start) != next_address or int(end) < int(start):
            raise ValueError(f"{name}: noncontiguous register ranges {text!r}")
        next_address = int(end) + 1
    if amount == 0 or next_address != base + amount:
        raise ValueError(f"{name}: register count/range mismatch {text!r}")
    return [name[:-1] + str(base + offset) for offset in range(amount)]


def validate_output_table(table):
    """Fail clearly if the public documentation schema changes."""
    if not isinstance(table, pd.DataFrame):
        raise ValueError("Expected an RTDE output DataFrame")
    if not table.columns.is_unique or not REQUIRED_COLUMNS.issubset(table.columns):
        raise ValueError(f"RTDE output table requires columns {sorted(REQUIRED_COLUMNS)!r}")
    if table.empty:
        raise ValueError("RTDE output table is empty")


def select_output_table(tables):
    """The public guide's second table is outputs; verify its timestamp anchor."""
    if len(tables) < 2:
        raise ValueError("RTDE documentation is missing the second (output) table")
    table = tables[1]
    validate_output_table(table)
    if "timestamp" not in table["Name"].values:
        raise ValueError("The second documentation table is not the RTDE output table (missing timestamp)")
    return table


def documentation_records(table):
    """Return ordered, exhaustive test metadata; never filter out old fields."""
    validate_output_table(table)
    records = []
    seen = set()
    for _, row in table.iterrows():
        name = cell_text(row["Name"])
        availability = parse_availability(name, row[VERSION_COLUMN])
        for field in expand_register_names(name, row["Comment"]):
            if field in seen:
                raise ValueError(f"Duplicate documented output field {field!r}")
            seen.add(field)
            records.append((field, *availability))
    return records


def extract_exhaustive_outputs(lines):
    """Keep the original source-based extraction independent of documentation."""
    save_outputs = False
    outputs = []
    for line in lines:
        if "// INPUT / OUTPUT" in line:
            save_outputs = True
        if save_outputs:
            if "//" not in line and len(line) > 1:
                outputs.append(line.split('"')[1] + "\n")
        if "// NOT IN OFFICIAL DOCS" in line:
            break
    return outputs


def main():
    """Generate both resources with the existing no-argument CLI."""
    package_paths = list(URCL_PATH.glob("**/data_package.cpp"))
    if len(package_paths) != 1:
        raise ValueError(f"Expected one data_package.cpp, found {len(package_paths)}")
    with package_paths[0].open() as pkg_file:
        outputs = extract_exhaustive_outputs(pkg_file)

    # Preserve two-component versions as text (float inference loses 10.10).
    tables = pd.read_html(DOCS_URL, converters={VERSION_COLUMN: str})
    records = documentation_records(select_output_table(tables))
    with open(OUTPUT_PATH, "w") as output_file:
        output_file.writelines(outputs)
    with open(WEB_OUTPUT_PATH, "w") as web_output_file:
        web_output_file.writelines("\t".join(record) + "\n" for record in records)


if __name__ == "__main__":
    main()
