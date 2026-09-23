"""Offline regression tests for TEST ONLY RTDE documentation metadata."""

import importlib.util
import io
import pathlib
import runpy
import sys
from unittest.mock import mock_open

import pandas as pd
import pytest

import generate_rtde_outputs as generator


def make_table(*rows):
    return pd.DataFrame(rows, columns=["Name", "Type", "Comment", generator.VERSION_COLUMN])


@pytest.mark.parametrize("annotation, expected", [
    (None, ("*", "*")),
    (float("nan"), ("*", "*")),
    (pd.NA, ("*", "*")),
    ("", ("*", "*")),
    (" \t\n ", ("*", "*")),
    ("5.8.99 / 10.9.99", ("*", "*")),
    ("5.9.0 / 10.10.0", ("5.9.0", "10.10.0")),
    ("5.9 / 10.10", ("5.9.0", "10.10.0")),
    ("5.23.0 / 10.11.0", ("5.23.0", "10.11.0")),
    (" \n10.12.0\t /\u00a0 5.26.0 \n", ("5.26.0", "10.12.0")),
    ("10.14/5.26", ("5.26.0", "10.14.0")),
    ("5.11.0", ("5.11.0", "*")),
    ("5.3.0", ("*", "*")),
    ("10.15", ("-", "10.15.0")),
    ("10.15.1", ("-", "10.15.1")),
    ("10.10.0", ("-", "10.10.0")),
    ("10.9.99", ("-", "*")),
    ("10.7", ("-", "*")),
    ("3.14.3", ("*", "*")),
    ("3.14 / 10.7", ("-", "*")),
    ("3.14.3 / 5.9.0 / 10.11.0", ("5.9.0", "10.11.0")),
    ("5.3 / 10.15", ("*", "10.15.0")),
    ("5.11 / 10.7", ("5.11.0", "*")),
])
def test_availability(annotation, expected):
    assert generator.parse_availability("field", annotation) == expected


@pytest.mark.parametrize("annotation", [
    "5", "10", "5.9.0.1", "v5.9", "5.9.", ".5.9", "5..9",
    "5.9 /", "/ 5.9", "5.9 // 10.10", "5.9, 10.10", "5.9 10.10",
    "5.9.0 new feature", "available since 5.9.0", "-", "*", "NaN",
    "5.9e0", "-5.9", "5.-9", "5.9 / 11.1", "4.0", "6.0",
    "5.9 / 5.10", "10.10 / 10.11", "3.1 / 3.2", 5.9, 10,
])
def test_malformed_annotations_fail(annotation):
    with pytest.raises(ValueError):
        generator.parse_availability("field", annotation)


def test_safety_bit_exception_is_explicit_and_narrow():
    annotation = "5.25.0 / 10.12.0 Is 3PE input active"
    assert generator.parse_availability("safety_status_bits", annotation) == ("*", "*")
    assert generator.parse_availability("safety_status_bits", annotation.replace(" / ", "\n/\t")) == ("*", "*")
    assert generator.parse_availability("safety_status_bits", "5.25.0 / 10.12.0") == ("5.25.0", "10.12.0")
    with pytest.raises(ValueError, match="other_field"):
        generator.parse_availability("other_field", annotation)
    with pytest.raises(ValueError, match="malformed"):
        generator.parse_availability("safety_status_bits", annotation.replace("5.25.0", "5.26.0"))


@pytest.mark.parametrize("direction", ["input", "output"])
@pytest.mark.parametrize("kind", ["int", "double"])
def test_legacy_register_range(direction, kind):
    name = f"{direction}_{kind}_register_X"
    comment = f"48 general purpose {kind} registers (X: [0..23] reserved for Fieldbus/PLC, [24..47] for external RTDE clients)"
    records = generator.documentation_records(make_table((name, "INT32", comment, "[24..47] 5.3.0")))
    assert records == [(name[:-1] + str(i), "*", "*") for i in range(48)]


@pytest.mark.parametrize("annotation", [
    "[24..47] 5.9.0", "[24..47] 10.10.0", "[24..47] 10.7",
    "[24..47] 5.3.0 / 10.15", "[0..23] 5.3.0", "[24..47] nonsense",
])
def test_unsupported_register_version_ranges_fail(annotation):
    with pytest.raises(ValueError, match="range-specific"):
        generator.parse_availability("output_int_register_X", annotation)


def test_legacy_range_requires_register_field():
    with pytest.raises(ValueError, match="range-specific"):
        generator.parse_availability("field", "[24..47] 5.3.0")


@pytest.mark.parametrize("direction", ["input", "output"])
def test_bit_register_expansion_inherits_metadata(direction):
    name = f"{direction}_bit_register_X"
    comment = "64 general purpose bits (X: [64..127]) reserved for external RTDE clients"
    table = make_table(
        ("timestamp", "DOUBLE", None, None),
        (name, "BOOL", comment, "5.23.0 / 10.11.0"),
        ("tool_output_voltage_2", "INT32", "Tool Flange V2", "10.15"),
        ("old_x_only", "DOUBLE", None, "10.7"),
    )
    records = generator.documentation_records(table)
    assert records == [
        ("timestamp", "*", "*"),
        *[(name[:-1] + str(i), "5.23.0", "10.11.0") for i in range(64, 128)],
        ("tool_output_voltage_2", "-", "10.15.0"),
        ("old_x_only", "-", "*"),
    ]


def test_comments_do_not_supply_introduction_versions():
    table = make_table(("actual_TCP_acceleration", "VECTOR6D", "5.26.0 / 10.12.0 in comment only", None))
    assert generator.documentation_records(table) == [("actual_TCP_acceleration", "*", "*")]


@pytest.mark.parametrize("name, comment", [
    ("", ""), ("field\tbad", ""), ("field bad", ""), ("field\nbad", ""),
    ("unknown_X", "1 general purpose [0..0]"),
    ("output_int_register_X", None),
    ("output_int_register_X", "48 registers [0..47]"),
    ("output_int_register_X", "48 general purpose registers"),
    ("output_int_register_X", "48 general purpose registers [0..46]"),
    ("output_int_register_X", "48 general purpose registers [0..23] [25..48]"),
    ("output_int_register_X", "48 general purpose registers [0..24] [24..47]"),
    ("output_int_register_X", "48 general purpose registers [47..0]"),
    ("output_int_register_X", "0 general purpose registers [0..0]"),
])
def test_invalid_names_and_register_descriptions(name, comment):
    with pytest.raises(ValueError):
        generator.expand_register_names(name, comment)


def test_duplicate_expanded_fields_fail():
    table = make_table(
        ("output_int_register_X", "INT32", "2 general purpose registers [0..1]", None),
        ("output_int_register_1", "INT32", "", None),
    )
    with pytest.raises(ValueError, match="Duplicate.*output_int_register_1"):
        generator.documentation_records(table)


def test_validate_table_and_columns():
    valid = make_table(("timestamp", "DOUBLE", "", None))
    assert generator.select_output_table([pd.DataFrame(), valid]) is valid
    for column in generator.REQUIRED_COLUMNS:
        with pytest.raises(ValueError, match="requires columns"):
            generator.documentation_records(valid.drop(columns=column))
    with pytest.raises(ValueError, match="requires columns"):
        generator.documentation_records(pd.concat([valid, valid[["Name"]]], axis=1))
    with pytest.raises(ValueError, match="empty"):
        generator.documentation_records(make_table())
    with pytest.raises(ValueError, match="DataFrame"):
        generator.documentation_records(None)
    with pytest.raises(ValueError, match="missing.*table"):
        generator.select_output_table([valid])
    with pytest.raises(ValueError, match="missing timestamp"):
        generator.select_output_table([valid, make_table(("input", "INT32", "", None))])


def test_html_column_contract_offline():
    html = """<table><tr><th>Name</th><th>Type</th><th>Comment</th><th>Introduced in Version</th></tr>
    <tr><td>timestamp</td><td>DOUBLE</td><td>Time elapsed</td><td></td></tr>
    <tr><td>tool_output_current_2</td><td>DOUBLE</td><td>Tool V2</td><td>10.15</td></tr>
    <tr><td>at_cutoff</td><td>DOUBLE</td><td></td><td>10.10</td></tr></table>"""
    table = pd.read_html(io.StringIO(html), flavor="lxml", converters={generator.VERSION_COLUMN: str})[0]
    assert generator.documentation_records(table) == [
        ("timestamp", "*", "*"), ("tool_output_current_2", "-", "10.15.0"),
        ("at_cutoff", "-", "10.10.0"),
    ]


SOURCE = '''// unrelated
{"input_only", value},
// INPUT / OUTPUT
{"timestamp", value},
// comment

// OUTPUT
{"library_only", value},
// NOT IN OFFICIAL DOCS
{"undocumented", value},
'''


def test_exhaustive_extraction_remains_independent():
    assert generator.extract_exhaustive_outputs(io.StringIO(SOURCE)) == ["timestamp\n", "library_only\n"]


def test_import_has_no_network_source_search_or_resource_io(monkeypatch):
    spec = importlib.util.spec_from_file_location("generator_import_safety", generator.__file__)
    module = importlib.util.module_from_spec(spec)

    def forbidden(*args, **kwargs):
        pytest.fail("Import must not fetch docs, discover sources, or open resource files")

    monkeypatch.setattr(sys, "dont_write_bytecode", True)
    monkeypatch.setattr(pd, "read_html", forbidden)
    monkeypatch.setattr(pathlib.Path, "glob", forbidden)
    monkeypatch.setattr(pathlib.Path, "open", forbidden)
    monkeypatch.setattr("builtins.open", forbidden)
    spec.loader.exec_module(module)
    assert callable(module.main)


def test_main_outputs_and_cli_guard(tmp_path, monkeypatch):
    source = tmp_path / "data_package.cpp"
    source.write_text(SOURCE)
    exhaustive = tmp_path / "exhaustive.txt"
    docs = tmp_path / "docs.txt"
    monkeypatch.setattr(generator, "URCL_PATH", tmp_path)
    monkeypatch.setattr(generator, "OUTPUT_PATH", exhaustive)
    monkeypatch.setattr(generator, "WEB_OUTPUT_PATH", docs)

    def read_html(url, converters):
        assert url == generator.DOCS_URL
        assert converters == {generator.VERSION_COLUMN: str}
        return [pd.DataFrame(), make_table(
            ("timestamp", "DOUBLE", "", None),
            ("docs_only", "DOUBLE", "", "10.7"),
            ("tool_output_voltage_2", "INT32", "", "10.15"),
        )]

    monkeypatch.setattr(pd, "read_html", read_html)
    generator.main()
    assert exhaustive.read_text() == "timestamp\nlibrary_only\n"
    assert docs.read_text() == "timestamp\t*\t*\ndocs_only\t-\t*\ntool_output_voltage_2\t-\t10.15.0\n"
    assert all(len(line.split("\t")) == 3 for line in docs.read_text().splitlines())
    # Exercise the unchanged no-argument script entry point without writing any
    # real repository resources. Path.open still reads only the temporary source.
    monkeypatch.setattr(pathlib.Path, "glob", lambda self, pattern: iter([source]))
    mocked_open = mock_open()
    monkeypatch.setattr("builtins.open", mocked_open)
    runpy.run_path(generator.__file__, run_name="__main__")
    assert mocked_open.call_count == 2


@pytest.mark.parametrize("source_count", [0, 2])
def test_main_requires_unique_source(tmp_path, monkeypatch, source_count):
    for index in range(source_count):
        folder = tmp_path / str(index)
        folder.mkdir()
        (folder / "data_package.cpp").write_text(SOURCE)
    monkeypatch.setattr(generator, "URCL_PATH", tmp_path)
    with pytest.raises(ValueError, match="Expected one data_package.cpp"):
        generator.main()