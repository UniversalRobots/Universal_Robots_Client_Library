"""Offline tests: python -B -m unittest discover -s tests/resources -v."""

from io import StringIO
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import generate_rtde_outputs as updater


SERVER = '''// unrelated code
const std::unordered_map<std::string, std::string> VARIABLE_TYPES{
  // INPUTS
  { "speed_slider_mask", "UINT32" },

  // INPUT / OUTPUT
  { "input_int_register_0", "INT32" },

  // OUTPUTS
  { "timestamp", "DOUBLE" },
  { "old_field", "UINT32" },

  // NOT IN OFFICIAL DOCS
  { "special_field", "BOOL" },
};
// more unrelated code
'''


def documentation(rows) -> StringIO:
    """Make a small page with inputs first and outputs second, like the website."""
    output_rows = "".join(
        f"<tr><td>{name}</td><td>{data_type}</td><td>{comment}</td></tr>"
        for name, data_type, comment in rows
    )
    return StringIO(f'''
        <table><tr><th>Name</th></tr><tr><td>input_only</td></tr></table>
        <table>
          <tr><th>Name</th><th>Type</th><th>Comment</th></tr>
          {output_rows}
        </table>
    ''')


class ExtractFieldsTests(unittest.TestCase):
    @patch.object(updater, "MINIMUM_FIELDS", 1)
    def test_extract_names_types_and_registers(self) -> None:
        page = documentation([
            ("timestamp", "DOUBLE", "Time"),
            ("output_bit_registers0_to_31", "UINT32", "Packed bits"),
            ("output_bit_register_X", "BOOL", "2 general purpose bits (X: [64..65])"),
            ("input_int_register_X", "INT32",
             "4 general purpose integer registers (X: [0..1] reserved, [2..3] external)"),
        ])

        fields = updater.extract_fields(page)

        self.assertEqual(fields, {
            "timestamp": "DOUBLE",
            "output_bit_registers0_to_31": "UINT32",
            "output_bit_register_64": "BOOL",
            "output_bit_register_65": "BOOL",
            "input_int_register_0": "INT32",
            "input_int_register_1": "INT32",
            "input_int_register_2": "INT32",
            "input_int_register_3": "INT32",
        })

    def test_reject_incomplete_documentation(self) -> None:
        page = documentation([("timestamp", "DOUBLE", "Time")])

        with self.assertRaises(ValueError):
            updater.extract_fields(page)

    def test_reject_invalid_field_definitions(self) -> None:
        for name, data_type in (("bad-name", "DOUBLE"), ("timestamp", "UNKNOWN")):
            with self.subTest(name=name), self.assertRaises(ValueError):
                updater.validate_and_add({}, name, data_type)

        with self.assertRaises(ValueError):
            updater.validate_and_add({"timestamp": "DOUBLE"}, "timestamp", "DOUBLE")


class UpdateServerTests(unittest.TestCase):
    def test_add_new_field(self) -> None:
        updated, fields, changes = updater.update_server(SERVER, {"new_field": "BOOL"})

        expected = SERVER.replace(
            '\n' + updater.UNDOCUMENTED_MARKER,
            '  { "new_field", "BOOL" },\n\n' + updater.UNDOCUMENTED_MARKER,
        )
        self.assertEqual(updated, expected)
        self.assertEqual(fields["new_field"], "BOOL")
        self.assertEqual(changes, ["Added new_field: BOOL"])

    def test_update_existing_types(self) -> None:
        documented = {"timestamp": "UINT64", "input_int_register_0": "DOUBLE"}

        updated, fields, changes = updater.update_server(SERVER, documented)

        expected = SERVER.replace('"timestamp", "DOUBLE"', '"timestamp", "UINT64"')
        expected = expected.replace('"input_int_register_0", "INT32"',
                                    '"input_int_register_0", "DOUBLE"')
        self.assertEqual(updated, expected)
        self.assertEqual(fields["timestamp"], "UINT64")
        self.assertEqual(fields["input_int_register_0"], "DOUBLE")
        self.assertEqual(len(changes), 2)

    def test_preserve_missing_fields(self) -> None:
        updated, fields, changes = updater.update_server(SERVER, {"timestamp": "DOUBLE"})

        self.assertEqual(updated, SERVER)
        self.assertEqual(fields["old_field"], "UINT32")
        self.assertEqual(changes, [])

    def test_reject_changes_to_protected_entries(self) -> None:
        for name in ("speed_slider_mask", "special_field"):
            with self.subTest(name=name), self.assertRaises(ValueError):
                updater.update_server(SERVER, {name: "BOOL"})

    def test_accept_spaces_and_tabs(self) -> None:
        for line in (
            '{"timestamp","DOUBLE"},',
            '  { "timestamp", "DOUBLE" },',
            '\t{\t"timestamp"\t,\t"DOUBLE"\t},\t',
        ):
            with self.subTest(line=line):
                source = SERVER.replace('  { "timestamp", "DOUBLE" },', line)

                updated, _, _ = updater.update_server(source, {"timestamp": "UINT64"})

                self.assertEqual(updated, source.replace('"DOUBLE"', '"UINT64"'))

    def test_second_run_changes_nothing(self) -> None:
        documented = {"new_field": "BOOL", "timestamp": "UINT64"}
        first, first_fields, _ = updater.update_server(SERVER, documented)

        second, second_fields, changes = updater.update_server(first, documented)

        self.assertEqual(second, first)
        self.assertEqual(second_fields, first_fields)
        self.assertEqual(changes, [])


@patch.object(updater, "MINIMUM_FIELDS", 1)
class WriteResourcesTests(unittest.TestCase):
    def test_write_map_and_recipes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            server = Path(directory) / "server.cpp"
            recipe = Path(directory) / "recipe.txt"
            docs_recipe = Path(directory) / "docs_recipe.txt"
            server.write_text(SERVER)
            page = documentation([("timestamp", "UINT64", "Time")])

            updater.update_resources(page, server, recipe, docs_recipe)

            self.assertEqual(server.read_text(),
                             SERVER.replace('"timestamp", "DOUBLE"', '"timestamp", "UINT64"'))
            self.assertEqual(recipe.read_text(),
                             "input_int_register_0\ntimestamp\nold_field\n")
            self.assertEqual(docs_recipe.read_text(), "timestamp\n")

    def test_invalid_documentation_leaves_files_unchanged(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            server = Path(directory) / "server.cpp"
            recipe = Path(directory) / "recipe.txt"
            docs_recipe = Path(directory) / "docs_recipe.txt"
            server.write_text(SERVER)
            recipe.write_text("existing recipe\n")
            docs_recipe.write_text("existing documentation recipe\n")
            page = StringIO("<html>No tables</html>")

            with self.assertRaises(ValueError):
                updater.update_resources(page, server, recipe, docs_recipe)

            self.assertEqual(server.read_text(), SERVER)
            self.assertEqual(recipe.read_text(), "existing recipe\n")
            self.assertEqual(docs_recipe.read_text(), "existing documentation recipe\n")


if __name__ == "__main__":
    unittest.main()
