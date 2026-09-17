#!/usr/bin/env python3
"""Rewrite JUnit XML into the shape Codecov Test Analytics documents.

CTest --output-junit emits a bare <testsuite name="(empty)"> with <properties/>
and <system-out> on every case. Codecov only documents JUnit XML, and its
parser example is <testsuites><testsuite><testcase classname name time/>.
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def _local(tag: str | None) -> str:
    if not tag:
        return ""
    return tag.rsplit("}", 1)[-1]


def _first(root: ET.Element, name: str) -> ET.Element | None:
    for el in root.iter():
        if _local(el.tag) == name:
            return el
    return None


def _as_float(value: str | None) -> float:
    try:
        return float(value) if value not in (None, "") else 0.0
    except ValueError:
        return 0.0


def _duration(root: ET.Element) -> str:
    # Prefer the document aggregate (testsuites or a bare CTest testsuite).
    if root.get("time") not in (None, ""):
        return root.get("time") or "0"
    total = sum(
        _as_float(child.get("time"))
        for child in root
        if _local(child.tag) == "testsuite"
    )
    return f"{total:g}" if total else "0"


def _cases(root: ET.Element) -> list[ET.Element]:
    return [el for el in root.iter() if _local(el.tag) == "testcase"]


def _split_name(classname: str, name: str) -> tuple[str, str]:
    if classname == name and "." in name:
        fixture, _, test = name.rpartition(".")
        return fixture, test
    return classname, name


def rewrite(src: Path, dst: Path, suite: str) -> None:
    orig = ET.parse(src).getroot()
    cases = _cases(orig)
    ts_orig = _first(orig, "testsuite")
    time_s = _duration(orig)
    timestamp = ts_orig.get("timestamp", "") if ts_orig is not None else ""

    failures = 0
    skipped = 0
    errors = 0
    new_cases: list[ET.Element] = []

    for case in cases:
        classname, name = _split_name(case.get("classname", ""), case.get("name", ""))
        tc = ET.Element(
            "testcase",
            {
                "classname": classname,
                "name": name,
                "time": case.get("time", "0"),
            },
        )
        failure_el = None
        skipped_el = None
        is_error = False
        sysout: list[str] = []
        for child in case:
            kind = _local(child.tag)
            if kind == "failure":
                failure_el = child
            elif kind == "error":
                failure_el = child
                is_error = True
            elif kind == "skipped":
                skipped_el = child
            elif kind == "system-out":
                sysout.append("".join(child.itertext()))

        if failure_el is not None:
            if is_error:
                errors += 1
            else:
                failures += 1
            tag = "error" if is_error else "failure"
            msg = failure_el.get("message") or ("Error" if is_error else "Failed")
            text = (failure_el.text or "").strip() or "\n".join(sysout).strip()
            node = ET.SubElement(tc, tag, {"message": msg})
            if text:
                node.text = text
        elif skipped_el is not None:
            skipped += 1
            ET.SubElement(
                tc, "skipped", {"message": skipped_el.get("message") or "skipped"}
            )
        new_cases.append(tc)

    testsuites = ET.Element(
        "testsuites",
        {
            "name": suite,
            "tests": str(len(cases)),
            "failures": str(failures),
            "errors": str(errors),
            "time": str(time_s),
        },
    )
    ts_attrs = {
        "name": suite,
        "errors": str(errors),
        "failures": str(failures),
        "skipped": str(skipped),
        "time": str(time_s),
        "tests": str(len(cases)),
    }
    if timestamp:
        ts_attrs["timestamp"] = timestamp
    testsuite = ET.SubElement(testsuites, "testsuite", ts_attrs)
    testsuite.extend(new_cases)

    tree = ET.ElementTree(testsuites)
    ET.indent(tree, space="  ")
    dst.parent.mkdir(parents=True, exist_ok=True)
    tree.write(dst, encoding="utf-8", xml_declaration=True)


def main(argv: list[str]) -> int:
    if len(argv) < 3:
        print(
            "usage: prepare_junit_for_codecov.py <input.xml> <output.junit.xml> [suite-name]",
            file=sys.stderr,
        )
        return 2
    src = Path(argv[1])
    dst = Path(argv[2])
    suite = argv[3] if len(argv) > 3 else "ctest"
    if not src.is_file():
        print(f"missing junit file: {src}", file=sys.stderr)
        return 1
    if not dst.name.endswith("junit.xml"):
        print("output must be named *junit.xml (Codecov Test Analytics glob)", file=sys.stderr)
        return 1
    rewrite(src, dst, suite)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
