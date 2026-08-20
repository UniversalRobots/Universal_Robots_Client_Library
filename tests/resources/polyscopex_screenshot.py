#!/usr/bin/env python3
"""Take a screenshot of the PolyScope X UI with a real post-load delay for SPA rendering.

Usage: polyscopex_screenshot.py <url> <output_file> <delay_ms>
"""
import sys

from playwright.sync_api import sync_playwright


def main():
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <url> <output_file> <delay_ms>")
        sys.exit(1)

    url, output_file, delay_ms = sys.argv[1], sys.argv[2], int(sys.argv[3])

    with sync_playwright() as p:
        browser = p.chromium.launch(args=["--no-sandbox", "--disable-setuid-sandbox"])
        page = browser.new_page(viewport={"width": 1920, "height": 1080})
        page.goto(url, wait_until="load")
        page.wait_for_timeout(delay_ms)
        page.screenshot(path=output_file)
        browser.close()


if __name__ == "__main__":
    main()
