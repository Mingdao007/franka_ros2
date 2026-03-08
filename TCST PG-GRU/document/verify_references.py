#!/usr/bin/env python3
from __future__ import annotations

import re
import subprocess
from pathlib import Path

BIB_PATH = Path(__file__).resolve().parent / "references.bib"


def extract_dois(text: str) -> list[str]:
    pattern = re.compile(r"doi\s*=\s*\{([^}]+)\}", flags=re.IGNORECASE)
    return sorted(set(m.group(1).strip() for m in pattern.finditer(text)))


def check_doi(doi: str) -> tuple[bool, str]:
    url = f"https://doi.org/{doi}"
    proc = subprocess.run(
        ["curl", "-sI", url],
        check=False,
        capture_output=True,
        text=True,
    )
    head = proc.stdout.splitlines()
    status = head[0] if head else "NO_STATUS"
    ok = " 302 " in status or " 301 " in status or " 200 " in status
    return ok, status


def main() -> None:
    text = BIB_PATH.read_text(encoding="utf-8")
    dois = extract_dois(text)
    if not dois:
        raise SystemExit("No DOI found in references.bib")

    failed = []
    for doi in dois:
        ok, status = check_doi(doi)
        print(f"{doi}\t{status}")
        if not ok:
            failed.append(doi)

    if failed:
        raise SystemExit(f"Failed DOI checks: {failed}")


if __name__ == "__main__":
    main()
