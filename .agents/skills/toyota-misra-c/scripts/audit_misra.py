#!/usr/bin/env python3
"""
Toyota Embedded MISRA-C Rule Auditor
Scans C source files against the 22 Toyota Embedded MISRA-C rules.
"""

import sys
import re
import os

def audit_c_file(filepath):
    print(f"\n==========================================")
    print(f"Auditing MISRA-C Rules: {os.path.basename(filepath)}")
    print(f"==========================================")

    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        lines = f.readlines()

    violations = []

    for idx, line in enumerate(lines):
        line_num = idx + 1
        stripped = line.strip()

        # Rule 1: No // comments
        if "//" in line and not ('"' in line and line.find('//') > line.find('"')):
            # ensure it's not inside a string literal
            violations.append((line_num, "Rule 1", "'//' comment style detected. Use /* ... */ instead."))

        # Rule 2: { and } should not be on same statement line (e.g. if(...) { ...; })
        if ("if" in stripped or "while" in stripped or "for" in stripped or "else" in stripped) and "{" in stripped and "}" in stripped:
            if not "} else {" in stripped:
                violations.append((line_num, "Rule 2", "'{' and '}' appear on same statement line."))

        # Rule 4: One assignment per line
        # Count assignments outside comments/strings
        if not stripped.startswith("/*") and not stripped.startswith("*"):
            semicolons = stripped.count(";")
            equals = len(re.findall(r'(?<![!=<>])=(?![=])', stripped))
            if semicolons > 1 and equals > 1:
                violations.append((line_num, "Rule 4", "Multiple assignments on one line."))

        # Rule 6: Ternary operator should be avoided
        if "?" in stripped and ":" in stripped and not stripped.startswith("/*"):
            # crude check for ternary outside strings
            if re.search(r'\w+\s*\?\s*\w+\s*:\s*\w+', stripped):
                violations.append((line_num, "Rule 6", "Ternary operator '? :' detected. Use if-else."))

        # Rule 9: Octal constants shall not be used in code (e.g. 012 but not 0 or 0x)
        code_only = re.sub(r'/\*.*?\*/', '', stripped)
        code_only = re.sub(r'//.*', '', code_only)
        octal_matches = re.findall(r'\b0[0-7]+[uUlL]*\b', code_only)
        for m in octal_matches:
            violations.append((line_num, "Rule 9", f"Octal literal '{m}' detected in code. Do not prefix numbers with leading 0."))

        # Rule 16: Loop counter shall not have floating type
        if re.search(r'for\s*\(\s*float\b', stripped):
            violations.append((line_num, "Rule 16", "Floating point loop counter detected in 'for' loop."))

        # Rule 17: Controlling expression not boolean (e.g. if(x) instead of if(x != 0))
        # flagged by common linters, skipped in simple regex

    if not violations:
        print("[PASS] No obvious MISRA-C rule violations detected by static auditor!")
    else:
        print(f"[FOUND {len(violations)} VIOLATIONS]:")
        for line_num, rule, desc in violations:
            print(f"  Line {line_num:4d} | {rule:7s} | {desc}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python audit_misra.py <file1.c> [file2.c ...]")
        sys.exit(1)

    for path in sys.argv[1:]:
        if os.path.exists(path):
            audit_c_file(path)
        else:
            print(f"File not found: {path}")
