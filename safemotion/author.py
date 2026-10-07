"""Author notice with a tamper-evident check. It detects an edited notice; it cannot stop anyone editing source.
(c) 2026 Ali Amini"""
import hashlib

AUTHOR = {"product": "safemotion", "name": "Ali Amini", "year": 2026}
EXPECTED_SIGNATURE = "253588cb6f80f5d5ea498b1fc77123ec7ae9cb31a3a96c11ead2c5e3390b9513"


def signature(author=None) -> str:
    a = author or AUTHOR
    return hashlib.sha256(f"{a['product']}|{a['name']}|{a['year']}|sm1".encode()).hexdigest()


def verify(author=None, expected=EXPECTED_SIGNATURE) -> bool:
    return signature(author) == expected


def notice(author=None, expected=EXPECTED_SIGNATURE) -> str:
    a = author or AUTHOR
    if verify(a, expected):
        return f"{a['product']} (c) {a['year']} {a['name']}"
    return f"{a['product']} (modified copy: the author notice was altered)"
