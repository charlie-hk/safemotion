"""Error type with stable codes (SM-E..). (c) 2026 Ali Amini"""


class SafeMotionError(ValueError):
    def __init__(self, code: str, message: str):
        super().__init__(f"{code}: {message}")
        self.code = code
