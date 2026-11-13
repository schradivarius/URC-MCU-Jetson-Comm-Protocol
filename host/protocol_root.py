import os

ENV_VAR = "ROVER_PROTOCOL_ROOT"

DEFAULT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def protocol_root():
    override = os.environ.get(ENV_VAR)
    if not override:
        return DEFAULT_ROOT
    return os.path.abspath(os.path.expanduser(override))


def firmware_src(*parts):
    return os.path.join(protocol_root(), "firmware", "src", *parts)


def tools(*parts):
    return os.path.join(protocol_root(), "tools", *parts)


def build_dir():
    return os.path.join(protocol_root(), "tests", "cpp", "build")


def is_overridden():
    return bool(os.environ.get(ENV_VAR))
