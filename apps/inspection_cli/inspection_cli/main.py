import argparse
from inspection_runtime.commands import COMMANDS, dispatch

def main(argv=None):
    parser = argparse.ArgumentParser(description="Inspection commands; use a command to invoke the existing implementation.")
    parser.add_argument("command", choices=sorted(COMMANDS))
    parser.add_argument("arguments", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    return dispatch(args.command, args.arguments)
