#!/usr/bin/env python3
"""Print the compiler flags of a Teensy build for clang-tidy, one flag per line.

check_includes.sh calls this module with PlatformIO's compilation database of one board. It takes
the command that compiles src/main.cpp, the file that includes the most of the board's headers,
and keeps the flags that clang understands: without the compiler, the source file, the object
file and the switches that only GCC knows.
"""

import json
import shlex
import sys
from typing import List

# The file whose command gives the board's flags
MAIN_SOURCE = 'src/main.cpp'
# Switches of the board's GCC that clang doesn't know or that only concern the object file
GCC_ONLY_FLAGS = frozenset({
    '-c', '-felide-constructors', '-fsingle-precision-constant', '-fpermissive',
    '-Wno-psabi', '-Wno-error=narrowing', '-Wno-maybe-uninitialized',
})
# Flags that every board's command has: the processor, the board's name and an include folder.
# Without one of them, the command isn't a Teensy build, and clang would check something else.
REQUIRED_FLAG_STARTS = ('-mcpu=', '-DARDUINO_TEENSY', '-I')


class BoardFlagsError(Exception):
    """The compilation database has no usable command for src/main.cpp."""


def main_source_command(database_text: str) -> str:
    """Return the command that compiles src/main.cpp in a compilation database."""
    try:
        entries = json.loads(database_text)
    except json.JSONDecodeError as error:
        raise BoardFlagsError(f"the compilation database isn't JSON: {error}") from error
    for entry in entries:
        if entry.get('file') == MAIN_SOURCE:
            return str(entry['command'])
    raise BoardFlagsError(f'the compilation database has no command for {MAIN_SOURCE}')


def clang_flags(command: str) -> List[str]:
    """Return the flags of a GCC command that clang-tidy can take for the same file."""
    words = shlex.split(command)
    if len(words) < 2 or not words[-1].endswith(MAIN_SOURCE):
        raise BoardFlagsError(f"the command doesn't end with {MAIN_SOURCE}")
    flags = words[1:-1]
    if '-o' in flags:
        output_at = flags.index('-o')
        del flags[output_at:output_at + 2]
    flags = [flag for flag in flags if flag not in GCC_ONLY_FLAGS]
    for start in REQUIRED_FLAG_STARTS:
        if not any(flag.startswith(start) for flag in flags):
            raise BoardFlagsError(f'the command for {MAIN_SOURCE} has no {start} flag')
    return flags


def main(argv: List[str]) -> int:
    """Print the flags from the compilation database that argv names, one per line."""
    if len(argv) != 2:
        print(f'usage: {argv[0]} COMPILE_COMMANDS_JSON', file=sys.stderr)
        return 2
    try:
        with open(argv[1], encoding='utf-8') as database:
            flags = clang_flags(main_source_command(database.read()))
    except (OSError, BoardFlagsError) as error:
        print(f'error: {error}', file=sys.stderr)
        return 1
    print('\n'.join(flags))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
