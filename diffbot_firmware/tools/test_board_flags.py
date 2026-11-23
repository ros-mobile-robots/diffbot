"""Unit tests of board_flags.py, which turns PlatformIO's compile command into clang's flags.

The firmware workflow runs them in the firmware container, in diffbot_firmware:
    python -m unittest discover -s tools
"""

import json
import unittest

from board_flags import BoardFlagsError
from board_flags import clang_flags
from board_flags import main_source_command

# A compile command of the shape that PlatformIO writes for a Teensy 3.2, shortened. The include
# folder of the rosserial library has spaces in its name, so the command quotes it.
TEENSY31_COMMAND = (
    'arm-none-eabi-g++ -o .pio/build/teensy31/src/main.cpp.o -c -fno-exceptions '
    '-felide-constructors -fsingle-precision-constant -fpermissive -Wno-psabi -mthumb '
    '-mcpu=cortex-m4 -DARDUINO_TEENSY32 -DF_CPU=72000000L -Iinclude -Isrc '
    '"-I.pio/libdeps/teensy31/Rosserial Arduino Library/src" src/main.cpp'
)


def database(*entries: dict) -> str:
    """Return a compilation database with the given entries as JSON text."""
    return json.dumps(list(entries))


class TestMainSourceCommand(unittest.TestCase):

    def test_main_source_command_returns_the_command_of_main_cpp_among_other_files(self) -> None:
        text = database(
            {'file': 'lib/pid/pid.cpp', 'command': 'arm-none-eabi-g++ -c lib/pid/pid.cpp'},
            {'file': 'src/main.cpp', 'command': TEENSY31_COMMAND},
        )
        self.assertEqual(main_source_command(text), TEENSY31_COMMAND)

    def test_main_source_command_raises_an_error_without_an_entry_for_main_cpp(self) -> None:
        text = database({'file': 'lib/pid/pid.cpp', 'command': 'arm-none-eabi-g++ -c pid.cpp'})
        with self.assertRaises(BoardFlagsError):
            main_source_command(text)

    def test_main_source_command_raises_an_error_for_text_that_isnt_json(self) -> None:
        with self.assertRaises(BoardFlagsError):
            main_source_command('compile_commands.json was cut off [')


class TestClangFlags(unittest.TestCase):

    def test_clang_flags_keep_a_quoted_include_folder_as_one_flag(self) -> None:
        flags = clang_flags(TEENSY31_COMMAND)
        self.assertIn('-I.pio/libdeps/teensy31/Rosserial Arduino Library/src', flags)

    def test_clang_flags_leave_out_the_compiler_the_source_and_the_object_file(self) -> None:
        flags = clang_flags(TEENSY31_COMMAND)
        for word in ('arm-none-eabi-g++', 'src/main.cpp', '-o',
                     '.pio/build/teensy31/src/main.cpp.o'):
            self.assertNotIn(word, flags, f'{word} is still among the flags')

    def test_clang_flags_leave_out_the_switches_that_only_gcc_knows(self) -> None:
        flags = clang_flags(TEENSY31_COMMAND)
        for switch in ('-c', '-felide-constructors', '-fsingle-precision-constant',
                       '-fpermissive', '-Wno-psabi'):
            self.assertNotIn(switch, flags, f'{switch} is still among the flags')

    def test_clang_flags_keep_the_processor_the_board_and_the_include_folders(self) -> None:
        flags = clang_flags(TEENSY31_COMMAND)
        for flag in ('-mthumb', '-mcpu=cortex-m4', '-DARDUINO_TEENSY32', '-Iinclude', '-Isrc'):
            self.assertIn(flag, flags, f'{flag} is missing')

    def test_clang_flags_raise_an_error_when_a_required_flag_is_missing(self) -> None:
        for missing in ('-mcpu=cortex-m4', '-DARDUINO_TEENSY32', '-Iinclude -Isrc '
                        '"-I.pio/libdeps/teensy31/Rosserial Arduino Library/src"'):
            command = TEENSY31_COMMAND.replace(missing + ' ', '')
            with self.assertRaises(BoardFlagsError, msg=f'without {missing}'):
                clang_flags(command)

    def test_clang_flags_raise_an_error_for_a_command_of_another_file(self) -> None:
        with self.assertRaises(BoardFlagsError):
            clang_flags('arm-none-eabi-g++ -mcpu=cortex-m4 -DARDUINO_TEENSY32 -Isrc -c pid.cpp')


if __name__ == '__main__':
    unittest.main()
