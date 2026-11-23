"""PlatformIO extra script: write a board's compile commands into its build folder.

check_includes.sh loads this script through PLATFORMIO_EXTRA_SCRIPTS for `pio run -t compiledb`,
with PLATFORMIO_BUILD_DIR=.pio/check-includes. PlatformIO then writes
$BUILD_DIR/compile_commands.json, which is .pio/check-includes/<board>/compile_commands.json, and
leaves a compile_commands.json of one's own in diffbot_firmware alone.
"""

import os

from SCons.Script import DefaultEnvironment

DefaultEnvironment().Replace(
    COMPILATIONDB_PATH=os.path.join('$BUILD_DIR', 'compile_commands.json'))
