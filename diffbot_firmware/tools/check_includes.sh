#!/usr/bin/env bash
# Checks that firmware files include the header of every name they use, with clang-tidy's check
# misc-include-cleaner and the compiler flags of one PlatformIO environment. It runs in the
# firmware container, in diffbot_firmware. From the repository's root:
#   .docker/firmware/run.sh tools/check_includes.sh native test/test_bno055/test_bno055.cpp
#   .docker/firmware/run.sh tools/check_includes.sh teensy31 src/main.cpp
# native is for the files that build on the PC: the libraries without Arduino, test/fakes and the
# test_* suites. teensy31 and teensy40 are for the board-only files: src/main.cpp, the libraries
# that use Arduino and the tests that run on a Teensy.
#
# Steps:
#   1. Check the arguments and that the environment's libraries are there. Install clang-tidy into
#      .pio/clang-tidy on the first run.
#   2. Collect the environment's compiler flags: for native the flags of the unit tests, for a
#      board the flags that PlatformIO compiles src/main.cpp with, and what the board's GCC knows
#      by itself and clang doesn't.
#   3. Run clang-tidy on each file, a header with -x c++-header in front of the flags.
#
# Exit status: 0 when clang-tidy could read every file. Its warnings don't change the status,
# because older files have some (AGENTS.md, "Check the includes", says which ones count). 1 when
# clang-tidy failed on a file or the flags couldn't be found, 2 for wrong arguments.
set -euo pipefail

readonly CLANG_TIDY_VERSION=18.1.8
readonly CLANG_TIDY_ENV=.pio/clang-tidy
readonly CLANG_TIDY=$CLANG_TIDY_ENV/bin/clang-tidy
readonly BOARD_GCC=/pio-core/packages/toolchain-gccarmnoneeabi-teensy/bin/arm-none-eabi-g++
# PlatformIO writes a board's compile commands here, in diffbot_firmware
readonly DATABASE=compile_commands.json
# The headers behind the entry headers unity.h, Arduino.h and Wire.h. A name from one of them
# counts as included when the file includes the entry header.
readonly IGNORED_HEADERS='unity.*;core_pins.h;WireKinetis.h;WireIMXRT.h'
readonly TIDY_OPTIONS=(
    --checks='-*,misc-include-cleaner'
    "--config={CheckOptions: {misc-include-cleaner.IgnoreHeaders: \"$IGNORED_HEADERS\"}}"
)
TOOLS_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
readonly TOOLS_DIR

# The flags for clang-tidy, which step 2 fills
compile_flags=()
# Where step 2 keeps a compile_commands.json that was there before, until the exit
saved_database=''

usage() {
    echo "Usage: $0 {native|teensy31|teensy40} FILE..."
    echo "Checks the includes of each FILE with clang-tidy's misc-include-cleaner, in diffbot_firmware."
}

# 1. The arguments, the environment's libraries and clang-tidy
check_arguments() {
    local env=$1 file
    shift
    case "$env" in
        native | teensy31 | teensy40) ;;
        *)
            echo "error: unknown environment: $env" >&2
            usage >&2
            exit 2
            ;;
    esac
    if [ "$#" -eq 0 ]; then
        echo "error: name at least one file to check" >&2
        exit 2
    fi
    for file in "$@"; do
        if [ ! -f "$file" ]; then
            echo "error: no such file in diffbot_firmware: $file" >&2
            exit 2
        fi
    done
    if [ ! -d ".pio/libdeps/$env" ]; then
        echo "error: .pio/libdeps/$env is missing. Build the board or run the unit tests first:" >&2
        echo "  pio run -e teensy31, pio run -e teensy40 or pio test -e native" >&2
        exit 2
    fi
}

install_clang_tidy() {
    if [ -x "$CLANG_TIDY" ]; then
        return
    fi
    echo "Installing clang-tidy $CLANG_TIDY_VERSION into $CLANG_TIDY_ENV (once)"
    python3 -m venv "$CLANG_TIDY_ENV"
    "$CLANG_TIDY_ENV/bin/pip" install --quiet "clang-tidy==$CLANG_TIDY_VERSION"
}

# 2. The compiler flags of the unit tests on the PC: the native environment's
native_flags() {
    local dir
    compile_flags=(-std=c++17 -Wall -DUNIT_TEST -Itest -I.pio/libdeps/native/Unity/src
                   '-I.pio/libdeps/native/Rosserial Arduino Library/src')
    for dir in lib/*/; do
        compile_flags+=("-I${dir%/}")
    done
}

# Puts back the compile_commands.json that was there before step 2, or removes PlatformIO's.
# The compile commands aren't committed.
restore_database() {
    rm -f "$DATABASE"
    if [ -n "$saved_database" ]; then
        mv "$saved_database" "$DATABASE"
    fi
}

# 2. The compiler flags of a board
board_flags() {
    local board=$1 flags version multilib sysroot cxx
    # 2.1 PlatformIO's compile commands for the board, written anew. A compile_commands.json
    #     that was there before comes back at the exit, also after an error.
    if [ -e "$DATABASE" ]; then
        saved_database=$(mktemp)
        mv "$DATABASE" "$saved_database"
    fi
    trap restore_database EXIT
    if ! pio run -e "$board" -t compiledb > /dev/null; then
        echo "error: PlatformIO wrote no compile commands for $board" >&2
        exit 1
    fi
    # 2.2 The flags of the command for src/main.cpp that clang understands (board_flags.py)
    if ! flags=$(python3 "$TOOLS_DIR/board_flags.py" "$DATABASE"); then
        exit 1
    fi
    mapfile -t compile_flags <<< "$flags"
    # 2.3 What the board's GCC knows by itself and clang doesn't: the target, two integer types
    #     and the folders of the toolchain's C and C++ headers. int32_t is long for the board's
    #     GCC and int for clang. Without the two definitions, clang stops at two declarations of
    #     random() that then differ in their return type.
    if ! version=$("$BOARD_GCC" -dumpfullversion) ||
       ! multilib=$("$BOARD_GCC" "${compile_flags[@]}" -print-multi-directory); then
        echo "error: the toolchain's GCC didn't answer: $BOARD_GCC" >&2
        exit 1
    fi
    sysroot=$(dirname "$(dirname "$BOARD_GCC")")/arm-none-eabi
    cxx=$sysroot/include/c++/$version
    compile_flags+=(--target=arm-none-eabi
                    -D__INT32_TYPE__='long int' -D__UINT32_TYPE__='long unsigned int'
                    -isystem "$cxx" -isystem "$cxx/arm-none-eabi/$multilib"
                    -isystem "$sysroot/include")
    # 2.4 Unity, for the tests that run on a Teensy: its folder, and without the configuration
    #     header that PlatformIO writes only when it builds such a test
    compile_flags+=("-I.pio/libdeps/$board/Unity/src" -UUNITY_INCLUDE_CONFIG_H)
}

# 3. clang-tidy on each file. The status is 1 if it failed on one of them.
check_files() {
    local env=$1 file status=0
    shift
    for file in "$@"; do
        echo "== $file ($env)"
        case "$file" in
            *.h | *.hpp)
                "$CLANG_TIDY" "${TIDY_OPTIONS[@]}" "$file" -- -x c++-header "${compile_flags[@]}" ||
                    status=1
                ;;
            *)
                "$CLANG_TIDY" "${TIDY_OPTIONS[@]}" "$file" -- "${compile_flags[@]}" || status=1
                ;;
        esac
    done
    return "$status"
}

main() {
    if [ "$#" -eq 0 ]; then
        usage >&2
        exit 2
    fi
    if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
        usage
        exit 0
    fi
    local env=$1
    shift
    check_arguments "$env" "$@"
    install_clang_tidy
    if [ "$env" = native ]; then
        native_flags
    else
        board_flags "$env"
    fi
    check_files "$env" "$@"
}

main "$@"
