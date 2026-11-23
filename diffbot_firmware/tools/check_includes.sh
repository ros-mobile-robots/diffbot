#!/usr/bin/env bash
# Checks that firmware files include the header of every name they use, with clang-tidy's check
# misc-include-cleaner and the compiler flags of one PlatformIO environment. It runs in the
# firmware container, in diffbot_firmware. From the repository's root:
#   .docker/firmware/run.sh tools/check_includes.sh native test/test_bno055/test_bno055.cpp
#   .docker/firmware/run.sh tools/check_includes.sh native_googletest \
#     test/googletest/test_wire_bus/test_wire_bus.cpp
#   .docker/firmware/run.sh tools/check_includes.sh teensy31 src/main.cpp
# native is for the files that build on the PC: the libraries without Arduino, test/fakes and the
# Unity suites test_*. native_googletest is for the GoogleTest suites in test/googletest. teensy31
# and teensy40 are for the board-only files: src/main.cpp, the libraries that use Arduino and the
# tests that run on a Teensy.
#
# Steps:
#   1. Check the arguments and that the environment's libraries are there. Install clang-tidy into
#      .pio/clang-tidy on the first run, and again when the version there isn't the pinned one.
#   2. Collect the environment's compiler flags: for native and native_googletest the flags that
#      pio test compiles their suites with, for a board the flags that PlatformIO compiles
#      src/main.cpp with, and what the board's GCC knows by itself and clang doesn't.
#      PlatformIO writes the board's compile commands into a build folder of the check's own,
#      .pio/check-includes, so the board's build and a compile_commands.json of one's own stay
#      as they are.
#   3. Run clang-tidy on each file, with the test folders that PlatformIO adds for it (see
#      file_folders()). A header gets -x c++-header in front of the flags.
#
# Exit status: 0 when clang-tidy could read every file. Its warnings don't change the status,
# because older files have some (AGENTS.md, "Check the includes", says which ones count). 1 when
# clang-tidy failed on a file, couldn't be installed or the flags couldn't be found. 2 for wrong
# arguments and when the environment's library folder, .pio/libdeps/<environment>, is missing.
set -euo pipefail

readonly CLANG_TIDY_VERSION=18.1.8
readonly CLANG_TIDY_ENV=.pio/clang-tidy
readonly CLANG_TIDY=$CLANG_TIDY_ENV/bin/clang-tidy
readonly BOARD_GCC=/pio-core/packages/toolchain-gccarmnoneeabi-teensy/bin/arm-none-eabi-g++
# The check's own build folder. A build folder of its own keeps PlatformIO from cleaning the
# board's build in .pio/build, which it does when the build settings change.
readonly CHECK_BUILD_DIR=.pio/check-includes
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

usage() {
    echo "Usage: $0 {native|native_googletest|teensy31|teensy40} FILE..."
    echo "Checks the includes of each FILE with clang-tidy's misc-include-cleaner, in diffbot_firmware."
}

# 1. The arguments, the environment's libraries and clang-tidy
check_arguments() {
    local env=$1 file setup
    shift
    case "$env" in
        native | native_googletest | teensy31 | teensy40) ;;
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
        case "$env" in
            native*) setup="pio test -e $env" ;;
            *) setup="pio run -e $env" ;;
        esac
        echo "error: .pio/libdeps/$env is missing. $setup downloads its libraries." >&2
        exit 2
    fi
}

install_clang_tidy() {
    local installed=''
    if [ -x "$CLANG_TIDY" ]; then
        # The line "LLVM version 18.1.8" of clang-tidy --version
        installed=$("$CLANG_TIDY" --version 2> /dev/null | awk '/LLVM version/ {print $3}') ||
            installed=''
    fi
    if [ "$installed" = "$CLANG_TIDY_VERSION" ]; then
        return
    fi
    echo "Installing clang-tidy $CLANG_TIDY_VERSION into $CLANG_TIDY_ENV${installed:+, instead of $installed}"
    if ! python3 -m venv "$CLANG_TIDY_ENV" ||
       ! "$CLANG_TIDY_ENV/bin/pip" install --quiet --force-reinstall "clang-tidy==$CLANG_TIDY_VERSION"; then
        echo "error: clang-tidy $CLANG_TIDY_VERSION couldn't be installed into $CLANG_TIDY_ENV" >&2
        exit 1
    fi
}

# 2. The compiler flags of the unit tests on the PC, for the environment native or
#    native_googletest, as pio test compiles a suite: PlatformIO's definitions for a test build,
#    the project's folders, and the include folders of the environment's test libraries.
#    file_folders() adds the test folders for each file. For native, the flags leave out the
#    Unity configuration that pio test generates in .pio/build/native/unity_config.
pc_test_flags() {
    local env=$1 version major minor patch dir googletest
    # PlatformIO defines PLATFORMIO as its version in one number, 60200 for 6.2.0. The line
    # of pio --version is "PlatformIO Core, version 6.2.0".
    if ! version=$(pio --version | awk '{print $NF}') ||
       ! [[ $version =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        echo "error: PlatformIO's version couldn't be read" >&2
        exit 1
    fi
    IFS=. read -r major minor patch <<< "$version"
    compile_flags=(-std=c++17 -Wall "-DPLATFORMIO=$(printf '%d%02d%02d' "$major" "$minor" "$patch")"
                   -DPIO_UNIT_TESTING -DUNIT_TEST -Iinclude -Isrc)
    for dir in lib/*/; do
        compile_flags+=("-I${dir%/}")
    done
    if [ "$env" = native ]; then
        compile_flags+=(-I.pio/libdeps/native/Unity/src
                        '-I.pio/libdeps/native/Rosserial Arduino Library/src')
    else
        googletest=.pio/libdeps/native_googletest/googletest
        compile_flags+=("-I$googletest/googlemock/include" "-I$googletest/googletest/include")
    fi
}

# 2. The compiler flags of a board
board_flags() {
    local board=$1 database=$CHECK_BUILD_DIR/$1/compile_commands.json flags version multilib
    local sysroot cxx
    # 2.1 PlatformIO's compile commands for the board, written anew into the check's build folder.
    #     compiledb_path.py tells PlatformIO to write them there instead of into diffbot_firmware.
    rm -f "$database"
    if ! PLATFORMIO_BUILD_DIR=$CHECK_BUILD_DIR \
         PLATFORMIO_EXTRA_SCRIPTS="pre:$TOOLS_DIR/compiledb_path.py" \
         pio run -e "$board" -t compiledb > /dev/null; then
        echo "error: PlatformIO wrote no compile commands for $board" >&2
        exit 1
    fi
    # 2.2 The flags of the command for src/main.cpp that clang understands (board_flags.py)
    if ! flags=$(python3 "$TOOLS_DIR/board_flags.py" "$database"); then
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

# 3. The include folders for one file, one per line, for a board its own folder. For the PC
#    tests, as pio test adds them (ConfigureTestTarget() in PlatformIO 6.2.0's piotest.py), in
#    the order in which the compiler searches them. The test folder is the first folder named
#    test in the file's path, test itself for the firmware's files and
#    tools/include_check_samples/test for the samples. Below it:
#    - A file of a suite, whose suite is the innermost folder test_* below the test folder, such
#      as test/googletest/test_wire_bus/test_wire_bus.cpp: the suite's folder, then the
#      grouping folders above it up to the next enclosing suite's, the outermost first, such as
#      test/googletest, then the test folder. PlatformIO prepends each of them, which reverses
#      the order of the grouping folders. A header in the suite's folder comes before one of
#      the same name in test/googletest.
#    - A shared file in no suite, such as test/fakes/fake_bno055.h: the grouping folders down to
#      its own, the outermost first, then the test folder, as a suite below them gets them.
#    - A file outside the test folder: its own folder, then the project's test folder.
file_folders() {
    local env=$1 file=$2 root='' suite='' path='' part folder
    local -a parts above=()
    if [[ $env != native* ]]; then
        echo "-I$(dirname "$file")"
        return
    fi
    IFS=/ read -r -a parts <<< "$(dirname "$file")"
    for part in "${parts[@]}"; do
        path=${path:+$path/}$part
        if [ -z "$root" ]; then
            if [ "$part" = test ]; then
                root=$path
            fi
        elif [[ $part == test_* ]]; then
            suite=$path
        fi
    done
    if [ -z "$root" ]; then
        echo "-I$(dirname "$file")"
        echo "-Itest"
        return
    fi
    if [ -n "$suite" ]; then
        echo "-I$suite"
        folder=$(dirname "$suite")
    else
        folder=$(dirname "$file")
    fi
    while [ "$folder" != "$root" ] && [[ $(basename "$folder") != test_* ]]; do
        above=("-I$folder" "${above[@]}")
        folder=$(dirname "$folder")
    done
    if [ "${#above[@]}" -gt 0 ]; then
        printf '%s\n' "${above[@]}"
    fi
    echo "-I$root"
}

# 3. clang-tidy on each file. The status is 1 if it failed on one of them.
check_files() {
    local env=$1 file status=0
    local -a folders
    shift
    for file in "$@"; do
        echo "== $file ($env)"
        mapfile -t folders < <(file_folders "$env" "$file")
        case "$file" in
            *.h | *.hpp)
                "$CLANG_TIDY" "${TIDY_OPTIONS[@]}" "$file" -- -x c++-header "${compile_flags[@]}" \
                    "${folders[@]}" || status=1
                ;;
            *)
                "$CLANG_TIDY" "${TIDY_OPTIONS[@]}" "$file" -- "${compile_flags[@]}" \
                    "${folders[@]}" || status=1
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
    if [ "$env" = native ] || [ "$env" = native_googletest ]; then
        pc_test_flags "$env"
    else
        board_flags "$env"
    fi
    check_files "$env" "$@"
}

main "$@"
