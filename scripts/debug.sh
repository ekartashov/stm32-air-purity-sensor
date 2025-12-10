#!/bin/bash
# STM32 Project Debugging Script (real ST-LINK + GDB, no mocking)
#
# This script:
#   - Uses the STM32 VS Code bundles (gnu-tools-for-stm32, stlink-gdbserver, programmer)
#   - Can flash the ELF via STM32_Programmer_CLI
#   - Can start ST-LINK_gdbserver and arm-none-eabi-gdb
#   - Still provides simple UART monitoring

set -euo pipefail

# ---------------------------------------------------------------------------
# Paths & defaults
# ---------------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Adjust if your ELF lives elsewhere / has another name.
ELF_REL_PATH="build/Debug/air-purity-sensor-stwink.elf"

DEFAULT_BAUD_RATE=115200
DEFAULT_UART_PORT=/dev/ttyACM1
DEFAULT_GDB_PORT=3333

BAUD_RATE="$DEFAULT_BAUD_RATE"
UART_PORT="$DEFAULT_UART_PORT"
GDB_PORT="$DEFAULT_GDB_PORT"
ACTION=""

# Cache for tool paths (filled lazily)
BUNDLES_ROOT=""
ARM_GDB=""
STLINK_GDBSERVER=""
STM32_PROGRAMMER_CLI=""
STM32_PROGRAMMER_BIN_DIR=""

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

usage() {
    cat <<EOF
Usage: $0 [options]

Options:
  -h, --help              Show this help message
  -b, --baud <rate>       Set UART baud rate (default: $DEFAULT_BAUD_RATE)
  -p, --port <port>       Set UART port (default: $DEFAULT_UART_PORT)
      --gdb-port <port>   Set GDB server port (default: $DEFAULT_GDB_PORT)

Actions (exactly one required):
      --upload-only       Flash ELF to target via ST-LINK (STM32_Programmer_CLI)
      --verify-upload     Check ST-LINK presence and ELF existence
      --gdb-server        Start ST-LINK GDB server only
      --gdb-session       Start GDB and attach to running GDB server
      --debug             Full flow: build-check + GDB server + GDB
      --monitor           UART console (minicom/screen)
      --both              Flash + debug + UART monitor

Typical flows:
  # Just flash firmware (no debug)
  $0 --upload-only

  # Start full interactive debug session (GDB)
  $0 --debug

  # For scripting/agent usage:
  #   1. Start server:
  $0 --gdb-server --gdb-port 3333
  #   2. In another process, use arm-none-eabi-gdb -batch ...
EOF
}

die() {
    echo "ERROR: $*" >&2
    exit 1
}

log() {
    echo "[debug.sh] $*"
}

# ---------------------------------------------------------------------------
# CLI parsing
# ---------------------------------------------------------------------------

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help) usage; exit 0 ;;
        -b|--baud) BAUD_RATE="$2"; shift 2 ;;
        -p|--port) UART_PORT="$2"; shift 2 ;;
        --gdb-port) GDB_PORT="$2"; shift 2 ;;
        --upload-only|--verify-upload|--gdb-server|--gdb-session|--debug|--monitor|--both)
            ACTION="${1#--}"
            shift
            ;;
        *)
            die "Unknown option: $1"
            ;;
    esac
done

if [[ -z "$ACTION" ]]; then
    die "No action specified. Use --debug, --upload-only, --gdb-server, --gdb-session, --monitor, --verify-upload or --both."
fi

cd "$PROJECT_ROOT"

ELF_PATH="$PROJECT_ROOT/$ELF_REL_PATH"

# ---------------------------------------------------------------------------
# Tool discovery (STM32 VS Code bundles)
# ---------------------------------------------------------------------------

get_bundles_root() {
    if [[ -n "${CUBE_BUNDLE_PATH:-}" ]]; then
        BUNDLES_ROOT="$CUBE_BUNDLE_PATH"
    else
        BUNDLES_ROOT="$HOME/.local/share/stm32cube/bundles"
    fi

    if [[ ! -d "$BUNDLES_ROOT" ]]; then
        die "STM32 bundles directory not found. Set CUBE_BUNDLE_PATH or install STM32Cube for VS Code bundles."
    fi
}

find_arm_gdb() {
    [[ -n "$ARM_GDB" ]] && return 0
    get_bundles_root

    # Prefer gnu-tools-for-stm32
    if [[ -d "$BUNDLES_ROOT/gnu-tools-for-stm32" ]]; then
        local cand
        cand="$(find "$BUNDLES_ROOT/gnu-tools-for-stm32" -maxdepth 4 -type f -name 'arm-none-eabi-gdb' 2>/dev/null | head -n1 || true)"
        if [[ -n "$cand" ]]; then
            ARM_GDB="$cand"
            return 0
        fi
    fi

    # Fallback: search entire bundles tree
    local cand
    cand="$(find "$BUNDLES_ROOT" -maxdepth 6 -type f -name 'arm-none-eabi-gdb' 2>/dev/null | head -n1 || true)"
    if [[ -n "$cand" ]]; then
        ARM_GDB="$cand"
        return 0
    fi

    # Last resort: PATH
    if command -v arm-none-eabi-gdb >/dev/null 2>&1; then
        ARM_GDB="$(command -v arm-none-eabi-gdb)"
        return 0
    fi

    die "arm-none-eabi-gdb not found in STM32 bundles or PATH."
}

find_stlink_gdbserver() {
    [[ -n "$STLINK_GDBSERVER" ]] && return 0
    get_bundles_root

    local cand
    if [[ -d "$BUNDLES_ROOT/stlink-gdbserver" ]]; then
        cand="$(find "$BUNDLES_ROOT/stlink-gdbserver" -maxdepth 4 -type f \( -name 'ST-LINK_gdbserver*' -o -name 'stlink-gdbserver*' \) 2>/dev/null | head -n1 || true)"
    else
        cand="$(find "$BUNDLES_ROOT" -maxdepth 6 -type f \( -name 'ST-LINK_gdbserver*' -o -name 'stlink-gdbserver*' \) 2>/dev/null | head -n1 || true)"
    fi

    if [[ -n "$cand" ]]; then
        STLINK_GDBSERVER="$cand"
        return 0
    fi

    # Maybe it is in PATH
    if command -v ST-LINK_gdbserver >/dev/null 2>&1; then
        STLINK_GDBSERVER="$(command -v ST-LINK_gdbserver)"
        return 0
    fi
    if command -v stlink-gdbserver >/dev/null 2>&1; then
        STLINK_GDBSERVER="$(command -v stlink-gdbserver)"
        return 0
    fi

    die "ST-LINK GDB server not found (stlink-gdbserver bundle missing?)."
}

find_stm32_programmer_cli() {
    [[ -n "$STM32_PROGRAMMER_CLI" ]] && return 0
    get_bundles_root

    local cli
    if [[ -d "$BUNDLES_ROOT/programmer" ]]; then
        cli="$(find "$BUNDLES_ROOT/programmer" -maxdepth 6 -type f -name 'STM32_Programmer_CLI*' 2>/dev/null | head -n1 || true)"
    else
        cli="$(find "$BUNDLES_ROOT" -maxdepth 6 -type f -name 'STM32_Programmer_CLI*' 2>/dev/null | head -n1 || true)"
    fi

    if [[ -z "$cli" ]]; then
        die "STM32_Programmer_CLI not found in bundles. Install the 'programmer' bundle in STM32 VS Code extension."
    fi

    STM32_PROGRAMMER_CLI="$cli"
    STM32_PROGRAMMER_BIN_DIR="$(dirname "$cli")"
}

# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------

verify_build() {
    if [[ -f "$ELF_PATH" ]]; then
        log "ELF present: $ELF_PATH ($(stat -c%s "$ELF_PATH") bytes)"
    else
        die "ELF not found at: $ELF_PATH. Run your build first (e.g. ./scripts/build.sh)."
    fi
}

check_stlink_usb() {
    if command -v lsusb >/dev/null 2>&1; then
        if lsusb | grep -q "STMicroelectronics"; then
            log "ST-LINK device detected on USB."
            return 0
        else
            die "No STMicroelectronics device found via lsusb. Check USB cable / board power / ST-LINK driver."
        fi
    else
        log "lsusb not available; skipping USB presence check."
    fi
}

# ---------------------------------------------------------------------------
# UART monitor
# ---------------------------------------------------------------------------

start_uart_monitoring() {
    log "Starting UART monitor on $UART_PORT @ ${BAUD_RATE}..."
    if command -v minicom >/dev/null 2>&1; then
        log "Using minicom."
        sudo minicom -D "$UART_PORT" -b "$BAUD_RATE"
    elif command -v screen >/dev/null 2>&1; then
        log "Using screen."
        sudo screen "$UART_PORT" "$BAUD_RATE"
    else
        cat <<EOF
No minicom or screen found.

You can monitor manually with:
  sudo stty -F $UART_PORT $BAUD_RATE cs8 -cstopb -parenb -ixon -ixoff
  sudo cat $UART_PORT
EOF
        return 1
    fi
}

# ---------------------------------------------------------------------------
# Flashing via STM32_Programmer_CLI
# ---------------------------------------------------------------------------

flash_firmware() {
    verify_build
    find_stm32_programmer_cli
    check_stlink_usb

    log "Flashing ELF via STM32_Programmer_CLI..."
    log "  CLI: $STM32_PROGRAMMER_CLI"
    log "  ELF: $ELF_PATH"
    # 0x08000000 is typical flash base; adjust if your linker script uses a different origin.
    "$STM32_PROGRAMMER_CLI" -c port=SWD freq=4000 \
        -w "$ELF_PATH" 0x08000000 -rst
    log "Flash completed."
}

verify_upload_capability() {
    verify_build
    check_stlink_usb
    find_stm32_programmer_cli

    log "ST-LINK present and STM32_Programmer_CLI located at:"
    log "  $STM32_PROGRAMMER_CLI"
    log "Upload *should* work. Use --upload-only to actually program."
}

# ---------------------------------------------------------------------------
# Debug via ST-LINK_gdbserver + arm-none-eabi-gdb
# ---------------------------------------------------------------------------

create_gdb_init_if_missing() {
    local init_file="$PROJECT_ROOT/openocd_gdb_init.gdb"
    if [[ ! -f "$init_file" ]]; then
        cat > "$init_file" <<EOF
# Generic STM32 GDB init for ST-LINK_gdbserver
target remote localhost:$GDB_PORT
monitor reset halt
load
monitor reset halt
# At this point you can:
#   break main
#   continue
EOF
        log "Created GDB init file: $init_file"
    fi
}

start_gdb_server() {
    verify_build
    find_stlink_gdbserver
    find_stm32_programmer_cli
    check_stlink_usb

    log "Starting ST-LINK GDB server..."
    log "  Server : $STLINK_GDBSERVER"
    log "  Port   : $GDB_PORT"
    log "  -cp dir: $STM32_PROGRAMMER_BIN_DIR"

    "$STLINK_GDBSERVER" \
        -d -v \
        -p "$GDB_PORT" \
        -cp "$STM32_PROGRAMMER_BIN_DIR" &
    local pid=$!
    log "ST-LINK GDB server PID: $pid"
    log "Waiting a second for server to come up..."
    sleep 1
}

start_gdb_session() {
    verify_build
    find_arm_gdb
    create_gdb_init_if_missing

    local init_file="$PROJECT_ROOT/openocd_gdb_init.gdb"

    log "Starting arm-none-eabi-gdb:"
    log "  GDB : $ARM_GDB"
    log "  ELF : $ELF_PATH"
    log "  Init: $init_file"

    # Replace current shell with GDB; when GDB exits, script exits.
    exec "$ARM_GDB" -x "$init_file" "$ELF_PATH"
}

# ---------------------------------------------------------------------------
# Action dispatcher
# ---------------------------------------------------------------------------

case "$ACTION" in
    upload-only)
        flash_firmware
        ;;

    verify-upload)
        verify_upload_capability
        ;;

    gdb-server)
        start_gdb_server
        log "GDB server running on localhost:$GDB_PORT."
        log "In another terminal, run: arm-none-eabi-gdb $ELF_PATH"
        ;;

    gdb-session)
        # Assumes a GDB server is already running.
        start_gdb_session
        ;;

    debug)
        # Full interactive debug:
        #  - ensure ELF exists
        #  - start GDB server
        #  - start GDB and auto-connect/load
        start_gdb_server
        start_gdb_session
        ;;

    monitor)
        start_uart_monitoring
        ;;

    both)
        # Flash, start server+GDB, and keep UART monitor in a separate process.
        flash_firmware
        start_gdb_server
        # Start UART monitor in background, then GDB in foreground.
        start_uart_monitoring &
        start_gdb_session
        ;;

    *)
        die "Unhandled action: $ACTION"
        ;;
esac

log "Done."
