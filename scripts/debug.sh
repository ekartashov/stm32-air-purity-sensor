#!/bin/bash
# STM32 Project Debugging Script
# This script provides a comprehensive debugging environment for the air-purity-sensor-stwink project
# using the STM32 extension's stlink-gdbserver and supports UART monitoring.

set -e  # Exit on error

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Default configuration
DEFAULT_BAUD_RATE=115200
DEFAULT_UART_PORT=/dev/ttyACM1  # Common port for STM32 devices
DEFAULT_GDB_PORT=3333

# Function to display usage
usage() {
    echo "Usage: $0 [options]"
    echo
    echo "Options:"
    echo "  -h, --help                Show this help message"
    echo "  -b, --baud <rate>        Set UART baud rate (default: $DEFAULT_BAUD_RATE)"
    echo "  -p, --port <port>        Set UART port (default: $DEFAULT_UART_PORT)"
    echo "  --gdb-port <port>        Set GDB server port (default: $DEFAULT_GDB_PORT)"
    echo "  --debug                  Start GDB debugging session"
    echo "  --monitor                Start UART monitoring"
    echo "  --both                   Start both GDB debugging and UART monitoring"
    echo
    echo "Examples:"
    echo "  $0 --debug                Start GDB debugging session"
    echo "  $0 --monitor              Start UART monitoring"
    echo "  $0 --both                 Start both debugging and monitoring"
    echo "  $0 --baud 9600 --port /dev/ttyUSB1 --debug  Custom configuration"
}

# Parse arguments
BAUD_RATE=$DEFAULT_BAUD_RATE
UART_PORT=$DEFAULT_UART_PORT
GDB_PORT=$DEFAULT_GDB_PORT
ACTION=""

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        -b|--baud)
            BAUD_RATE="$2"
            shift 2
            ;;
        -p|--port)
            UART_PORT="$2"
            shift 2
            ;;
        --gdb-port)
            GDB_PORT="$2"
            shift 2
            ;;
        --debug)
            ACTION="debug"
            shift
            ;;
        --monitor)
            ACTION="monitor"
            shift
            ;;
        --both)
            ACTION="both"
            shift
            ;;
        *)
            echo "Error: Unknown option $1"
            usage
            exit 1
            ;;
    esac
done

# Validate action
if [ -z "$ACTION" ]; then
    echo "Error: No action specified. Use --debug, --monitor, or --both."
    usage
    exit 1
fi

# Change to the project root
cd "$PROJECT_ROOT"

# Function to get STM32 extension tool paths
get_stm32_tool_paths() {
    # Try to get tool paths from environment variables first
    if [ -n "$CUBE_BUNDLE_PATH" ]; then
        ARM_TOOLS_PATH="$CUBE_BUNDLE_PATH/gnu-tools-for-stm32/13.3.1+st.9/bin"
        STLINK_PATH="$CUBE_BUNDLE_PATH/stlink-gdbserver/7.11.0+st.1/bin/ST-LINK_gdbserver"
    else
        # Fallback to common installation paths
        ARM_TOOLS_PATH="/home/user/.local/share/stm32cube/bundles/gnu-tools-for-stm32/13.3.1+st.9/bin"
        STLINK_PATH="/home/user/.local/share/stm32cube/bundles/stlink-gdbserver/7.11.0+st.1/bin/ST-LINK_gdbserver"
    fi
    echo "$ARM_TOOLS_PATH"
    echo "$STLINK_PATH"
}

# Function to start stlink-gdbserver
start_stlink_gdbserver() {
    echo "Starting ST-Link GDB server on port $GDB_PORT with SWD interface..."
    STLINK_PATH=$(get_stm32_tool_paths | tail -n 1)
    $STLINK_PATH -d -p $GDB_PORT &
    STLINK_PID=$!
    echo "ST-Link GDB server started with PID $STLINK_PID"
    sleep 2  # Give the server time to start
    # Check if the server is running
    if ! nc -z localhost $GDB_PORT >/dev/null 2>&1; then
        echo "Warning: Could not connect to ST-Link GDB server on port $GDB_PORT"
        echo "The server may not be running properly. Please check the connection."
    fi
}

# Function to start UART monitoring
start_uart_monitoring() {
    echo "Starting UART monitoring on $UART_PORT at $BAUD_RATE baud..."

    # Check for available terminal programs
    if command -v minicom &> /dev/null; then
        echo "Using Minicom for UART monitoring..."
        sudo minicom -D "$UART_PORT" -b "$BAUD_RATE" &
        MONITOR_PID=$!
    elif command -v screen &> /dev/null; then
        echo "Using Screen for UART monitoring..."
        sudo screen "$UART_PORT" $BAUD_RATE &
        MONITOR_PID=$!
    else
        echo "Warning: Neither Minicom nor Screen found. Please install one of them for UART monitoring."
        echo "You can manually monitor the UART port using: sudo cat $UART_PORT"
        return 1
    fi

    echo "UART monitoring started with PID $MONITOR_PID"
}

# Function to start GDB debugging session
start_gdb_debugging() {
    echo "Starting GDB debugging session..."
    GDB_PATH=$(get_stm32_tool_paths | head -n 1)/arm-none-eabi-gdb
    $GDB_PATH -q -x openocd_gdb_init.gdb &
    GDB_PID=$!
    echo "GDB started with PID $GDB_PID"
}

# Create GDB initialization file if it doesn't exist
if [ ! -f "openocd_gdb_init.gdb" ]; then
    cat > openocd_gdb_init.gdb << EOF
target remote localhost:$GDB_PORT
monitor reset halt
load
monitor reset
continue
EOF
    echo "Created GDB initialization file: openocd_gdb_init.gdb"
fi

# Execute the requested action(s)
case "$ACTION" in
    debug)
        start_stlink_gdbserver
        start_gdb_debugging
        ;;
    monitor)
        start_uart_monitoring
        ;;
    both)
        start_stlink_gdbserver
        start_gdb_debugging
        start_uart_monitoring
        ;;
esac

echo "Debugging session started successfully!"
echo "To stop the session, use: kill $STLINK_PID $GDB_PID $MONITOR_PID"