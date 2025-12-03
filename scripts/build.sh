#!/bin/bash
# STM32 Project Build Script
# This script provides a comprehensive build environment for the air-purity-sensor-stwink project
# using CMake and the STM32 extension tools.

set -e  # Exit on error

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Default configuration
DEFAULT_BUILD_TYPE=Debug
DEFAULT_CMAKE_PRESET=Debug
DEFAULT_CLEAN=false

# Function to get STM32 extension tool paths
get_stm32_tool_paths() {
    # Try to get tool paths from environment variables first
    if [ -n "$CUBE_BUNDLE_PATH" ]; then
        ARM_TOOLS_PATH="$CUBE_BUNDLE_PATH/gnu-tools-for-stm32/13.3.1+st.9/bin"
    else
        # Fallback to common installation path
        ARM_TOOLS_PATH="/home/user/.local/share/stm32cube/bundles/gnu-tools-for-stm32/13.3.1+st.9/bin"
    fi
    echo "$ARM_TOOLS_PATH"
}

# Function to display usage
usage() {
    echo "Usage: $0 [options]"
    echo
    echo "Options:"
    echo "  -h, --help                Show this help message"
    echo "  -t, --type <type>        Set build type (default: $DEFAULT_BUILD_TYPE)"
    echo "  -p, --preset <preset>    Set CMake preset (default: $DEFAULT_CMAKE_PRESET)"
    echo "  -c, --clean               Clean build directory before building"
    echo "  --rebuild                 Clean and rebuild the project"
    echo
    echo "Examples:"
    echo "  $0                         Build with default settings"
    echo "  $0 --type Release         Build in Release mode"
    echo "  $0 --clean                Clean build directory"
    echo "  $0 --rebuild             Clean and rebuild"
}

# Parse arguments
BUILD_TYPE=$DEFAULT_BUILD_TYPE
CMAKE_PRESET=$DEFAULT_CMAKE_PRESET
CLEAN=$DEFAULT_CLEAN

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        -t|--type)
            BUILD_TYPE="$2"
            shift 2
            ;;
        -p|--preset)
            CMAKE_PRESET="$2"
            shift 2
            ;;
        -c|--clean)
            CLEAN=true
            shift
            ;;
        --rebuild)
            CLEAN=true
            REBUILD=true
            shift
            ;;
        *)
            echo "Error: Unknown option $1"
            usage
            exit 1
            ;;
    esac
done

# Change to the project root
cd "$PROJECT_ROOT"

# Function to clean build directory
clean_build() {
    echo "Cleaning build directory..."
    rm -rf build
    echo "Build directory cleaned."
}

# Function to create build directory
create_build_dir() {
    mkdir -p "build/$CMAKE_PRESET"
    # We'll change to the build directory later
}

# Function to run CMake
run_cmake() {
    echo "Running CMake with preset: $CMAKE_PRESET and build type: $BUILD_TYPE..."
    # Set compiler paths using STM32 extension tools
    ARM_TOOLS_PATH=$(get_stm32_tool_paths)
    export CC="$ARM_TOOLS_PATH/arm-none-eabi-gcc"
    export CXX="$ARM_TOOLS_PATH/arm-none-eabi-g++"
    # Use cube-cmake if available (STM32 extension)
    if command -v cube-cmake &> /dev/null; then
        echo "Using cube-cmake from STM32 extension..."
        CMAKE_COMMAND="cube-cmake"
    else
        CMAKE_COMMAND="cmake"
    fi
    # Change to project root for CMake
    cd "$PROJECT_ROOT"
    $CMAKE_COMMAND --preset "$CMAKE_PRESET" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
    # Return to build directory
    cd "build/$CMAKE_PRESET"
}

# Function to build the project
build_project() {
    echo "Building project..."
    # Check if ninja is available and use it if configured in CMakePresets.json
    if command -v ninja &> /dev/null; then
        echo "Using ninja build system..."
        ninja
    else
        echo "Using make build system..."
        cmake --build . --config "$BUILD_TYPE"
    fi
    echo "Build completed successfully."
}

# Execute the build process
if [ "$CLEAN" = true ]; then
    clean_build
fi

# Change to project root for CMake
cd "$PROJECT_ROOT"
echo "Current directory: $(pwd)"
run_cmake
echo "Current directory after CMake: $(pwd)"
# We're already in the build directory after run_cmake
build_project

echo "Build process completed successfully!"
echo "Binary files are located in the build directory."