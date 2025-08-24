#!/bin/bash

#
# Script to run all Rust examples in the baby_shark project.
#
# Usage:
#     ./run-all-examples.sh [PROFILE] [OPTIONS]
#
# Arguments:
#     PROFILE     Cargo profile to use (default: debug)
#                 Common values: debug, release, dev-opt, release-debug-info
#
# Options:
#     -v, --verbose           Enable verbose output
#     -c, --continue-on-error Continue running remaining examples even if one fails
#     -h, --help             Show this help message
#
# Examples:
#     ./run-all-examples.sh
#     ./run-all-examples.sh release
#     ./run-all-examples.sh dev-opt --verbose --continue-on-error
#

# Default values
PROFILE="dev-opt"
VERBOSE=false
CONTINUE_ON_ERROR=false

# Color output functions
print_success() {
    echo -e "\033[32m✓ $1\033[0m"
}

print_error() {
    echo -e "\033[31m✗ $1\033[0m"
}

print_info() {
    echo -e "\033[36mℹ $1\033[0m"
}

print_warning() {
    echo -e "\033[33m⚠ $1\033[0m"
}

# Help function
show_help() {
    echo "Script to run all Rust examples in the baby_shark project."
    echo ""
    echo "Usage:"
    echo "    $0 [PROFILE] [OPTIONS]"
    echo ""
    echo "Arguments:"
    echo "    PROFILE     Cargo profile to use (default: debug)"
    echo "                Common values: debug, release, dev-opt, release-debug-info"
    echo ""
    echo "Options:"
    echo "    -v, --verbose           Enable verbose output"
    echo "    -c, --continue-on-error Continue running remaining examples even if one fails"
    echo "    -h, --help             Show this help message"
    echo ""
    echo "Examples:"
    echo "    $0"
    echo "    $0 release"
    echo "    $0 dev-opt --verbose --continue-on-error"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -c|--continue-on-error)
            CONTINUE_ON_ERROR=true
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        -*)
            echo "Unknown option $1"
            show_help
            exit 1
            ;;
        *)
            if [[ -z "$PROFILE_SET" ]]; then
                PROFILE="$1"
                PROFILE_SET=true
            else
                echo "Too many arguments"
                show_help
                exit 1
            fi
            shift
            ;;
    esac
done

# Change to project root directory
cd "$(dirname "$0")/.."

# Define all available examples
examples=(
    "boolean"
    # "bsphere_simplification" skip this example because it expects user input
    "deform"
    "dual_contouring"
    "incremental_remeshing"
    "lightweighting"
    "offset"
    "simplification"
    "voxel_remeshing"
)

# Main execution
print_info "Running all baby_shark examples with profile: $PROFILE"
print_info "Found ${#examples[@]} examples to run"

success_count=0
failure_count=0
start_time=$(date +%s)

for example in "${examples[@]}"; do
    echo ""
    print_info "Running example: $example"
    echo "=================================================="
    
    # Build the cargo command
    cargo_args=("run" "--example" "$example")
    
    # Add profile if not debug (default)
    if [[ "$PROFILE" != "debug" ]]; then
        cargo_args+=("--profile" "$PROFILE")
    fi
    
    # Add verbose flag if requested
    if [[ "$VERBOSE" == "true" ]]; then
        cargo_args+=("--verbose")
    fi
    
    # Run the example
    if cargo "${cargo_args[@]}"; then
        print_success "Example '$example' completed successfully"
        ((success_count++))
    else
        exit_code=$?
        print_error "Example '$example' failed with exit code $exit_code"
        ((failure_count++))
        
        if [[ "$CONTINUE_ON_ERROR" != "true" ]]; then
            print_error "Stopping execution due to failure. Use --continue-on-error to continue running remaining examples."
            break
        fi
    fi
done

# Summary
end_time=$(date +%s)
duration=$((end_time - start_time))

echo ""
echo "=================================================="
print_info "Execution Summary"
echo "=================================================="
print_success "Successful examples: $success_count"
if [[ $failure_count -gt 0 ]]; then
    print_error "Failed examples: $failure_count"
fi
print_info "Total duration: ${duration} seconds"
print_info "Profile used: $PROFILE"

# Exit with appropriate code
if [[ $failure_count -gt 0 ]]; then
    exit 1
else
    exit 0
fi
