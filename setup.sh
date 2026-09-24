#!/bin/bash

# Build dependencies, run by `make setup`: nlopt for armplanning's cgo binding.
# Runtime dependencies are first_run.sh's.

set -euo pipefail

case "$(uname -s)" in
Linux)
    SUDO=""
    if [[ "$(id -u)" -ne 0 ]]; then
        SUDO="sudo"
    fi
    $SUDO apt-get update
    $SUDO apt-get install -y --no-install-recommends libnlopt-dev
    ;;
Darwin)
    brew tap viamrobotics/brews
    brew install nlopt-static
    ;;
*)
    echo "No build dependencies known for $(uname -s); install nlopt by hand." >&2
    exit 1
    ;;
esac
