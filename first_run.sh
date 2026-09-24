#!/bin/bash

# Runtime dependencies of bin/viam-mirka. The Linux binary links the nlopt
# shared library; the Darwin binary links nlopt statically and needs nothing.
# Build dependencies are setup.sh's.

set -euo pipefail

case "$(uname -s)" in
Linux)
    # ldconfig lives in sbin, which a non-root PATH may leave out. No grep -q:
    # it exits at the first match and pipefail would report ldconfig's SIGPIPE.
    if PATH="$PATH:/sbin:/usr/sbin" ldconfig -p | grep libnlopt >/dev/null; then
        echo "libnlopt is installed; nothing to do."
        exit 0
    fi
    SUDO=""
    if [[ "$(id -u)" -ne 0 ]]; then
        SUDO="sudo"
    fi
    if ! $SUDO apt-get install -y --no-install-recommends libnlopt0; then
        echo "viam-mirka needs the nlopt shared library (libnlopt.so, Debian/Ubuntu package libnlopt0)" \
            "and could not install it. Install libnlopt0, then restart the module." >&2
        exit 1
    fi
    ;;
*)
    echo "No runtime dependencies to install on $(uname -s)."
    ;;
esac
