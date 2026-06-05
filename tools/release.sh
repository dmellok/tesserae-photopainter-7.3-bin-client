#!/usr/bin/env bash
#
# Cut a leak-safe GitHub release for the current FW_VERSION in platformio.ini.
#
# Threat model: include/secrets.h is a gitignored convenience file containing
# WiFi password, MQTT broker URI, MQTT user/pass etc. as #define string
# literals. A naive `pio run` followed by `gh release create` will publish a
# firmware.bin with those literals sitting in .rodata, recoverable in 3
# seconds with `strings firmware.bin | grep`. (This is the bug that bit the
# sibling tesserae-esp32-bin-client project before this script was written.)
#
# Two layers of defence, both required -- skipping either is a release-day
# foot-gun. They run in this order:
#
#   1. Pre-build move-aside (with restore trap). Before any pio invocation
#      we move include/secrets.h to a unique .secrets.h.release-backup.<pid>
#      sibling and install an EXIT trap to restore it. Trap covers normal
#      exit, ^C, error exit, anything that doesn't kill -9 the shell. So
#      the build sees a missing secrets.h, app_config.h's __has_include
#      guard skips the include, and WIFI_DEFAULT_SSID et al fall back to ""
#      (no compile-time defaults baked in).
#
#   2. Post-build leak scan. We parse every quoted string literal of >= 4
#      chars out of the stashed backup and grep each one against every
#      built .bin. Any match aborts the release with a LEAK line naming
#      the value -- this is the actual safety net; layer 1 is the cheap
#      "should never matter" preventative. If layer 1 ever silently fails
#      (a contributor edits in a new "$LITERAL" outside a #define, the
#      __has_include guard regresses, whatever) layer 2 still catches it.
#
# Everything else (version parsing, dirty-tree refusal, tag + gh release)
# is bog-standard hygiene around those two layers.
#
# Usage:
#   tools/release.sh                 # build, scan, tag, release using FW_VERSION
#   tools/release.sh --dry-run       # build + scan only -- no tag, no upload.
#                                    # Use to validate the leak layers after any
#                                    # edit to this script or to app_config.h.
#   tools/release.sh --notes-only    # print suggested release notes and exit
#
set -euo pipefail

DRY_RUN=0
case "${1:-}" in
    --dry-run) DRY_RUN=1; shift ;;
esac

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

# ----------------------------------------------------------------------------
# Version + tag
# ----------------------------------------------------------------------------
# The macro is written as -DFW_VERSION=\"X.Y.Z\" in platformio.ini so the
# escaped quotes survive PlatformIO's shell-parsing. Strip backslashes from
# the captured field before using it as a tag name.
VERSION=$(awk -F'"' '/-DFW_VERSION/ {gsub(/\\/, "", $2); print $2; exit}' platformio.ini)
if [[ -z "${VERSION:-}" ]]; then
    echo "error: couldn't parse FW_VERSION from platformio.ini" >&2
    exit 1
fi
TAG="v${VERSION}"

if [[ "${1:-}" == "--notes-only" ]]; then
    echo "## tesserae-photopainter-7.3-bin-client ${TAG}"
    echo
    git log --pretty=format:"- %s" "${TAG}~1..HEAD" 2>/dev/null \
        || git log --pretty=format:"- %s" -20
    exit 0
fi

# ----------------------------------------------------------------------------
# Guardrails -- run only on a clean tree that's caught up with origin/main.
# ----------------------------------------------------------------------------
# git status --porcelain excludes gitignored paths by default, so moving
# secrets.h aside in Layer 1 below WON'T re-dirty the tree status. Good.
# Dry-runs skip the dirty-tree + remote-ahead checks because they don't tag
# or upload anything -- they exist precisely to validate the leak layers
# from a feature branch with uncommitted edits to this script.
if (( ! DRY_RUN )); then
    if [[ -n "$(git status --porcelain)" ]]; then
        echo "error: working tree is dirty; commit/stash first" >&2
        git status --short >&2
        exit 1
    fi
    git fetch origin --quiet
    if ! git merge-base --is-ancestor origin/main HEAD; then
        echo "error: origin/main has commits this branch doesn't; pull/rebase first" >&2
        exit 1
    fi
fi

PIO="${PIO:-$HOME/.platformio/penv/bin/pio}"
ENV="${ENV:-tesserae-photopainter-73-bin-client}"
SECRETS="${SECRETS:-include/secrets.h}"
BAK="$(dirname "$SECRETS")/.$(basename "$SECRETS").release-backup.$$"

# ----------------------------------------------------------------------------
# Layer 1 -- move secrets aside with a restore trap so the build sees nothing.
# ----------------------------------------------------------------------------
# The trap fires on EXIT (normal, error, ^C). The `|| true` keeps the trap
# silent in the (degenerate) case that the file was already restored or the
# move never happened. Installing the trap BEFORE the mv would be marginally
# safer against a sub-millisecond ^C, but the standard idiom is mv-then-trap
# and the race window is negligible.
SECRETS_MOVED=0
if [[ -f "$SECRETS" ]]; then
    echo "==> moving ${SECRETS} aside for the build (${BAK##*/})"
    mv "$SECRETS" "$BAK"
    SECRETS_MOVED=1
    # shellcheck disable=SC2064
    # We want $BAK/$SECRETS captured at install-time, not expanded at trap-time.
    trap "mv \"$BAK\" \"$SECRETS\" 2>/dev/null || true" EXIT
else
    echo "==> no ${SECRETS} present; build proceeds with compile-time defaults only"
fi

# ----------------------------------------------------------------------------
# Build
# ----------------------------------------------------------------------------
echo "==> building ${ENV} for FW_VERSION=${VERSION}"
"$PIO" run -e "$ENV" >/dev/null

BUILD_DIR=".pio/build/${ENV}"
ARTIFACTS=(bootloader.bin partitions.bin firmware.bin firmware.factory.bin)
for f in "${ARTIFACTS[@]}"; do
    [[ -f "$BUILD_DIR/$f" ]] || { echo "error: missing $BUILD_DIR/$f" >&2; exit 1; }
done

# ----------------------------------------------------------------------------
# Layer 2 -- scan every artifact for any string literal from the (stashed)
# secrets file. Min 4 chars per needle to keep "on", "id", etc from
# false-positive-matching against unrelated bytes in the binary.
# ----------------------------------------------------------------------------
if (( SECRETS_MOVED )); then
    echo "==> scanning artifacts for any string literal from ${SECRETS}"
    NEEDLES=$(grep -oE '"[^"]+"' "$BAK" | tr -d '"' | sort -u || true)
    LEAKED=0
    while IFS= read -r needle; do
        [[ -n "$needle" ]] || continue
        [[ ${#needle} -ge 4 ]] || continue
        for f in "${ARTIFACTS[@]}"; do
            # Do NOT use `grep -q` (or `-m 1`) here: under `set -o pipefail`,
            # grep exits on first match, sends SIGPIPE (141) to strings,
            # and pipefail bubbles that 141 up as the pipeline's exit
            # code. The `if` test then reads "non-zero" and the leak is
            # SILENTLY MISSED -- this was the original bug. Let grep read
            # the whole stream and check whether it produced any output.
            match=$(strings -n 4 "$BUILD_DIR/$f" | grep -F -- "$needle" || true)
            if [[ -n "$match" ]]; then
                echo "  LEAK: '${needle}' appears in ${f}" >&2
                LEAKED=1
            fi
        done
    done <<< "$NEEDLES"
    if (( LEAKED )); then
        echo "error: aborting release -- one or more secrets were embedded in the build" >&2
        echo "       investigate before retrying; layer 1 (move-aside) is supposed to" >&2
        echo "       prevent this. Check app_config.h's __has_include guard and any" >&2
        echo "       new string literals introduced since the last release." >&2
        exit 1
    fi
    echo "    clean: no ${SECRETS} literals found in any artifact"
else
    echo "==> skipping secrets-leak scan (no secrets.h was present)"
fi

# ----------------------------------------------------------------------------
# Stage artifacts + checksums
# ----------------------------------------------------------------------------
OUT_DIR="release/${VERSION}"
mkdir -p "$OUT_DIR"
for f in "${ARTIFACTS[@]}"; do
    cp "$BUILD_DIR/$f" "$OUT_DIR/"
done

(cd "$OUT_DIR" && shasum -a 256 *.bin > SHA256SUMS)
echo "==> artifacts:"
ls -la "$OUT_DIR/"
echo "==> SHA256SUMS:"
cat "$OUT_DIR/SHA256SUMS"

if (( DRY_RUN )); then
    echo "==> dry-run: skipping tag + GitHub release (artifacts staged in ${OUT_DIR})"
    echo "==> done (dry-run)"
    exit 0
fi

# ----------------------------------------------------------------------------
# Tag
# ----------------------------------------------------------------------------
if git rev-parse "$TAG" >/dev/null 2>&1; then
    echo "==> tag ${TAG} already exists; skipping tag step"
else
    echo "==> tagging ${TAG}"
    git tag -a "$TAG" -m "$TAG"
    git push origin "$TAG"
fi

# ----------------------------------------------------------------------------
# Release
# ----------------------------------------------------------------------------
if gh release view "$TAG" >/dev/null 2>&1; then
    echo "==> release ${TAG} already exists; uploading (clobbering) assets"
    gh release upload "$TAG" "$OUT_DIR"/*.bin "$OUT_DIR/SHA256SUMS" --clobber
else
    echo "==> creating GitHub release ${TAG}"
    NOTES=$(mktemp)
    {
        echo "## Flashing"
        echo
        echo '`firmware.factory.bin` is the combined image; flash it to offset 0 with esptool:'
        echo
        echo '```bash'
        echo 'esptool.py --chip esp32s3 --port /dev/cu.usbmodem... \\'
        echo '    write_flash 0x0 firmware.factory.bin'
        echo '```'
        echo
        echo 'Or flash the three pieces separately at their native offsets:'
        echo
        echo '```bash'
        echo 'esptool.py --chip esp32s3 --port /dev/cu.usbmodem... \\'
        echo '    write_flash 0x0     bootloader.bin \\'
        echo '                0x8000  partitions.bin \\'
        echo '                0x10000 firmware.bin'
        echo '```'
        echo
        echo 'First-boot provisioning: join the SoftAP `Tesserae-Setup` (password'
        echo '`tesserae`) and the captive portal will pop up automatically. Scan the'
        echo 'QR code on the panel splash or type the SSID and broker details by hand.'
        echo
        echo "## Checksums"
        echo
        echo '```'
        cat "$OUT_DIR/SHA256SUMS"
        echo '```'
        echo
        echo "## Changes"
        echo
        if PREV_TAG=$(git describe --tags --abbrev=0 "${TAG}^" 2>/dev/null); then
            git log --pretty=format:"- %s" "${PREV_TAG}..${TAG}^"
        else
            git log --pretty=format:"- %s"
        fi
    } > "$NOTES"

    gh release create "$TAG" \
        --title "$TAG" \
        --notes-file "$NOTES" \
        "$OUT_DIR"/*.bin "$OUT_DIR/SHA256SUMS"
    rm -f "$NOTES"
fi

echo "==> done"
