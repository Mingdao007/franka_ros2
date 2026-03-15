#!/usr/bin/env bash
set -euo pipefail

ZROOT="$HOME/Zotero"
echo "[1] checking Zotero local root: $ZROOT"
[ -d "$ZROOT" ] || { echo "Missing $ZROOT"; exit 1; }

echo "[2] checking sqlite"
[ -f "$ZROOT/zotero.sqlite" ] || { echo "Missing $ZROOT/zotero.sqlite"; exit 1; }

echo "[3] checking storage"
[ -d "$ZROOT/storage" ] || { echo "Missing $ZROOT/storage"; exit 1; }

echo "[4] sample .zotero-ft-cache count"
find "$ZROOT/storage" -name '.zotero-ft-cache' -print | sed -n '1,5p'

echo "[5] done"
