#!/usr/bin/env bash
set -euo pipefail
SDK_COMMIT="477ac31aa4ed12c8b201013ee42167f369a2a9b8"
rm -rf freeink-sdk
git clone https://github.com/Free-Ink/freeink-sdk.git freeink-sdk
cd freeink-sdk
git checkout "$SDK_COMMIT"
