#!/usr/bin/env bash
set -e
rm -rf freeink-sdk
git clone https://github.com/Free-Ink/freeink-sdk.git freeink-sdk
cd freeink-sdk
git checkout e62f6c16f0ed477ffbe1ad15fa838f32433adfc3
