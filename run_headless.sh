#!/usr/bin/env bash
export QT_QPA_PLATFORM=offscreen
exec ./build/solar_tracker "$@"
