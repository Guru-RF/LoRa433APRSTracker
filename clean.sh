#!/bin/bash

TARGET="/Volumes/APRSTRKR"

if [ -d "$TARGET" ]; then
    rm -rf "$TARGET"/*
    sync
fi
