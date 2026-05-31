@echo off
docker run --rm -i -w /app -v "%CD%:/app" rebble/pebble-sdk pebble build
