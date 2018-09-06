#!/bin/bash

find $(pwd) -type f -name "*.c" | while read file; do dos2unix "$file" "$file"; done
find $(pwd) -type f -name "*.h" | while read file; do dos2unix "$file" "$file"; done
dos2unix ./Makefile ./Makefile
