#!/bin/bash

# This script attempts to create automatically the symbolic links to the
#  grammar files. It requires the DSLs subfolders to be in ../dsls/

for LANG in query robot; do
    ln -s `find ../dsls/$LANG/ -name '*.xtext' | head -n 1` $LANG.xtext
done


