#!/bin/bash
sed s/ibs_\([^_]*\)_\([^_]*\)_\([^_]*\)_\([^ ]*\)/\1_\3/ $1 | gnuplot -p -e "plot '<cat' u 2:xticlabel(1)"
