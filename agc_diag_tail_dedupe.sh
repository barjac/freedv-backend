#!/bin/bash
# Refreshes $2 (window_file) from the last 6000 rows of $1 (datafile),
# deduplicated to one row per unique elapsed_ms (column 1) -- several
# 10ms audio blocks can land in the same integer millisecond when
# flushed in a burst, and AgcStep.cpp logs one CSV row per block, so
# consecutive rows sometimes share an x value with different y values.
# Plotted with connected lines, that draws a tall near-vertical zigzag at
# that x before moving on to the next distinct timestamp -- a rendering
# artifact, not a real signal spike. Keeping only the last row per
# timestamp (assumes same-key rows are contiguous, true for this
# time-ordered append log) removes it at the source.
#
# Called from agc_diag_plot.gp's own system(), once per loop iteration.
# Written to a temp file then renamed into place atomically, so nothing
# reading window_file concurrently ever sees a partial write.

datafile=$1
window_file=$2

tail -n 6000 "$datafile" 2>/dev/null | awk -F',' '
    NR == 1 { print; next }
    $1 != prev_key { if (prev_line != "") print prev_line }
    { prev_key = $1; prev_line = $0 }
    END { if (prev_line != "") print prev_line }
' > "${window_file}.tmp" 2>/dev/null

mv -f "${window_file}.tmp" "$window_file" 2>/dev/null
