# Live-updating view of ~/agc_diag.csv, written by the diagnostic build's
# AgcStep (branch bcj-agc-diagnostic-log). Run with:
#   gnuplot -persist agc_diag_plot.gp
# while FreeDV is transmitting. Refreshes once a second; Ctrl-C in the
# terminal to stop.

datafile = system("echo $HOME") . "/agc_diag.csv"

# Sized wide and tall enough that three stacked panels are each still
# readable -- the default wxt window is far too small for a 3-row
# multiplot. Adjust size/position for your own screen if needed.
#
# noenhanced: "enhanced" text mode treats '_' and '~' as subscript/
# overstrike markup rather than literal characters, which is what
# actually garbled "agc_diag.csv" (real underscore) and "~/agc_diag.csv"
# (literal tilde) in the waiting-for-data label -- not a real filename
# problem. None of this script's text needs sub/superscripts.
set terminal wxt size 1260,960 position 0,0 noenhanced font "sans,9"

set datafile separator ","
set grid

while (1) {

    # Show roughly the last 30 seconds so the plot stays readable during a
    # long session, once there's enough data to make that meaningful.
    # STATS_records ends up completely undefined (not 0) after this, in two
    # different ways depending on *how* there's nothing to summarize yet:
    # datafile not existing at all leaves any prior value alone, but
    # datafile existing with only a header row (the moment right after
    # FreeDV opens it, before the first 10ms block is logged) actively
    # clears it. So it must be re-defaulted to 0 *after* calling `stats`,
    # not before.
    stats datafile using 1 nooutput
    if (!exists("STATS_records")) {
        STATS_records = 0
    }
    has_data = (STATS_records > 5)

    if (has_data) {
        set xrange [(STATS_max / 1000.0) - 30 > 0 ? (STATS_max / 1000.0) - 30 : 0 : STATS_max / 1000.0]
    } else {
        set xrange [0:10]
    }

    set multiplot layout 3,1 title "AGC diagnostic -- live"

    set title "Input loudness (momentary LUFS)"
    set ylabel "LUFS"
    set yrange [-40:0]
    unset key
    if (has_data) {
        plot datafile using ($1/1000.0):2 with lines lc rgb "#2266cc" title "input LUFS"
    } else {
        # Not a literal "~/..." string -- that combination rendered as a
        # garbled glyph in this font/terminal (the '~' overlapping the
        # following 'a'). Uses the already-resolved datafile path instead,
        # which also has the benefit of showing the real path if $HOME
        # ever isn't what you expect.
        set label 1 "waiting for " . datafile . " (start transmitting in FreeDV)..." at graph 0.5,0.5 center
        plot NaN notitle
        unset label 1
    }

    set title "AGC gain"
    set ylabel "dB"
    set yrange [-22:14]
    if (has_data) {
        set key outside top center horizontal
        plot datafile using ($1/1000.0):3 with lines lc rgb "#cc6622" title "target gain", \
             datafile using ($1/1000.0):4 with lines lc rgb "#22aa44" title "current gain"
    } else {
        unset key
        plot NaN notitle
    }

    set title "Output level (post-AGC, post-limiter)"
    set xlabel "elapsed seconds"
    set ylabel "dBFS"
    set yrange [-40:2]
    unset key
    if (has_data) {
        plot datafile using ($1/1000.0):5 with lines lc rgb "#aa2266" title "output dBFS"
    } else {
        plot NaN notitle
    }

    unset multiplot

    pause 1
}
