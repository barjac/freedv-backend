# Live-updating view of ~/agc_diag.csv, written by the diagnostic build's
# AgcStep (branch bcj-agc-diagnostic-log). Run with:
#   gnuplot -persist agc_diag_plot.gp
# while FreeDV is transmitting. Refreshes once a second; Ctrl-C in the
# terminal to stop.

datafile = system("echo $HOME") . "/agc_diag.csv"

# A long session logs at ~100 rows/sec and only the last 30s is ever
# shown, but stats/plot re-parsing the *entire*, ever-growing datafile
# every second doesn't scale to a long session. window_file is a small
# rolling tail of datafile, refreshed via `tail` right below, once per
# loop iteration, by this script itself -- NOT by a separate process.
#
# An earlier version had a standalone shell loop in freedv-start-diag
# doing this refresh independently and concurrently. That seemed
# necessary at the time because calling `tail` via system() from inside
# this script appeared to silently stop taking effect once actually run
# the way freedv-start-diag launches it -- but that was investigated
# before the fatal has_data-false crash bug (see below) was found and
# fixed, and was most likely actually that same crash killing the whole
# process early, not a real problem with system(). The two-process
# version then went on to cause a *different*, confirmed, reproducible
# bug of its own: window_file could be read by this script's own
# stats/plot calls while the other process was concurrently rewriting
# it, causing "all points out of range" (even switching that loop to
# atomic write-then-rename only narrowed the race, since gnuplot's own
# autoscale still does more than one internal pass over the file, and
# the two reads could land on different -- if individually consistent --
# versions of it). A single process, refreshing synchronously right
# before reading, has no concurrent writer and so cannot have this
# problem at all.
window_file = system("echo $HOME") . "/freedv-data/agc_diag_window.csv"

# Sized wide and tall enough that three stacked panels are each still
# readable -- the default wxt window is far too small for a 3-row
# multiplot. Adjust size/position for your own screen if needed.
#
# noenhanced: "enhanced" text mode treats '_' and '~' as subscript/
# overstrike markup rather than literal characters, which is what
# actually garbled "agc_diag.csv" (real underscore) and "~/agc_diag.csv"
# (literal tilde) in the waiting-for-data label -- not a real filename
# problem. None of this script's text needs sub/superscripts.
#
# x11, not wxt: the script's logic and the data feeding it are both
# confirmed correct (has_data flips true, window_file has fresh valid
# rows, no errors in gnuplot's own stderr) -- yet the window stayed
# permanently blank past its title bar through multiple long, real
# transmissions once actually launched the way freedv-start-diag does
# (backgrounded, nested script, inside konsole). Never reproduced this
# in any isolated/foreground test. x11 is gnuplot's older, more basic
# interactive terminal -- worth trying as it doesn't share wxt's
# wxWidgets toolkit plumbing, a plausible source of a context-specific
# redraw bug that never surfaces as a script-level error.
set terminal x11 size 1260,960 position 0,0 noenhanced font "sans,9"

set datafile separator ","
set grid

while (1) {

    # Refresh window_file from datafile synchronously, right here, before
    # anything below reads it -- single process, no concurrent writer, so
    # nothing can observe a torn/mid-write version of it. Deduplicated to
    # one row per unique elapsed_ms (see agc_diag_tail_dedupe.sh) and
    # written to a temp file then renamed into place, out of caution
    # (costs nothing) even though this script is now the only writer.
    system("'" . system("echo $HOME") . "/GIT/freedv-backend/agc_diag_tail_dedupe.sh' '" . datafile . "' '" . window_file . "'")

    # `stats` filters the data it summarizes against whatever xrange/yrange
    # currently happen to be *set* ("if the axis is autoscaled, no range
    # limits are applied -- otherwise only values in range are considered",
    # per gnuplot's own docs for `stats`) -- and a panel's yrange from the
    # previous loop iteration (e.g. output dBFS's -60:5) is still sitting
    # there at the top of THIS iteration, before anything below sets a new
    # one. Column 1 (elapsed_ms, values in the hundreds of thousands) is
    # treated as "y" for a single-column stats call, so it was being
    # filtered against that leftover range and coming back essentially
    # empty every iteration after the first -- silently, since a
    # gnuplot-level warning doesn't stop the loop, it just makes
    # has_data wrongly false and everything downstream chase a ghost. This
    # was the actual root cause of "all points out of range" appearing
    # from the second iteration onward, through several failed fix
    # attempts (autoscale, per-column stats, fixed yrange) that all missed
    # it because none of them touched *this* call. Force autoscale (no
    # filtering, confirmed by the same docs) right here so this call is
    # never at the mercy of whatever a previous iteration's last panel
    # happened to leave set.
    set xrange [*:*]
    set yrange [*:*]

    # Show roughly the last 30 seconds so the plot stays readable during a
    # long session, once there's enough data to make that meaningful.
    # STATS_records ends up completely undefined (not 0) after this, in two
    # different ways depending on *how* there's nothing to summarize yet:
    # window_file not existing at all leaves any prior value alone, but
    # window_file existing with only a header row (the moment right after
    # FreeDV opens it, before the first 10ms block is logged) actively
    # clears it. So it must be re-defaulted to 0 *after* calling `stats`,
    # not before.
    stats window_file using 1 nooutput
    if (!exists("STATS_records")) {
        STATS_records = 0
    }
    has_data = (STATS_records > 5)

    if (has_data) {
        # Explicit numeric range computed from THIS stats call, not
        # `set xrange [*:*]` -- gnuplot's own autoscale kept producing "all
        # points out of range" against real, live, actively-growing data,
        # intermittently, even in this single-process design where nothing
        # else touches window_file between refresh and reading it. Not
        # reproduced in any isolated/synthetic test, only in real live
        # sessions, and the exact mechanism was never pinned down --
        # tried disabling it via explicit ranges computed from `stats`
        # instead (already proven reliable throughout this debugging)
        # rather than continuing to rely on whatever's going wrong inside
        # gnuplot's own autoscale in this context. window_file doesn't
        # change again after this point in the same iteration (see the
        # refresh comment above), so there's no new opportunity for these
        # bounds to go stale before the plot commands below use them.
        X_MIN = STATS_min / 1000.0
        X_MAX = STATS_max / 1000.0
        # +0.1 guards against a degenerate zero-width range if every row
        # currently in window_file happens to share one timestamp (seen
        # early in a session, before real audio arrives).
        if (X_MAX <= X_MIN) { X_MAX = X_MIN + 0.1 }
        set xrange [X_MIN:X_MAX]
    } else {
        set xrange [0:10]
    }

    set multiplot layout 3,1 title "AGC diagnostic -- live"

    set title "Input loudness (momentary LUFS)"
    set ylabel "LUFS"
    unset key
    if (has_data) {
        # Fixed, generous range -- NOT autoscale, NOT computed from `stats`
        # on this column either. Both were tried and both still produced
        # "all points out of range" against real live data, intermittently,
        # for reasons never pinned down (autoscale: gnuplot's own internal
        # mechanism; per-column stats: extra stats calls stacked per
        # iteration appeared to confuse gnuplot's own multiplot/replot
        # state -- warnings started citing "$GPVAL_LAST_MULTIPLOT" as the
        # source file, not this script). A fixed range computed from
        # nothing but AGC's own known constants can't have either problem:
        # SILENCE_THRESHOLD_LUFS is -33 in AgcStep.cpp, and raw (pre-AGC)
        # input has been seen as low as -60ish in testing -- -70:5 covers
        # that with margin either way.
        set yrange [-70:5]
        plot window_file using ($1/1000.0):2 with lines lc rgb "#2266cc" title "input LUFS"
    } else {
        # Fixed range, NOT [*:*] -- autoscaling `plot NaN` when there is no
        # other data anywhere yet to scale against is a FATAL gnuplot error
        # ("all points y value undefined!"), not just a cosmetic warning:
        # it kills the whole interpreter process outright. Confirmed via
        # direct reproduction -- exit code 1 with yrange [*:*], exit code 0
        # with any fixed range. This is what was actually behind gnuplot
        # going permanently blank the moment the modem started: has_data
        # flipping false for the first time in a session hit exactly this,
        # silently killing the main gnuplot process while `-persist` kept
        # its now-orphaned window open showing whatever was last drawn --
        # looking exactly like a frozen/stuck window from the outside, and
        # explaining why only a fresh launch against an already-nonempty
        # file ever appeared to work.
        set yrange [-1:1]
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
    if (has_data) {
        # Fixed, generous range -- see the input-LUFS panel's comment
        # above for why not autoscale or per-column stats. AGC's own
        # constants (AgcStep.cpp) clamp gain to [-20:12]dB -- -25:15
        # covers that with margin.
        set yrange [-25:15]
        set key outside top center horizontal
        plot window_file using ($1/1000.0):3 with lines lc rgb "#cc6622" title "target gain", \
             window_file using ($1/1000.0):4 with lines lc rgb "#22aa44" title "current gain"
    } else {
        set yrange [-1:1]
        unset key
        plot NaN notitle
    }

    set title "Output level (post-AGC, post-limiter)"
    set xlabel "elapsed seconds"
    set ylabel "dBFS"
    unset key
    if (has_data) {
        # Fixed, generous range -- see the input-LUFS panel's comment
        # above for why not autoscale or per-column stats. LIMITER_LEVEL_DB
        # targets -1dBFS in AgcStep.cpp; -60:5 covers real output down
        # through quiet passages with margin (occasional true-silence
        # blocks logged at the -100dBFS sentinel get clipped off the
        # bottom of this range -- deliberate, keeps the interesting ~20dB
        # of real signal readable instead of compressed into a sliver).
        set yrange [-60:5]
        plot window_file using ($1/1000.0):5 with lines lc rgb "#aa2266" title "output dBFS"
    } else {
        set yrange [-1:1]
        plot NaN notitle
    }

    unset multiplot

    pause 1
}
