//=========================================================================
// Name:            DiagnosticCsvLogger.h
// Purpose:         DIAGNOSTIC ONLY: shared CSV logger for the leveler/
//                  compressor-limiter pipeline steps, for live A/B tuning.
//
// Authors:         Claude Code (for Barry Jones, G4MKT)
// License:
//
//  All rights reserved.
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License version 2.1,
//  as published by the Free Software Foundation.  This program is
//  distributed in the hope that it will be useful, but WITHOUT ANY
//  WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
//  License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, see <http://www.gnu.org/licenses/>.
//
//=========================================================================

#ifndef UTIL__DIAGNOSTIC_CSV_LOGGER_H
#define UTIL__DIAGNOSTIC_CSV_LOGGER_H

#include <cstdio>
#include <chrono>

#include "freedv_sanitizers.h"

// DIAGNOSTIC ONLY -- not for production use. Only actually opens/writes
// anything when built with -DENABLE_AUDIO_DIAG_LOGGING=ON (see top-level
// CMakeLists.txt); otherwise every call here is a cheap no-op, so it's
// safe to leave both LevelerStep and CompressorLimiterStep unconditionally
// wired to a shared instance of this class.
//
// Writes one time-aligned CSV row per ~10ms processing chunk to
// ~/agc_diag.csv, feeding ~/freedv-scripts/agc_diag_wide_plot.py for live
// A/B tuning (see the "Replace AgcStep with a Leveler + Compressor/Limiter
// pair" plan's Verification section).
//
// LevelerStep calls logLevelerHalf() once per ~10ms sub-chunk it processes,
// and CompressorLimiterStep later calls logCompressorLimiterHalfAndFlush()
// once per matching sub-chunk to complete and write that row. These are
// NOT a simple alternating pair: within a single AudioPipeline::execute()
// call, LevelerStep runs its *entire* internal sub-chunk loop (and so
// calls logLevelerHalf() multiple times in a row) before
// CompressorLimiterStep is even invoked with that same buffer -- confirmed
// empirically wiring the two together with realistic (~100ms) chunk sizes,
// which is exactly why this class queues rows (FIFO) rather than holding a
// single pending slot; an earlier single-slot version silently dropped all
// but the last sub-chunk's row per call. Both classes split their input
// into identical chunk boundaries (see TEN_MS_SAMPLES in each .cpp) and
// process the exact same total sample count per call (AudioPipeline feeds
// one step's full output directly as the next step's full input), so the
// queue always drains to empty by the time both steps finish that call --
// no explicit synchronization needed between the two owning objects.
class DiagnosticCsvLogger
{
public:
    DiagnosticCsvLogger();
    ~DiagnosticCsvLogger();

    void logLevelerHalf(double inputDbfs, double feedbackLufs, double targetGainDb, double currentGainDb) FREEDV_NONBLOCKING;
    void logCompressorLimiterHalfAndFlush(double gainReductionDb, double outputDbfs) FREEDV_NONBLOCKING;

private:
    struct PendingRow
    {
        double inputDbfs;
        double feedbackLufs;
        double targetGainDb;
        double currentGainDb;
    };

    // Generous fixed capacity (no heap allocation -- keeps this honestly
    // FREEDV_NONBLOCKING even though it's diagnostic-only) -- far more than
    // the handful of ~10ms sub-chunks any single real-time buffer size
    // would ever produce in one call. Overflow silently drops the oldest
    // unflushed row rather than blocking or allocating; diagnostic data
    // only, never correctness-critical.
    static constexpr int PENDING_QUEUE_CAPACITY = 64;

    FILE* file_;
    std::chrono::steady_clock::time_point startTime_;

    PendingRow pendingQueue_[PENDING_QUEUE_CAPACITY];
    int pendingHead_;
    int pendingCount_;
};

#endif // UTIL__DIAGNOSTIC_CSV_LOGGER_H
