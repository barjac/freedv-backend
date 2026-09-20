//=========================================================================
// Name:            DiagnosticCsvLogger.cpp
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

#include "DiagnosticCsvLogger.h"

#include <cstdlib>
#include <string>

DiagnosticCsvLogger::DiagnosticCsvLogger()
    : file_(nullptr)
    , pendingHead_(0)
    , pendingCount_(0)
{
#if defined(FREEDV_ENABLE_AUDIO_DIAG_LOGGING)
    const char* home = std::getenv("HOME");
    if (home != nullptr)
    {
        std::string path = std::string(home) + "/agc_diag.csv";
        file_ = fopen(path.c_str(), "w");
        if (file_ != nullptr)
        {
            fprintf(file_, "elapsed_ms,input_dbfs,feedback_lufs,leveler_target_gain_db,leveler_current_gain_db,leveler_applied_gain_db,comp_limiter_gain_reduction_db,output_dbfs\n");
            fflush(file_);
        }
    }
#endif // defined(FREEDV_ENABLE_AUDIO_DIAG_LOGGING)

    startTime_ = std::chrono::steady_clock::now();
}

DiagnosticCsvLogger::~DiagnosticCsvLogger()
{
    if (file_ != nullptr)
    {
        fclose(file_);
        file_ = nullptr;
    }
}

void DiagnosticCsvLogger::logLevelerHalf(double inputDbfs, double feedbackLufs, double targetGainDb, double currentGainDb, double appliedGainDb) FREEDV_NONBLOCKING
{
    if (file_ == nullptr) return;

    if (pendingCount_ >= PENDING_QUEUE_CAPACITY)
    {
        // Overflow -- drop the oldest unflushed row rather than block or
        // allocate. Diagnostic data only; see header comment.
        pendingHead_ = (pendingHead_ + 1) % PENDING_QUEUE_CAPACITY;
        pendingCount_--;
    }

    int tail = (pendingHead_ + pendingCount_) % PENDING_QUEUE_CAPACITY;
    pendingQueue_[tail] = PendingRow{inputDbfs, feedbackLufs, targetGainDb, currentGainDb, appliedGainDb};
    pendingCount_++;
}

void DiagnosticCsvLogger::logCompressorLimiterHalfAndFlush(double gainReductionDb, double outputDbfs) FREEDV_NONBLOCKING
{
    if (file_ == nullptr || pendingCount_ == 0) return;

    PendingRow row = pendingQueue_[pendingHead_];
    pendingHead_ = (pendingHead_ + 1) % PENDING_QUEUE_CAPACITY;
    pendingCount_--;

    auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - startTime_).count();

    // Blocking file I/O accepted here -- diagnostic-only, not for
    // production use (see FREEDV_ENABLE_AUDIO_DIAG_LOGGING gating above).
    // Same rationale/precedent as AgcStep.cpp's own diagnostic log on the
    // bcj-agc-diagnostic-log branch.
    FREEDV_BEGIN_VERIFIED_SAFE
    fprintf(file_, "%lld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
        (long long)elapsedMs, row.inputDbfs, row.feedbackLufs,
        row.targetGainDb, row.currentGainDb, row.appliedGainDb, gainReductionDb, outputDbfs);
    fflush(file_);
    FREEDV_END_VERIFIED_SAFE
}
