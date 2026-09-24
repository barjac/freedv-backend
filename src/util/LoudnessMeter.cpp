//=========================================================================
// Name:            LoudnessMeter.cpp
// Purpose:         Thin libebur128 wrapper for EBU R128 momentary loudness
//                  measurement (mono).
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

#include "LoudnessMeter.h"

#include <cassert>
#include <cmath>

#include "ebur128.h" // from libebur128

LoudnessMeter::LoudnessMeter(int sampleRate)
{
    ebur128State_ = ebur128_init(1, sampleRate, EBUR128_MODE_M);
    assert(ebur128State_ != nullptr);
}

LoudnessMeter::~LoudnessMeter()
{
    ebur128_destroy((ebur128_state**)&ebur128State_);
}

void LoudnessMeter::addFrames(const short* samples, int numSamples) FREEDV_NONBLOCKING
{
    ebur128_state* state = static_cast<ebur128_state*>(ebur128State_);

    // Note: libebur128 is unlikely to use RT-unsafe constructs in normal
    // operation (per existing RTSan-enabled tests). Verified 2025-09-30,
    // see AgcStep.cpp's original use of the same call.
    FREEDV_BEGIN_VERIFIED_SAFE
    ebur128_add_frames_short(state, samples, numSamples);
    FREEDV_END_VERIFIED_SAFE
}

bool LoudnessMeter::getMomentaryLoudness(double* lufsOut, double silenceFloorLufs) const FREEDV_NONBLOCKING
{
    ebur128_state* state = static_cast<ebur128_state*>(ebur128State_);

    double lufs = 0.0;
    int result;
    FREEDV_BEGIN_VERIFIED_SAFE
    result = ebur128_loudness_momentary(state, &lufs);
    FREEDV_END_VERIFIED_SAFE

    // TEMPORARY (2026-09-24): always write *lufsOut, even when this
    // returns false -- lets a caller log the *actual* raw value (clamped
    // to a finite sentinel, since -HUGE_VAL doesn't round-trip through
    // %.2f/CSV cleanly) instead of an opaque "invalid" placeholder, to
    // properly calibrate SILENCE_FLOOR_LUFS_RNNOISE_OFF against real data
    // rather than guessing at another value blind. Existing callers are
    // unaffected -- they only read *lufsOut when this returns true, same
    // as before this change.
    if (result != EBUR128_SUCCESS)
    {
        *lufsOut = -200.0; // no measurement at all (e.g. not enough data yet)
        return false;
    }
    *lufsOut = (lufs == -HUGE_VAL) ? -200.0 : lufs;

    // -HUGE_VAL (genuine, unconditional silence) is always rejected
    // regardless of silenceFloorLufs; the floor itself is the caller-
    // supplied, situational cutoff (see the header's own comment).
    if (lufs == -HUGE_VAL || lufs <= silenceFloorLufs)
    {
        return false;
    }

    return true;
}

void LoudnessMeter::reset() FREEDV_NONBLOCKING
{
    // Intentional no-op -- see header comment.
}
