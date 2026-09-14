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

constexpr double SILENCE_FLOOR_LUFS = -70.0; // libebur128 returns -HUGE_VAL below this; treated as "no valid reading"

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

bool LoudnessMeter::getMomentaryLoudness(double* lufsOut) const FREEDV_NONBLOCKING
{
    ebur128_state* state = static_cast<ebur128_state*>(ebur128State_);

    double lufs = 0.0;
    int result;
    FREEDV_BEGIN_VERIFIED_SAFE
    result = ebur128_loudness_momentary(state, &lufs);
    FREEDV_END_VERIFIED_SAFE

    if (result != EBUR128_SUCCESS || lufs == -HUGE_VAL || lufs <= SILENCE_FLOOR_LUFS)
    {
        return false;
    }

    *lufsOut = lufs;
    return true;
}

void LoudnessMeter::reset() FREEDV_NONBLOCKING
{
    // Intentional no-op -- see header comment.
}
