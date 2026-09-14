//=========================================================================
// Name:            LoudnessMeter.h
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

#ifndef UTIL__LOUDNESS_METER_H
#define UTIL__LOUDNESS_METER_H

#include "freedv_sanitizers.h"

// Mono EBU R128 momentary loudness meter (K-weighted RMS over the last
// 400ms, gated per BS.1770 -- see libebur128). Used by CompressorLimiterStep
// to measure its own output for LevelerStep's feedback loop (see the
// "Replace AgcStep with a Leveler + Compressor/Limiter pair" plan).
class LoudnessMeter
{
public:
    LoudnessMeter(int sampleRate);
    ~LoudnessMeter();

    void addFrames(const short* samples, int numSamples) FREEDV_NONBLOCKING;

    // Returns true and sets *lufsOut if a valid (non-gated, non-silent)
    // momentary loudness reading is currently available. Returns false
    // (leaving *lufsOut untouched) during silence/gating, mirroring
    // AgcStep's original EBUR128_SUCCESS/-HUGE_VAL check.
    bool getMomentaryLoudness(double* lufsOut) const FREEDV_NONBLOCKING;

    // No-op: libebur128 has no RT-safe way to clear its internal loudness
    // history (only destroy+reinit, both of which allocate) -- reset() is
    // declared FREEDV_NONBLOCKING (see IPipelineStep) so it must not
    // allocate. Matches AgcStep's own original behavior, which never reset
    // its ebur128 state either.
    void reset() FREEDV_NONBLOCKING;

private:
    void* ebur128State_;
};

#endif // UTIL__LOUDNESS_METER_H
