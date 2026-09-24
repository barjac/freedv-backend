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

    // Returns true if a valid (non-gated, non-silent) momentary loudness
    // reading is currently available -- this is the authoritative signal
    // callers should use to decide whether to trust *lufsOut as real
    // feedback, mirroring AgcStep's original EBUR128_SUCCESS/-HUGE_VAL
    // check. *lufsOut is TEMPORARILY (2026-09-24) always written
    // regardless of the return value, clamped to -200.0 when there's no
    // real measurement (true silence, or not enough data yet) -- so a
    // caller can log the raw value for diagnostic calibration even when
    // it's being rejected. Existing callers that only read *lufsOut when
    // this returns true (the intended, permanent contract) are unaffected.
    //
    // silenceFloorLufs (2026-09-24, Barry -- found via a real capture: with
    // RNNoise off, genuine gaps between words in a reasonably quiet room
    // can read quieter than RNNoise's own small residual noise floor during
    // the same gaps, so momentary loudness drops below the default floor
    // far more often on real, ordinary speech, not just true silence --
    // confirmed by a live on/off/on/off test that it always recovers, so
    // this is a real, if surprising, gating difference, not a stuck/
    // corrupted state). Callers that care can pass a looser value; default
    // matches the original, always-used -70.0f so existing callers/tests
    // are unaffected.
    bool getMomentaryLoudness(double* lufsOut, double silenceFloorLufs = -70.0) const FREEDV_NONBLOCKING;

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
