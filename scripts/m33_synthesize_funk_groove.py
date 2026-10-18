#!/usr/bin/env python3
"""
Synthesizes the funk instrumental soundtrack for the moonwalk performance:
- Upbeat tempo: 125 BPM (0.48 s per beat)
- Duration: 2.8667 seconds (synchronized with 0.5x presentation pacing)
- Instrumentation: slap bass, acoustic drums, clavinet chops, brass section accents
- Clean 44.1 kHz 16-bit stereo WAV saved to web/assets/audio_moonwalk_funk.wav
"""

import pathlib
import numpy as np
from scipy.io import wavfile
from scipy.signal import butter, lfilter

SAMPLE_RATE = 44100
DURATION = 2.8667  # seconds: exact presentation duration (86 frames / 60 fps / 0.5)
TOTAL_SAMPLES = int(SAMPLE_RATE * DURATION)
BPM = 125.0
BEAT_DUR = 60.0 / BPM  # 0.48 s
SIXTEENTH = BEAT_DUR / 4.0  # 0.12 s


def get_envelope(n_samples, attack=0.005, decay=0.08, sustain=0.6, release=0.05):
    """Generates an ADSR envelope."""
    n_att = int(attack * SAMPLE_RATE)
    n_dec = int(decay * SAMPLE_RATE)
    n_rel = int(release * SAMPLE_RATE)
    n_sus = max(0, n_samples - n_att - n_dec - n_rel)

    att = np.linspace(0, 1, n_att) if n_att > 0 else np.array([])
    dec = np.linspace(1, sustain, n_dec) if n_dec > 0 else np.array([])
    sus = np.full(n_sus, sustain)
    rel = np.linspace(sustain, 0, n_rel) if n_rel > 0 else np.array([])

    env = np.concatenate([att, dec, sus, rel])
    if len(env) < n_samples:
        env = np.pad(env, (0, n_samples - len(env)))
    return env[:n_samples]


def synth_kick(t_start, out_l, out_r, velocity=1.0):
    dur = 0.28
    n = int(dur * SAMPLE_RATE)
    idx = int(t_start * SAMPLE_RATE)
    if idx + n > TOTAL_SAMPLES:
        n = TOTAL_SAMPLES - idx
    t = np.linspace(0, dur, n, endpoint=False)

    # Pitch sweep 160 Hz -> 45 Hz
    freq = 45.0 + 115.0 * np.exp(-35.0 * t)
    phase = 2.0 * np.pi * np.cumsum(freq) / SAMPLE_RATE
    body = np.sin(phase) * np.exp(-12.0 * t)

    # Click transient
    click = np.random.randn(n) * np.exp(-120.0 * t) * 0.3
    signal = (body + click) * velocity * 0.9

    out_l[idx:idx+n] += signal
    out_r[idx:idx+n] += signal


def synth_snare(t_start, out_l, out_r, velocity=1.0):
    dur = 0.24
    n = int(dur * SAMPLE_RATE)
    idx = int(t_start * SAMPLE_RATE)
    if idx + n > TOTAL_SAMPLES:
        n = TOTAL_SAMPLES - idx
    t = np.linspace(0, dur, n, endpoint=False)

    # Tonal snap 220 Hz -> 140 Hz
    freq = 140.0 + 80.0 * np.exp(-30.0 * t)
    phase = 2.0 * np.pi * np.cumsum(freq) / SAMPLE_RATE
    tone = np.sin(phase) * np.exp(-22.0 * t) * 0.6

    # Noise crack (highpassed)
    noise = np.random.randn(n) * np.exp(-16.0 * t) * 0.7
    # Simple highpass
    b, a = butter(2, 800.0 / (SAMPLE_RATE / 2.0), btype='high')
    noise = lfilter(b, a, noise)

    signal = (tone + noise) * velocity * 0.75
    out_l[idx:idx+n] += signal
    out_r[idx:idx+n] += signal


def synth_hihat(t_start, out_l, out_r, open_hat=False, velocity=0.6, pan=0.0):
    dur = 0.30 if open_hat else 0.06
    n = int(dur * SAMPLE_RATE)
    idx = int(t_start * SAMPLE_RATE)
    if idx + n > TOTAL_SAMPLES:
        n = TOTAL_SAMPLES - idx
    t = np.linspace(0, dur, n, endpoint=False)

    noise = np.random.randn(n)
    b, a = butter(2, [4000.0 / (SAMPLE_RATE / 2.0), 16000.0 / (SAMPLE_RATE / 2.0)], btype='band')
    noise = lfilter(b, a, noise)
    decay = 10.0 if open_hat else 45.0
    signal = noise * np.exp(-decay * t) * velocity * 0.4

    gl = (1.0 - pan) * 0.5 + 0.5
    gr = (1.0 + pan) * 0.5 + 0.5
    out_l[idx:idx+n] += signal * gl
    out_r[idx:idx+n] += signal * gr


def synth_crash(t_start, out_l, out_r, velocity=0.8):
    dur = 0.8
    n = int(dur * SAMPLE_RATE)
    idx = int(t_start * SAMPLE_RATE)
    if idx + n > TOTAL_SAMPLES:
        n = TOTAL_SAMPLES - idx
    t = np.linspace(0, dur, n, endpoint=False)

    noise = np.random.randn(n)
    b, a = butter(2, 3500.0 / (SAMPLE_RATE / 2.0), btype='high')
    noise = lfilter(b, a, noise)
    signal = noise * np.exp(-4.5 * t) * velocity * 0.45

    out_l[idx:idx+n] += signal * 0.9
    out_r[idx:idx+n] += signal * 1.1


def synth_bass_note(t_start, freq, duration, out_l, out_r, velocity=1.0):
    n = int(duration * SAMPLE_RATE)
    idx = int(t_start * SAMPLE_RATE)
    if idx + n > TOTAL_SAMPLES:
        n = TOTAL_SAMPLES - idx
    t = np.linspace(0, duration, n, endpoint=False)

    # Rich harmonic sawtooth + sub sine
    harm1 = np.sin(2.0 * np.pi * freq * t)
    harm2 = 0.6 * np.sin(2.0 * np.pi * 2 * freq * t)
    harm3 = 0.3 * np.sin(2.0 * np.pi * 3 * freq * t)
    harm4 = 0.15 * np.sin(2.0 * np.pi * 4 * freq * t)
    raw = harm1 + harm2 + harm3 + harm4

    # Pluck envelope
    env = np.exp(-4.5 * t) * get_envelope(n, attack=0.004, decay=0.08, sustain=0.5, release=0.04)
    signal = raw * env * velocity * 0.85

    out_l[idx:idx+n] += signal
    out_r[idx:idx+n] += signal


def synth_clav(t_start, freqs, duration, out_l, out_r, velocity=0.7, pan=0.2):
    n = int(duration * SAMPLE_RATE)
    idx = int(t_start * SAMPLE_RATE)
    if idx + n > TOTAL_SAMPLES:
        n = TOTAL_SAMPLES - idx
    t = np.linspace(0, duration, n, endpoint=False)

    signal = np.zeros(n)
    for f in freqs:
        # Sharp staccato chop with odd harmonics
        w = np.sin(2.0 * np.pi * f * t) + 0.4 * np.sin(2.0 * np.pi * 3 * f * t)
        signal += w

    env = np.exp(-14.0 * t) * velocity * 0.25
    signal = signal * env

    gl = (1.0 - pan) * 0.5 + 0.5
    gr = (1.0 + pan) * 0.5 + 0.5
    out_l[idx:idx+n] += signal * gl
    out_r[idx:idx+n] += signal * gr


def synth_brass(t_start, freqs, duration, out_l, out_r, velocity=0.85):
    n = int(duration * SAMPLE_RATE)
    idx = int(t_start * SAMPLE_RATE)
    if idx + n > TOTAL_SAMPLES:
        n = TOTAL_SAMPLES - idx
    t = np.linspace(0, duration, n, endpoint=False)

    signal = np.zeros(n)
    for f in freqs:
        # Rich brass chord with detune
        w1 = np.sin(2.0 * np.pi * f * t)
        w2 = 0.5 * np.sin(2.0 * np.pi * (f * 1.003) * t)
        w3 = 0.3 * np.sin(2.0 * np.pi * (f * 2.0) * t)
        signal += (w1 + w2 + w3)

    env = get_envelope(n, attack=0.015, decay=0.09, sustain=0.65, release=0.06)
    signal = signal * env * velocity * 0.35

    out_l[idx:idx+n] += signal * 0.95
    out_r[idx:idx+n] += signal * 1.05


def main():
    print("================================================================")
    print("MW-M33: SYNTHESIZING ORIGINAL UPBEAT FUNK SOUNDTRACK")
    print("================================================================")

    out_l = np.zeros(TOTAL_SAMPLES, dtype=np.float32)
    out_r = np.zeros(TOTAL_SAMPLES, dtype=np.float32)

    # Note frequencies (Funk key of E)
    E1, G1, A1, B1 = 41.20, 48.99, 55.00, 61.74
    D2, E2, G2, A2 = 73.42, 82.41, 97.99, 110.00
    E3, G3, A3, B3, C4, D4 = 164.81, 196.00, 220.00, 246.94, 261.63, 293.66
    E4, G4, A4, B4, D5 = 329.63, 392.00, 440.00, 493.88, 587.33

    # Funk Chords
    E9_CHORD = [E3, G3, B3, D4, E4]
    A7_CHORD = [A3, C4, E4, G4]
    FINAL_HIT = [E3, B3, E4, G4, B4, D5]

    # 1. DRUMS & GROOVE (125 BPM -> 0.48s beat)
    # 6 beats = 2.88 s
    beats = [i * BEAT_DUR for i in range(7)]

    # Kicks
    synth_kick(beats[0], out_l, out_r, velocity=1.0)
    synth_kick(beats[0] + 3 * SIXTEENTH, out_l, out_r, velocity=0.85)  # syncopated kick
    synth_kick(beats[2], out_l, out_r, velocity=0.95)
    synth_kick(beats[2] + 2 * SIXTEENTH, out_l, out_r, velocity=0.80)
    synth_kick(beats[4], out_l, out_r, velocity=1.0)
    synth_kick(beats[5] + 2 * SIXTEENTH, out_l, out_r, velocity=1.1)   # final punch

    # Snares (Beats 2, 4, 6 and ghost notes)
    synth_snare(beats[1], out_l, out_r, velocity=1.0)
    synth_snare(beats[1] + 3 * SIXTEENTH, out_l, out_r, velocity=0.45) # ghost
    synth_snare(beats[3], out_l, out_r, velocity=1.0)
    synth_snare(beats[4] + 3 * SIXTEENTH, out_l, out_r, velocity=0.55) # pickup roll
    synth_snare(beats[5], out_l, out_r, velocity=0.75)
    synth_snare(beats[5] + 2 * SIXTEENTH, out_l, out_r, velocity=1.2)  # final crack

    # Hi-hats (16th note pattern)
    for step in range(24):
        t_hat = step * SIXTEENTH
        if t_hat >= DURATION - 0.2:
            break
        is_open = (step % 4 == 2)  # upbeat open hat
        vel = 0.7 if is_open else (0.5 if step % 2 == 0 else 0.35)
        pan = 0.2 if step % 2 == 0 else -0.2
        synth_hihat(t_hat, out_l, out_r, open_hat=is_open, velocity=vel, pan=pan)

    # Crash Cymbal on beat 1 and on final stinger
    synth_crash(beats[0], out_l, out_r, velocity=0.75)
    synth_crash(beats[5] + 2 * SIXTEENTH, out_l, out_r, velocity=1.0)

    # 2. FUNK BASSLINE
    synth_bass_note(beats[0], E1, 0.25, out_l, out_r, velocity=1.0)
    synth_bass_note(beats[0] + 2 * SIXTEENTH, E2, 0.16, out_l, out_r, velocity=0.9)  # slap octave pop
    synth_bass_note(beats[1] + 2 * SIXTEENTH, G1, 0.20, out_l, out_r, velocity=0.85)
    synth_bass_note(beats[2], A1, 0.22, out_l, out_r, velocity=0.95)
    synth_bass_note(beats[2] + 2 * SIXTEENTH, B1, 0.18, out_l, out_r, velocity=0.8)
    synth_bass_note(beats[3], D2, 0.22, out_l, out_r, velocity=0.9)
    synth_bass_note(beats[3] + 2 * SIXTEENTH, E2, 0.20, out_l, out_r, velocity=0.95)
    synth_bass_note(beats[4], E1, 0.24, out_l, out_r, velocity=1.0)
    synth_bass_note(beats[4] + 2 * SIXTEENTH, G2, 0.18, out_l, out_r, velocity=0.9)
    synth_bass_note(beats[5] + 2 * SIXTEENTH, E1, 0.45, out_l, out_r, velocity=1.2)  # final root punch

    # 3. RHYTHM CLAVINET CHOPS (Offbeats)
    for b in range(5):
        synth_clav(beats[b] + SIXTEENTH, E9_CHORD, 0.10, out_l, out_r, velocity=0.7)
        synth_clav(beats[b] + 3 * SIXTEENTH, E9_CHORD, 0.10, out_l, out_r, velocity=0.65)

    # 4. BRASS / HORN SECTION
    synth_brass(beats[0], E9_CHORD, 0.28, out_l, out_r, velocity=1.0)
    synth_brass(beats[1] + 2 * SIXTEENTH, E9_CHORD, 0.18, out_l, out_r, velocity=0.85)
    synth_brass(beats[2] + 2 * SIXTEENTH, A7_CHORD, 0.22, out_l, out_r, velocity=0.9)
    synth_brass(beats[4], E9_CHORD, 0.24, out_l, out_r, velocity=0.95)
    # Triumphant Final Stinger at t = 2.64 s
    synth_brass(beats[5] + 2 * SIXTEENTH, FINAL_HIT, 0.40, out_l, out_r, velocity=1.2)

    # 5. MASTERING: Soft analog saturation & normalization
    mix_l = np.tanh(out_l * 1.35)
    mix_r = np.tanh(out_r * 1.35)

    max_peak = max(np.max(np.abs(mix_l)), np.max(np.abs(mix_r)))
    if max_peak > 0:
        mix_l = (mix_l / max_peak) * 0.92
        mix_r = (mix_r / max_peak) * 0.92

    # Smooth fade out at very end (120 ms)
    n_fade = int(0.12 * SAMPLE_RATE)
    mix_l[-n_fade:] *= np.linspace(1, 0, n_fade)
    mix_r[-n_fade:] *= np.linspace(1, 0, n_fade)

    stereo_16bit = np.stack([
        (mix_l * 32767).astype(np.int16),
        (mix_r * 32767).astype(np.int16)
    ], axis=1)

    # Save to web/assets and output/m33
    p_web = pathlib.Path("web/assets/audio_moonwalk_funk.wav")
    p_out = pathlib.Path("output/m33/audio_moonwalk_funk.wav")

    p_web.parent.mkdir(parents=True, exist_ok=True)
    p_out.parent.mkdir(parents=True, exist_ok=True)

    wavfile.write(p_web, SAMPLE_RATE, stereo_16bit)
    wavfile.write(p_out, SAMPLE_RATE, stereo_16bit)

    print(f"Saved funk soundtrack to {p_web} ({p_web.stat().st_size / 1024:.1f} KB)")


if __name__ == "__main__":
    main()
