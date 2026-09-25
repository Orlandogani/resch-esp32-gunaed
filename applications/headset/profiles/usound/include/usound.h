/**
 * @file usound.h
 * @brief Phase 1 profile: a wired USB headset.
 *
 * Duplex USB audio (UAC2) plus button controls over USB HID. The profile wires the
 * two rings `main/` owns to `subsys/usb_audio`, supplies the feedback servo's backlog
 * measurement, and translates button events into HID consumer-control reports.
 *
 * It does not create the rings, does not own the USB device stack's lifetime, and
 * applies no gain — `drivers/audio_playback` renders the bytes it is given, so
 * whoever writes the ring owns the volume (`applications/headset/docs/design.md`,
 * "Open questions").
 */
#pragma once

#include "headset_profile.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The `usound` profile singleton.
 *
 * Static storage; safe to hold for the life of the image. Registering it does not
 * touch hardware — that happens in `start()`.
 *
 * @return Never NULL.
 */
const headset_profile_t *usound_profile(void);

#ifdef __cplusplus
}
#endif
