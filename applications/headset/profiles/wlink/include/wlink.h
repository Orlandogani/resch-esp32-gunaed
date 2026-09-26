/**
 * @file wlink.h
 * @brief Phase 2 profile: a wireless headset talking to the dongle.
 *
 * Duplex audio over `subsys/audio_link` in the peripheral role (BLE 2M PHY + Opus,
 * ADR-022): the downlink is decoded into the playback ring `main/` owns, the
 * microphone is encoded from `drivers/audio_capture`'s ring, and every frame the
 * headset sends reports the playback backlog so the dongle's USB feedback servo can
 * make the host send at this headset's I2S rate. Buttons become CONTROL messages
 * (`headset_link_proto.h`); the dongle owns the HID surface toward the PC.
 *
 * Like `usound`, it owns its transport: `start()` brings BLE up and `stop()` takes it
 * down, so the radio is powered only while this profile is the active one.
 */
#pragma once

#include "headset_profile.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The `wlink` profile singleton. Static storage; registering it touches no
 *        hardware. @return Never NULL.
 */
const headset_profile_t *wlink_profile(void);

#ifdef __cplusplus
}
#endif
