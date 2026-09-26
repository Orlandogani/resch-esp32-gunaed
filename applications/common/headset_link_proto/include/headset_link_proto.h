/**
 * @file headset_link_proto.h
 * @brief CONTROL messages between the headset (`wlink` profile) and the dongle.
 *
 * `subsys/audio_link` carries CONTROL frames as opaque bytes (ADR-013); this header is
 * what the two products agree those bytes mean. Both applications include it, so the
 * vocabulary cannot drift apart. Byte 0 is the message type; the rest is per type.
 * Unknown types are ignored, so either side may add messages without breaking the
 * other.
 *
 * The dongle owns the HID surface toward the PC (applications/dongle/docs/design.md);
 * the headset only reports what was pressed.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    /** Headset → dongle. [type][bits]: one HID consumer-control tap. `bits` is the
     *  one-byte report of the dongle's consumer-control descriptor, below. The
     *  dongle sends it as a press followed by an all-zero release. */
    HLP_MSG_BUTTON = 0x01,

    /** Dongle → headset. [type][which][mute][vol_lo][vol_hi]: the host changed mute
     *  or volume on a UAC2 feature unit. `which` 0 = speaker, 1 = mic; volume in
     *  1/256 dB, signed, little-endian. Informational: neither side attenuates yet
     *  (applications/headset/docs/design.md, "Open questions"). */
    HLP_MSG_HOST_VOLUME = 0x02,
} hlp_msg_type_t;

/* Bits of the one-byte consumer-control report, in the order of the dongle's (and
 * the usound profile's) HID report descriptor. */
#define HLP_HID_VOL_UP    (1u << 0)
#define HLP_HID_VOL_DOWN  (1u << 1)
#define HLP_HID_MUTE      (1u << 2)
#define HLP_HID_PLAY      (1u << 3)

#define HLP_BUTTON_LEN      2u
#define HLP_HOST_VOLUME_LEN 5u

#ifdef __cplusplus
}
#endif
