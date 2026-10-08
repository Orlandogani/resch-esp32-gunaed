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

    /** Headset → dongle. [type][percent][flags]: battery state of charge, 0..100, from
     *  the open-circuit-voltage table (an estimate, ±10 %). `flags` bit 0 = external
     *  power present, bit 1 = charging. Sent on change and on link-up. */
    HLP_MSG_BATTERY = 0x03,

    /** Headset → dongle. [type][worn]: 1 when the IMU sees the headset moving, 0 after
     *  it has been still for the IMU's still time — a heuristic for "taken off". */
    HLP_MSG_WEAR = 0x04,

    /** Headset → dongle. [type][w][x][y][z]: head orientation, a unit quaternion (game
     *  rotation: no magnetometer, so heading drifts slowly), each component int16 Q14
     *  little-endian (16384 = 1.0). Sent at the IMU's pose rate while worn and linked.
     *  The dongle forwards it nowhere yet (TBD-014). */
    HLP_MSG_HEAD_POSE = 0x05,
} hlp_msg_type_t;

/* Bits of the one-byte consumer-control report, in the order of the dongle's (and
 * the usound profile's) HID report descriptor. */
#define HLP_HID_VOL_UP    (1u << 0)
#define HLP_HID_VOL_DOWN  (1u << 1)
#define HLP_HID_MUTE      (1u << 2)
#define HLP_HID_PLAY      (1u << 3)

#define HLP_BUTTON_LEN      2u
#define HLP_HOST_VOLUME_LEN 5u
#define HLP_BATTERY_LEN     3u
#define HLP_WEAR_LEN        2u
#define HLP_HEAD_POSE_LEN   9u

#define HLP_BATTERY_EXT_POWER (1u << 0)
#define HLP_BATTERY_CHARGING  (1u << 1)

#ifdef __cplusplus
}
#endif
