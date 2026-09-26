/**
 * @file wlink_central.h
 * @brief The dongle end of the wireless link: `subsys/audio_link` as BLE central.
 *
 * The host's speaker stream is encoded and sent to the headset; the headset's
 * microphone is decoded into the mic ring the host side reads. The speaker path's
 * backlog — what the dongle has not yet sent plus what the headset reports it is
 * holding for playback — is handed to the host side's feedback servo, so the host
 * ends up sending at the headset's I2S rate and no resampler is needed anywhere
 * (ADR-022). The microphone direction needs no servo: USB IN is an asynchronous
 * source, and TinyUSB sizes each packet from its FIFO fill (FW-AUD-024).
 *
 * Headset button messages become HID consumer taps on the host; host volume changes
 * are forwarded to the headset (`headset_link_proto.h`).
 */
#pragma once

#include "dongle_backend.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief The singleton. Static storage; touches no hardware until `start()`. */
const dongle_backend_t *wlink_central_backend(void);

#ifdef __cplusplus
}
#endif
