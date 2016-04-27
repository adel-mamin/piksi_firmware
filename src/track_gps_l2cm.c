/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 * Contact: Adel Mamin <adelm@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track.h"
#include "track_gps_l2cm.h"
#include "track_api.h"
#include "decode.h"
#include "manage.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

#include <string.h>

#include "settings.h"

#define NUM_GPS_L2CM_TRACKERS   12

/** L2C coherent integration time [ms] */
#define L2C_COHERENT_INTEGRATION_TIME_MS 20

/* Alias detection interval [ms] */
#define L2C_ALIAS_DETECT_INTERVAL_MS     500

#define L2CM_TRACK_SETTING_SECTION "l2cm_track"

/*  code: nbw zeta k carr_to_code
 carrier:                    nbw  zeta k fll_aid */

#define LOOP_PARAMS_MED "(20 ms, (1, 0.7, 1, 1200), (13, 0.7, 1, 5))"

/*                          k1,   k2,  lp,  lo */
#define LD_PARAMS          "0.0247, 1.5, 50, 240"
#define LD_PARAMS_DISABLE  "0.02, 1e-6, 1, 1"

#define CN0_EST_LPF_CUTOFF 5

typedef struct {
  aided_tl_state_t tl_state;   /**< Tracking loop filter state. */
  u32 code_phase_rate_fp;      /**< Code phase rate in NAP register units. */
  s32 carrier_freq_fp;         /**< Carrier frequency in NAP register units. */
  u32 corr_sample_count;       /**< Number of samples in correlation period. */
  corr_t cs[3];                /**< EPL correlation results in correlation period. */
  cn0_est_state_t cn0_est;     /**< C/N0 Estimator. */
  u8 int_ms;                   /**< Integration length. */
  bool short_cycle;            /**< Set to true when a short 1ms integration is requested. */
  u8 stage;                    /**< 0 = First-stage. 1 ms integration.
                                    1 = Second-stage. After nav bit sync,
                                    retune loop filters and typically (but
                                    not necessarily) use longer integration. */
  alias_detect_t alias_detect; /**< Alias lock detector. */
  lock_detect_t lock_detect;   /**< Phase-lock detector state. */
} gps_l2cm_tracker_data_t;

/** Do L1C/A to L2 CM handover.
 *
 * The condition for the handover is the availability of bitsync on L1 C/A
 *
 * \param sat L1C/A Satellite ID
 * \param nap_channel Associated NAP channel
 * \param code_phase L1CA code phase [chips]
 */
void do_l1ca_to_l2cm_handover(u16 sat, u8 nap_channel, float code_phase)
{
  /* First, get L2C capability for the SV from NDB */
  u32 l2c_cpbl;
  // TODO: uncomment this as soon as NDB gets available
  // ndb_gps_l2cm_l2c_cap_read(&l2c_cpbl);
  // TODO: remove this as soon as NDB gets available
  l2c_cpbl = ~0;
  if (0 == (l2c_cpbl & ((u32)1 << sat))) {
    log_info("SV %u does not support L2C signal", sat);
    return;
  }

  if ((code_phase < 0) ||
      ((code_phase > 0.5) && (code_phase < (GPS_L1CA_CHIPS_NUM - 0.5)))) {
    log_warn("Unexpected L1C/A to L2C handover code phase: %f", code_phase);
    return;
  }

  if (code_phase > (GPS_L1CA_CHIPS_NUM - 0.5)) {
    code_phase = GPS_L2CM_CHIPS_NUM - (GPS_L1CA_CHIPS_NUM - code_phase);
  }

  log_warn("---- ADEL debug %s:%d", __FILE__, __LINE__);

  /* compose SID: same SV, but code is L2 CM */
  gnss_signal_t sid = {
    .sat  = sat,
    .code = CODE_GPS_L2CM
  };

  /* find available tracking channel first */
  s16 l2cm_channel_id = -1;

  for (u8 i = 0; i < nap_track_n_channels; i++) {
    /* if (tracker_channel_available(i, sid) && */
    /*     decoder_channel_available(i, sid)) { */
    if (tracker_channel_available(i, sid)) {
      l2cm_channel_id = i;
      log_warn("---- ADEL debug %s:%d", __FILE__, __LINE__);
      break;
    }
  }

  log_warn("---- ADEL debug %s:%d", __FILE__, __LINE__);

  if (-1 == l2cm_channel_id) {
    log_warn("No free channel for L2 CM tracking");
    return;
  }
  log_warn("---- ADEL debug %s:%d", __FILE__, __LINE__);

  /* free tracking channel found */
  u32 ref_sample_count = nap_timing_count();

  log_warn("---- ADEL debug %s:%d", __FILE__, __LINE__);

  /* recalculate doppler freq for L2 from L1*/
  double carrier_freq = tracking_channel_carrier_freq_get(nap_channel) *
                        GPS_L2_HZ / GPS_L1_HZ;

  log_debug("L2C Dopp %f", carrier_freq);
    log_warn("---- ADEL debug %s:%d", __FILE__, __LINE__);

  /* get initial cn0 from parent L1 channel */
  float cn0_init = tracking_channel_cn0_get(nap_channel);

    log_warn("---- ADEL debug %s:%d", __FILE__, __LINE__);

  s8 elevation = tracking_channel_evelation_degrees_get(nap_channel);
    log_warn("---- ADEL debug %s:%d", __FILE__, __LINE__);

  tracking_lock();
  /* Start the tracking channel */
  if (!tracker_channel_init((tracker_channel_id_t)l2cm_channel_id, sid,
                            ref_sample_count, code_phase,
                            carrier_freq, cn0_init, elevation)) {
    log_error("tracker channel init for L2 CM failed");
  } else {
    log_info("L2 CM handover done. Tracking channel %u, parent channel %u",
             (u8)l2cm_channel_id, nap_channel);
  }
  tracking_unlock();

    log_warn("---- ADEL debug %s:%d", __FILE__, __LINE__);

  /* Start the decoder channel */
  if (!decoder_channel_init((u8)l2cm_channel_id, sid)) {
    log_error("decoder channel init for L2 CM failed");
  }
    log_warn("---- ADEL debug %s:%d", __FILE__, __LINE__);
}
