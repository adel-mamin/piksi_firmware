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

static struct loop_params {
  float code_bw, code_zeta, code_k, carr_to_code;
  float carr_bw, carr_zeta, carr_k, carr_fll_aid_gain;
  u8 coherent_ms;
} loop_params_stage;

static struct lock_detect_params {
  float k1, k2;
  u16 lp, lo;
} lock_detect_params;

static float track_cn0_use_thres = 31.0; /* dBHz */
static float track_cn0_drop_thres = 31.0; /* dBHz */

static char loop_params_string[120] = LOOP_PARAMS_MED;
static char lock_detect_params_string[24] = LD_PARAMS;
static bool use_alias_detection = true;

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

static tracker_t gps_l2cm_trackers[NUM_GPS_L2CM_TRACKERS];
static gps_l2cm_tracker_data_t gps_l2cm_tracker_data[NUM_GPS_L2CM_TRACKERS];

static void tracker_gps_l2cm_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data);
static void tracker_gps_l2cm_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data);
static void tracker_gps_l2cm_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data);

static bool parse_loop_params(struct setting *s, const char *val);
static bool parse_lock_detect_params(struct setting *s, const char *val);

static const tracker_interface_t tracker_interface_gps_l2cm = {
  .code =         CODE_GPS_L2CM,
  .init =         tracker_gps_l2cm_init,
  .disable =      tracker_gps_l2cm_disable,
  .update =       tracker_gps_l2cm_update,
  .trackers =     gps_l2cm_trackers,
  .num_trackers = NUM_GPS_L2CM_TRACKERS
};

static tracker_interface_list_element_t
  tracker_interface_list_element_gps_l2cm = {
    .interface = &tracker_interface_gps_l2cm,
    .next = 0
  };

/** Register L2 CM tracker into the the tracker interface & settings
 *  framework.
 */
void track_gps_l2cm_register(void)
{
  SETTING_NOTIFY(L2CM_TRACK_SETTING_SECTION, "loop_params",
                 loop_params_string,
                 TYPE_STRING, parse_loop_params);

  SETTING_NOTIFY(L2CM_TRACK_SETTING_SECTION, "lock_detect_params",
                 lock_detect_params_string,
                 TYPE_STRING, parse_lock_detect_params);

  SETTING(L2CM_TRACK_SETTING_SECTION, "cn0_use",
          track_cn0_use_thres, TYPE_FLOAT);

  SETTING(L2CM_TRACK_SETTING_SECTION, "cn0_drop",
          track_cn0_drop_thres, TYPE_FLOAT);

  SETTING(L2CM_TRACK_SETTING_SECTION, "alias_detect",
          use_alias_detection, TYPE_BOOL);

  for (u32 i = 0; i < NUM_GPS_L2CM_TRACKERS; i++) {
    gps_l2cm_trackers[i].active = false;
    gps_l2cm_trackers[i].data = &gps_l2cm_tracker_data[i];
  }

  tracker_interface_register(&tracker_interface_list_element_gps_l2cm);
}

static void tracker_gps_l2cm_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data)
{
  (void)channel_info;
  gps_l2cm_tracker_data_t *data = tracker_data;

  memset(data, 0, sizeof(gps_l2cm_tracker_data_t));
  tracker_ambiguity_unknown(channel_info->context);

  const struct loop_params *l = &loop_params_stage;

  data->int_ms = l->coherent_ms;

  aided_tl_init(&(data->tl_state), 1e3 / data->int_ms,
                common_data->code_phase_rate - GPS_CA_CHIPPING_RATE,
                l->code_bw, l->code_zeta, l->code_k,
                l->carr_to_code,
                common_data->carrier_freq,
                l->carr_bw, l->carr_zeta, l->carr_k,
                l->carr_fll_aid_gain);

  data->code_phase_rate_fp = common_data->code_phase_rate *
                             NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;
  data->carrier_freq_fp = (s32)(common_data->carrier_freq *
                                NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ);
  data->short_cycle = true;

  /* Initialise C/N0 estimator */
  cn0_est_init(&data->cn0_est, 1e3 / data->int_ms, common_data->cn0,
               CN0_EST_LPF_CUTOFF, 1e3 / data->int_ms);

  /* Initialize lock detector */
  lock_detect_init(&data->lock_detect,
                   lock_detect_params.k1, lock_detect_params.k2,
                   lock_detect_params.lp, lock_detect_params.lo);

  /* TODO: Reconfigure alias detection between stages */
  u8 alias_detect_ms = l->coherent_ms;
  alias_detect_init(&data->alias_detect,
                    L2C_ALIAS_DETECT_INTERVAL_MS / alias_detect_ms,
                    (alias_detect_ms - 1) * 1e-3);

}

static void tracker_gps_l2cm_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data)
{
  (void)channel_info;
  (void)common_data;
  (void)tracker_data;
}

static void tracker_gps_l2cm_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data)
{
  gps_l2cm_tracker_data_t *data = tracker_data;

  /* Read early ([0]), prompt ([1]) and late ([2]) correlations. */
  if (data->short_cycle) {
    tracker_correlations_read(channel_info->context, data->cs,
                              &data->corr_sample_count,
                              &common_data->code_phase_early,
                              &common_data->carrier_phase);
    alias_detect_first(&data->alias_detect, data->cs[1].I, data->cs[1].Q);
  } else {
    /* This is the end of the long cycle's correlations. */
    corr_t cs[3];
    tracker_correlations_read(channel_info->context, cs,
                              &data->corr_sample_count,
                              &common_data->code_phase_early,
                              &common_data->carrier_phase);
    /* Accumulate short cycle correlations with long ones. */
    for(int i = 0; i < 3; i++) {
      data->cs[i].I += cs[i].I;
      data->cs[i].Q += cs[i].Q;
    }
  }

  u8 int_ms = data->short_cycle ? 1 : (data->int_ms - 1);
  common_data->TOW_ms = tracker_tow_update(channel_info->context,
                                           common_data->TOW_ms,
                                           int_ms);

  /* We're doing long integrations, alternate between short and long
   * cycles. This is because of FPGA pipelining and latency.
   * The loop parameters can only be updated at the end of the second
   * integration interval.
   */
  bool short_cycle = data->short_cycle;

  data->short_cycle = !data->short_cycle;

  if (short_cycle) {
    tracker_retune(channel_info->context, data->carrier_freq_fp,
                   data->code_phase_rate_fp, 0);
    return;
  }

  common_data->update_count += data->int_ms;

  tracker_bit_sync_update(channel_info->context, data->int_ms, data->cs[1].I);

  corr_t* cs = data->cs;

  /* Update C/N0 estimate */
  common_data->cn0 = cn0_est(&data->cn0_est,
                            cs[1].I/data->int_ms, cs[1].Q/data->int_ms);
  if (common_data->cn0 > track_cn0_drop_thres) {
    common_data->cn0_above_drop_thres_count = common_data->update_count;
  }

  if (common_data->cn0 < track_cn0_use_thres) {
    /* SNR has dropped below threshold, indicate that the carrier phase
     * ambiguity is now unknown as cycle slips are likely. */
    tracker_ambiguity_unknown(channel_info->context);
    /* Update the latest time we were below the threshold. */
    common_data->cn0_below_use_thres_count = common_data->update_count;
  }

  /* Update PLL lock detector */
  bool last_outp = data->lock_detect.outp;
  lock_detect_update(&data->lock_detect, cs[1].I, cs[1].Q, data->int_ms);
  if (data->lock_detect.outo) {
    common_data->ld_opti_locked_count = common_data->update_count;
  }
  if (!data->lock_detect.outp) {
    common_data->ld_pess_unlocked_count = common_data->update_count;
  }

  /* Reset carrier phase ambiguity if there's doubt as to our phase lock */
  if (last_outp && !data->lock_detect.outp) {
    log_info_sid(channel_info->sid, "PLL stress");
    tracker_ambiguity_unknown(channel_info->context);
  }

  /* Run the loop filters. */

  /* Output I/Q correlations using SBP if enabled for this channel */
  tracker_correlations_send(channel_info->context, cs);

  correlation_t cs2[3];
  for (u32 i = 0; i < 3; i++) {
    cs2[i].I = cs[2-i].I;
    cs2[i].Q = cs[2-i].Q;
  }

  aided_tl_update(&data->tl_state, cs2);
  common_data->carrier_freq = data->tl_state.carr_freq;
  common_data->code_phase_rate = data->tl_state.code_freq +
                                 GPS_CA_CHIPPING_RATE;

  data->code_phase_rate_fp = common_data->code_phase_rate *
                             NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;

  data->carrier_freq_fp = common_data->carrier_freq *
                          NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ;

  /* Attempt alias detection if we have pessimistic phase lock detect OR
     optimistic phase lock detect */
  if (use_alias_detection &&
     (data->lock_detect.outp || data->lock_detect.outo)) {
    s32 I = (cs[1].I - data->alias_detect.first_I) / (data->int_ms - 1);
    s32 Q = (cs[1].Q - data->alias_detect.first_Q) / (data->int_ms - 1);
    float err = alias_detect_second(&data->alias_detect, I, Q);
    if (fabs(err) > (250 / data->int_ms)) {
      if (data->lock_detect.outp) {
        log_warn_sid(channel_info->sid, "False phase lock detected");
      }

      tracker_ambiguity_unknown(channel_info->context);
      /* Indicate that a mode change has occurred. */
      common_data->mode_change_count = common_data->update_count;

      data->tl_state.carr_freq += err;
      data->tl_state.carr_filt.y = data->tl_state.carr_freq;
    }
  }

  /* Must have (at least optimistic) phase lock */
  /* Must have nav bit sync, and be correctly aligned */
  if ((data->lock_detect.outo) &&
      tracker_bit_aligned(channel_info->context)) {
    log_info_sid(channel_info->sid, "synced @ %u ms, %.1f dBHz",
                 (unsigned int)common_data->update_count,
                 common_data->cn0);
    /* Indicate that a mode change has occurred. */
    common_data->mode_change_count = common_data->update_count;
  }

  tracker_retune(channel_info->context, data->carrier_freq_fp,
                 data->code_phase_rate_fp,
                 data->int_ms);
}

/** Parse a string describing the tracking loop filter parameters into
 *  the loop_params_stage struct.
 *
 * \param s Settings structure provided to store the input string.
 * \param val The input string to parse.
 * \retval true Success
 * \retval false Failure
 */
static bool parse_loop_params(struct setting *s, const char *val)
{
  /** The string contains loop parameters for one stage */

  struct loop_params loop_params_parse;

  const char *str = val;
  struct loop_params *l = &loop_params_parse;

  unsigned int tmp; /* newlib's sscanf doesn't support hh size modifier */

  if (sscanf(str, "( %u ms , ( %f , %f , %f , %f ) , ( %f , %f , %f , %f ) ) ",
             &tmp,
             &l->code_bw, &l->code_zeta, &l->code_k, &l->carr_to_code,
             &l->carr_bw, &l->carr_zeta, &l->carr_k, &l->carr_fll_aid_gain
             ) < 9) {
    log_error("Ill-formatted tracking loop param string: %20s", str);
    return false;
  }
  l->coherent_ms = tmp;

  if (l->coherent_ms != L2C_COHERENT_INTEGRATION_TIME_MS) {
    log_error("Invalid coherent integration length for L2CM: %" PRIu8,
              l->coherent_ms);
    return false;
  }
  /* Successfully parsed the input. Save to memory. */
  strncpy(s->addr, val, s->len);
  if (s->len > 0) {
    char *ptr = (char*) s->addr;
    ptr[s->len - 1] = '\0';
  }
  memcpy(&loop_params_stage, &loop_params_parse, sizeof(loop_params_stage));

  return true;
}

/** Parse a string describing the tracking loop phase lock detector
 *  parameters into the lock_detect_params structs.
 *
 * \param s Settings structure provided to store the input string.
 * \param val The input string to parse.
 * \retval true Success
 * \retval false Failure
 */
static bool parse_lock_detect_params(struct setting *s, const char *val)
{
  struct lock_detect_params p;

  if (sscanf(val, "%f , %f , %" SCNu16 " , %" SCNu16,
             &p.k1, &p.k2, &p.lp, &p.lo) < 4) {
      log_error("Ill-formatted lock detect param string: %20s", val);
      return false;
  }
  /* Successfully parsed the input. Save to memory. */
  strncpy(s->addr, val, s->len);
  if (s->len > 0) {
    char *ptr = (char*) s->addr;
    ptr[s->len - 1] = '\0';
  }
  memcpy(&lock_detect_params, &p, sizeof(lock_detect_params));

  return true;
}
