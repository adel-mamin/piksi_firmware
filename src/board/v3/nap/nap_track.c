#include "nap/nap_common.h"
#include "nap/track_channel.h"

#include <ch.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

#include <assert.h>
#include <string.h>

#define SPACING_HALF_CHIP         ((u16)(FPGA_FREQ / GPS_CA_CHIPPING_RATE) / 2)

BSEMAPHORE_DECL(timing_strobe_sem, TRUE);

struct {
  u32 code_phase;
  s64 carrier_phase;
  u32 len;
} pipeline[NAP_MAX_N_TRACK_CHANNELS];

/** Calculate the future code phase after N samples.
 * Calculate the expected code phase in N samples time with carrier aiding.
 *
 * \param code_phase   Current code phase in chips.
 * \param carrier_freq Current carrier frequency (i.e. Doppler) in Hz used for
 *                     carrier aiding.
 * \param n_samples    N, the number of samples to propagate for.
 * \param code         Code identifier.
 *
 * \return The propagated code phase in chips.
 */
static double propagate_code_phase(double code_phase, double carrier_freq,
                                   u32 n_samples, code_t code)
{
  /* Calculate the code phase rate with carrier aiding. */
  double code_phase_rate = (1.0 + carrier_freq/code_to_carr_freq(code)) *
                           GPS_CA_CHIPPING_RATE;
  code_phase += n_samples * code_phase_rate / SAMPLE_FREQ;
  u32 cp_int = floor(code_phase);
  code_phase -= cp_int - (cp_int % code_to_chip_num(code));
  return code_phase;
}

static u32 calc_length_samples(code_t code_id, u8 codes, s32 cp_start, u32 cp_rate)
{
  u16 chips = codes * code_to_chip_num(code_id);
  u64 cp_units = (1ULL << NAP_TRACK_CODE_PHASE_FRACTIONAL_WIDTH) * chips
                 - cp_start;
  u32 samples = cp_units / cp_rate;
  return samples;
}

u16 sid_to_init_g1(gnss_signal_t sid)
{
  u16 ret;
  u32 gps_l2cm_prns_init_values[] = {
    /* 0  */ 0742417664,
    /* 1  */ 0756014035,
    /* 2  */ 0002747144,
    /* 3  */ 0066265724,
    /* 4  */ 0601403471,
    /* 5  */ 0703232733,
    /* 6  */ 0124510070,
    /* 7  */ 0617316361,
    /* 8  */ 0047541621,
    /* 9  */ 0733031046,
    /* 10 */ 0713512145,
    /* 11 */ 0024437606,
    /* 12 */ 0021264003,
    /* 13 */ 0230655351,
    /* 14 */ 0001314400,
    /* 15 */ 0222021506,
    /* 16 */ 0540264026,
    /* 17 */ 0205521705,
    /* 18 */ 0064022144,
    /* 19 */ 0120161274,
    /* 20 */ 0044023533,
    /* 21 */ 0724744327,
    /* 22 */ 0045743577,
    /* 23 */ 0741201660,
    /* 24 */ 0700274134,
    /* 25 */ 0010247261,
    /* 26 */ 0713433445,
    /* 27 */ 0737324162,
    /* 28 */ 0311627434,
    /* 29 */ 0710452007,
    /* 30 */ 0722462133,
    /* 31 */ 0050172213,
    /* 32 */ 0500653703,
    /* 33 */ 0755077436,
    /* 34 */ 0136717361,
    /* 35 */ 0756675453,
    /* 36 */ 0435506112
  };

  switch (sid.code) {

    case CODE_GPS_L2CM:
      ret = (u16) gps_l2cm_prns_init_values[sid.sat];
      break;

    case CODE_GPS_L1CA:
    default:
      ret = 0x3ff;
      break;
  }

  return ret;
}

u8 sid_to_rf_fronend_channel(gnss_signal_t sid)
{
  u8 ret;
  switch (sid.code) {
  case CODE_GPS_L1CA:
  case CODE_SBAS_L1CA:
    ret = 0;
    break;
  case CODE_GPS_L2CM:
    ret = 3;
    break;
  default:
    assert(0);
    break;
  }
  return ret;
}

u32 nap_track_init(u8 channel, gnss_signal_t sid, u32 ref_timing_count,
                   float carrier_freq, float code_phase)
{
  u32 now = NAP->TIMING_COUNT;
  u32 track_count = now + 200000;
  /* log_warn("---- ADEL debug %s:%d chan = %d prn = %d, carr_freq = %f, code_phase = %f", */
  /*          __FILE__, __LINE__, (int)channel, (int)(sid.sat - 1), carrier_freq, code_phase); */
  double cp = propagate_code_phase(code_phase, carrier_freq,
                                  track_count - ref_timing_count, sid.code);

  /* Contrive for the timing strobe to occur at or close to a
   * PRN edge (code phase = 0) */
  track_count += (SAMPLE_FREQ / GPS_CA_CHIPPING_RATE) *
                 (code_to_chip_num(sid.code) - cp) *
                 (1.0 + carrier_freq / code_to_carr_freq(sid.code));

  /* log_warn("---- ADEL debug %s:%d chan = %d prn = %d, carr_freq = %f, code_phase = %f", */
  /*          __FILE__, __LINE__, (int)channel, (int)(sid.sat - 1), carrier_freq, code_phase); */

  u8 prn = sid.sat - 1;
  u16 control = (prn << NAP_TRK_CONTROL_PRN_Pos) & NAP_TRK_CONTROL_PRN_Msk;
  control |= (sid_to_rf_fronend_channel(sid) << NAP_TRK_CONTROL_RF_FE_Pos) & \
             NAP_TRK_CONTROL_RF_FE_Msk;
  //control |= (1 << NAP_TRK_CONTROL_RF_FE_CH_Pos) & NAP_TRK_CONTROL_RF_FE_CH_Msk;

  log_warn("- ADEL debug %s:%d ch=%d prn=%d, cf=%f, cp=%f, ctrl=0x%04X",
           __FILE__, __LINE__, (int)channel, (int)(sid.sat - 1), carrier_freq, code_phase, control);

  NAP->TRK_CH[channel].CONTROL = control;
  log_warn("---- ADEL debug %s:%d", __FILE__, __LINE__);
  /* We always start at zero code phase */
  NAP->TRK_CH[channel].CODE_INIT_INT = 0;
  log_warn("---- ADEL debug %s:%d", __FILE__, __LINE__);
  NAP->TRK_CH[channel].CODE_INIT_FRAC = 0;
  log_warn("---- ADEL debug %s:%d", __FILE__, __LINE__);
  NAP->TRK_CH[channel].CODE_INIT_G1 = sid_to_init_g1(sid);
  log_warn("---- ADEL debug %s:%d", __FILE__, __LINE__);
  NAP->TRK_CH[channel].CODE_INIT_G2 = 0x3ff;
  log_warn("---- ADEL debug %s:%d", __FILE__, __LINE__);

  NAP->TRK_CH[channel].SPACING = (SPACING_HALF_CHIP << 16) | SPACING_HALF_CHIP;

  u32 cp_rate = (1.0 + carrier_freq / code_to_carr_freq(sid.code)) *
                GPS_CA_CHIPPING_RATE * NAP_TRACK_CODE_PHASE_RATE_UNITS_PER_HZ;

  NAP->TRK_CH[channel].CARR_PINC = -carrier_freq *
                                   NAP_TRACK_CARRIER_FREQ_UNITS_PER_HZ;
  NAP->TRK_CH[channel].CODE_PINC = cp_rate;

  log_warn("---- ADEL debug %s:%d", __FILE__, __LINE__);

  pipeline[channel].len = NAP->TRK_CH[channel].LENGTH =
    calc_length_samples(sid.code, 1, 0, cp_rate) + 1;
  pipeline[channel].code_phase = (NAP->TRK_CH[channel].LENGTH) * cp_rate;
  pipeline[channel].carrier_phase = NAP->TRK_CH[channel].LENGTH *
                                    (s64)-NAP->TRK_CH[channel].CARR_PINC;
  NAP->TRK_CONTROL |= (1 << channel); /* Set to start on the timing strobe */

  NAP->TRK_TIMING_COMPARE = track_count - SAMPLE_FREQ / GPS_CA_CHIPPING_RATE;

  log_warn("---- ADEL debug %s:%d", __FILE__, __LINE__);

  //chThdSleep(CH_CFG_ST_FREQUENCY * ceil((float)(track_count - now)/SAMPLE_FREQ));
  /* Spin until we're really running */
  while (!(NAP->TRK_CH[channel].STATUS & NAP_TRK_STATUS_RUNNING));

  log_warn("---- ADEL debug %s:%d", __FILE__, __LINE__);

  return NAP->TRK_CH[channel].START_SNAPSHOT;
}

void nap_track_update_wr_blocking(code_t code, u8 channel, s32 carrier_freq,
                                  u32 code_phase_rate, u8 rollover_count,
                                  u8 corr_spacing)
{
  (void)corr_spacing; /* This is always written as 0 now... */

  NAP->TRK_CH[channel].CARR_PINC = -carrier_freq;
  NAP->TRK_CH[channel].CODE_PINC = code_phase_rate;
  NAP->TRK_CH[channel].LENGTH = calc_length_samples(code, rollover_count + 1,
                                    pipeline[channel].code_phase,
                                    code_phase_rate);

  volatile u16 cp_int = NAP->TRK_CH[channel].CODE_PHASE_INT;
  if ((cp_int != 0x3fe)) /* Sanity check, we should be just before rollover */
    asm("nop");
}

void nap_track_corr_rd_blocking(u8 channel,
                                u32* count_snapshot, corr_t corrs[],
                                double *code_phase_early,
                                double *carrier_phase)
{
  corr_t lc[5];
  if (NAP->TRK_CH[channel].STATUS & 0x3F)
    log_error("Track correlator overflow 0x%08X on channel %d",
              NAP->TRK_CH[channel].STATUS, channel);
  for (u8 i = 0; i < 5; i++) {
    lc[i].I = NAP->TRK_CH[channel].CORR[i].I >> 8;
    lc[i].Q = NAP->TRK_CH[channel].CORR[i].Q >> 8;
  }
  *count_snapshot = NAP->TRK_CH[channel].START_SNAPSHOT;
  u64 nap_phase = ((u64)NAP->TRK_CH[channel].CODE_PHASE_INT << 32) |
                        NAP->TRK_CH[channel].CODE_PHASE_FRAC;
  *code_phase_early = (double)nap_phase / NAP_TRACK_CODE_PHASE_UNITS_PER_CHIP;
  *carrier_phase = (double)pipeline[channel].carrier_phase / (1ull << 32);
  memcpy(corrs, &lc[1], sizeof(corr_t)*3);
  if (pipeline[channel].code_phase != NAP->TRK_CH[channel].CODE_PHASE_FRAC)
    asm("nop");
  pipeline[channel].code_phase +=
    NAP->TRK_CH[channel].LENGTH * NAP->TRK_CH[channel].CODE_PINC;
  pipeline[channel].carrier_phase -=
    NAP->TRK_CH[channel].LENGTH * (s64)NAP->TRK_CH[channel].CARR_PINC;
  pipeline[channel].len = NAP->TRK_CH[channel].LENGTH;
  if (!(NAP->STATUS & 1))
    asm("bkpt");
}

void nap_track_disable(u8 channel)
{
  NAP->TRK_CONTROL &= ~(1 << channel); /* Set to start on the timing strobe */
}

