// =============================================================================
// ATtiny1614 Tracked-Lag Clock Generator — BBD / Geological Aliasing
// =============================================================================
// Hardware:
//   - MCU    : ATtiny1614, 20 MHz internal oscillator, megaTinyCore
//   - Input  : USVCO square wave, 50–150 kHz  →  PA2 (TCB0 event capture)
//   - Output : Detuned clock to MN3102         →  PB0 (TCA0 WO0 default)
//   - Lag CV : 0–5V pot                        →  PA5 (ADC AIN5)
//   - UPDI   : PA0
//
// Signal pipeline:
//   Period measurement : TCB0 FRQMEAS, PA2 rising edge → Tin (20 MHz counts)
//   Lag control        : ADC PA5 → EMA → delta_hz (0–250 Hz)
//   Frequency compute  : f_out = f_in - delta_hz
//                        T_out_half computed in Q8 fixed-point
//   Sigma-delta output : fractional counts accumulated each cycle;
//                        CMP0 toggled ±1 to average the exact target frequency
//   Hardware toggle    : TCA0 FRQ mode, PB0 toggles on CMP0 match — no ISR
//
// Effective BBD clock = PB0 output / 2  (MN3102 internal ÷2)
//
// At delta_hz = 0:   f_out tracks f_in exactly (÷1)
// At delta_hz = 250: f_out = f_in - 250 Hz
//   e.g. f_in = 100 kHz  →  f_out = 99.75 kHz  →  BBD clock = 49.875 kHz
// =============================================================================

#include <avr/io.h>
#include <avr/interrupt.h>

// ---------------------------------------------------------------------------
// Pin / channel constants
// ---------------------------------------------------------------------------
#define INPUT_PIN_bm   PIN2_bm              // PA2 — TCB0 event capture
#define OUTPUT_PIN_bm  PIN0_bm              // PB0 — TCA0 WO0 (default position)
#define LAG_AIN        ADC_MUXPOS_AIN5_gc  // PA5 — lag/delta pot

// ---------------------------------------------------------------------------
// Timing guard-rails (20 MHz timer counts)
//   50 kHz → 400 counts,  150 kHz → 133 counts
//   20 % headroom: accept 110 … 480 counts
// ---------------------------------------------------------------------------
#define TIN_MIN   110u
#define TIN_MAX   480u
#define CMP_MIN   2u
#define CMP_MAX   65534u

// ---------------------------------------------------------------------------
// Frequency offset scaling
//   156250 = (2 × F_CPU) / 256 = 40 000 000 / 256
//   Used to compute the Q8 fractional correction to T_out_half:
//     correction_q8 = delta_hz × Tin² / 156250
//   Max: 250 × 480² / 156250 = 369  →  1.44 extra counts (fits in uint32_t)
// ---------------------------------------------------------------------------
#define F_DENOM   156250UL

// ---------------------------------------------------------------------------
// Shared state — written in ISR, read in main loop
// ---------------------------------------------------------------------------
volatile uint16_t tin_counts  = 200;   // default ≈ 100 kHz
volatile bool     tin_updated = false;

// ---------------------------------------------------------------------------
// ISR — TCB0 capture (fires on every rising edge of the USVCO input)
// ---------------------------------------------------------------------------
ISR(TCB0_CAPT_vect)
{
    // Reading CCMP clears the interrupt flag automatically.
    // In FRQMEAS mode CCMP = edge-to-edge period in 20 MHz timer ticks.
    uint16_t t = TCB0.CCMP;
    if (t >= TIN_MIN && t <= TIN_MAX) {
        tin_counts  = t;
        tin_updated = true;
    }
}

// ---------------------------------------------------------------------------
// TCA0 — Frequency generation on WO0 (PB0, default mux — no PORTMUX needed)
//
//   FRQ mode: counter counts 0 … CMP0, toggles PB0, resets.
//   Output half-period = CMP0 + 1 timer ticks.
//   CMP0BUF is double-buffered; writes latch at the next period boundary.
// ---------------------------------------------------------------------------
static void setup_tca0(void)
{
    PORTB.DIRSET = OUTPUT_PIN_bm;

    // Default: Tin=200 (100 kHz), delta=0 → Tout_half=100 → CMP0=99
    TCA0.SINGLE.CMP0BUF = 99;
    TCA0.SINGLE.PERBUF  = 99;

    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_FRQ_gc
                      | TCA_SINGLE_CMP0EN_bm;
    TCA0.SINGLE.CNT   = 0;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc
                      | TCA_SINGLE_ENABLE_bm;
}

// ---------------------------------------------------------------------------
// TCB0 — Period measurement, FRQMEAS mode, PA2 via event system
// ---------------------------------------------------------------------------
static void setup_tcb0(void)
{
    PORTA.DIRCLR = INPUT_PIN_bm;

    EVSYS.ASYNCCH0   = EVSYS_ASYNCCH0_PORTA_PIN2_gc;
    EVSYS.ASYNCUSER9 = EVSYS_ASYNCUSER09_ASYNCCH0_gc;  // TCB0 ← ASYNCCH0

    TCB0.EVCTRL  = TCB_CAPTEI_bm;
    TCB0.CTRLB   = TCB_CNTMODE_FRQ_gc;
    TCB0.INTCTRL = TCB_CAPT_bm;
    TCB0.CTRLA   = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
}

// ---------------------------------------------------------------------------
// ADC — single-ended, 10-bit, VDD reference, PA5
// ---------------------------------------------------------------------------
static void setup_adc(void)
{
    ADC0.CTRLC = ADC_PRESC_DIV8_gc
               | ADC_REFSEL_VDDREF_gc
               | ADC_SAMPCAP_bm;
    ADC0.CTRLA = ADC_ENABLE_bm;
}

static inline uint16_t adc_read(uint8_t muxpos)
{
    ADC0.MUXPOS  = muxpos;
    ADC0.COMMAND = ADC_STCONV_bm;
    while (ADC0.COMMAND & ADC_STCONV_bm);
    return ADC0.RES;
}

// ===========================================================================
// Arduino entry points
// ===========================================================================

void setup(void)
{
    setup_adc();
    setup_tca0();
    setup_tcb0();
    sei();
}

void loop(void)
{
    if (!tin_updated) return;
    tin_updated = false;

    uint16_t tin = tin_counts;

    // -----------------------------------------------------------------------
    // ADC + EMA (alpha ≈ 1/8) → delta_hz (0–250 Hz)
    //
    //   10-bit ADC → 0–1023
    //   delta_hz = adc_ema × 125 >> 9  ≈  adc_ema × 250 / 1023
    //   At full scale: 1023 × 125 / 512 = 249.75 ≈ 250 Hz
    // -----------------------------------------------------------------------
    static uint16_t lag_ema = 0;
    uint16_t lag_raw = adc_read(LAG_AIN);
    lag_ema = lag_ema - (lag_ema >> 3) + (lag_raw >> 3);

    uint16_t delta_hz = (uint16_t)(((uint32_t)lag_ema * 125u) >> 9);

    // -----------------------------------------------------------------------
    // Compute T_out_half in Q8 fixed-point
    //
    //   f_out = f_in - delta_hz = F_CPU/Tin - delta_hz
    //   T_out = F_CPU / f_out
    //   T_out_half = F_CPU / (2 × f_out)
    //
    //   First-order approximation (accurate for delta_hz << f_in):
    //     T_out_half ≈ Tin/2 + delta_hz × Tin² / (2 × F_CPU)
    //
    //   In Q8 (×256):
    //     tout_half_q8 = (Tin << 7) + delta_hz × Tin² / 156250
    //
    //   At Tin=480, delta=250: correction = 369 Q8 counts = 1.44 timer ticks
    // -----------------------------------------------------------------------
    uint32_t tin_sq        = (uint32_t)tin * tin;
    uint32_t correction_q8 = ((uint32_t)delta_hz * tin_sq) / F_DENOM;
    uint32_t tout_half_q8  = ((uint32_t)tin << 7) + correction_q8;

    // -----------------------------------------------------------------------
    // Sigma-delta dithering
    //
    //   The integer timer can only step in whole counts (~300–1500 Hz per
    //   step at these frequencies). The sigma-delta accumulates the fractional
    //   part of T_out_half and adds 1 to CMP0 on overflow, averaging the
    //   output to exactly the target frequency over many cycles.
    // -----------------------------------------------------------------------
    static uint16_t sd_accum = 0;

    uint16_t cmp  = (uint16_t)(tout_half_q8 >> 8);   // integer part
    uint8_t  frac = (uint8_t)tout_half_q8;            // fractional part 0–255

    sd_accum += frac;
    if (sd_accum >= 256u) {
        cmp++;
        sd_accum -= 256u;
    }

    if (cmp < CMP_MIN)  cmp = CMP_MIN;
    if (cmp > CMP_MAX)  cmp = CMP_MAX;

    // Latch at next period boundary — glitch-free.
    TCA0.SINGLE.CMP0BUF = cmp - 1;
}
