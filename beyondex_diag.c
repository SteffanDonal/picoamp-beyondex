#include "beyondex_diag.h"

uint32_t i2s_dbg_get_underrun_count(void);
uint32_t i2s_dbg_get_overflow_count(void);
int32_t  i2s_get_buf_length(void);
int32_t  i2s_get_buf_us(void);

void beyondex_get_audio_diag(beyondex_audio_diag_t* out)
{
  out->magic    = 0x44584542u; // 'BEXD'
  out->underrun = i2s_dbg_get_underrun_count();
  out->overflow = i2s_dbg_get_overflow_count();
  out->buf_len  = i2s_get_buf_length();
  out->buf_us   = i2s_get_buf_us();
}
