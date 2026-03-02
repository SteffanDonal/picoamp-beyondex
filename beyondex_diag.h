#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct __attribute__((packed)) {
  uint32_t magic; // 'BEXD' = 0x44584542
  uint32_t underrun; // i2s_dbg_get_underrun_count()
  uint32_t overflow; // i2s_dbg_get_overflow_count()
  int32_t  buf_len; // i2s_get_buf_length()
  int32_t  buf_us; // i2s_get_buf_us()
  int32_t feedback; // feedback value
  uint32_t sof_calls; // times sof callback called
  uint32_t sof_updates; // times sof callback sets feedback
  uint16_t streaming_enabled; // streaming enabled flag
} beyondex_audio_diag_t;

void beyondex_get_audio_diag(beyondex_audio_diag_t* out);

static inline void beyondex_get_audio_diag_payload(
  uint8_t out[sizeof(beyondex_audio_diag_t)],
  int32_t feedback,
  uint32_t sof_calls,
  uint32_t sof_updates,
  uint16_t streaming_enabled
)
{
  beyondex_audio_diag_t d;
  beyondex_get_audio_diag(&d);
  d.feedback = feedback;
  d.sof_calls = sof_calls;
  d.sof_updates = sof_updates;
  d.streaming_enabled = streaming_enabled;
  __builtin_memcpy(out, &d, sizeof(d));
}

#ifdef __cplusplus
}
#endif
