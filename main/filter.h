#pragma once
void filter_old(void *args);
void process_block(const int32_t *in_buf, int32_t *M_Buf, int32_t *H_L_Buf, size_t frames);
