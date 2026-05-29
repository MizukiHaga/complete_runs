#ifndef LOGTEST_BATTERY_H_
#define LOGTEST_BATTERY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <kernel.h>
#include <stdio.h>

/* 出力先 FILE* を受け取り、内部状態を初期化する */
void Battery_Init(FILE *bt_out);

/* 周期タスク本体。app.cfg の CRE_CYC から定期的に起動される */
void battery_task(intptr_t unused);

#ifdef __cplusplus
}
#endif

#endif  // LOGTEST_BATTERY_H_
