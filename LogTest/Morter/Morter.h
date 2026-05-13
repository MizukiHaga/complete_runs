
#ifndef TESTAPP_MORTER_H_
#define TESTAPP_MORTER_H_

#ifdef __cplusplus
extern "C" {
#endif


#include <kernel.h>
#include <pbio/port.h>
#include <stdio.h>

/*
 * 周期タスクでモータへ出力指令 (pup_motor_set_power) を書き込むか。
 * 0: 指令なし（角度ログのみ向き）。1: 左右パワーを毎周期セット。
 */
#ifndef MORTER_ENABLE_DRIVE
#define MORTER_ENABLE_DRIVE  (0)
#endif

/*
 * MORTER_ENABLE_DRIVE が 1 のときのモータ出力 (-100〜100)。
 */
#define MORTER_LEFT_POWER   (100)
#define MORTER_RIGHT_POWER  (100)

void Morter_Configure(pbio_port_id_t left_motor_port, pbio_port_id_t right_motor_port);
void morter_task(intptr_t unused);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_MORTER_H_
