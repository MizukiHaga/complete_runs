#ifndef TESTAPP_MORTER_H_
#define TESTAPP_MORTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <kernel.h>
#include <pbio/port.h>
#include <stdio.h>

/* 周期タスクで左右同じパワーを出して直進するか。0 なら指令なし（ログのみ）。 */
#ifndef MORTER_ENABLE_DRIVE
#define MORTER_ENABLE_DRIVE  (1)
#endif

/* 直進時のモータ出力 (-100〜100)。左右同じ値にする。 */
#define MORTER_LEFT_POWER   (50)
#define MORTER_RIGHT_POWER  (50)

void Morter_Configure(pbio_port_id_t left_motor_port, pbio_port_id_t right_motor_port);
void Morter_SetOutput(FILE *fp);
void morter_task(intptr_t unused);

#ifdef __cplusplus
}
#endif

#endif  // TESTAPP_MORTER_H_
