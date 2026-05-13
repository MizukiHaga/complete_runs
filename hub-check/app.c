#include "kernel_cfg.h"
#include "app.h"

#include <spike/hub/display.h>
#include <spike/hub/speaker.h>
#include <spike/hub/system.h>

/* メインタスク(起動時にのみ関数コールされる) */
void main_task(intptr_t unused) {

  
  hub_display_text_scroll("GO SPIKE-RT!", 60);
  dly_tsk(3*1000*1000);
  hub_system_shutdown();

  /* タスク終了 */
  ext_tsk();
}
