#include "app.h"
#include "Battery/Battery.h"

#include <spike/hub/battery.h>

/* Bluetooth シリアルの出力先（Battery_Init で受け取る） */
static FILE *output_file = nullptr;

void Battery_Init(FILE *bt_out)
{
  output_file = bt_out;
}

/*
 * バッテリー残量（電圧 / 電流）をログに出す。
 *   BAT_voltage,<mV>
 *   BAT_current,<mA>
 *
 * 電圧の目安（Prime Hub のリチウム電池）:
 *   約 8400 mV → ほぼ満充電
 *   約 6000 mV → かなり減っている（充電推奨）
 */
void battery_task(intptr_t unused)
{
  if (!output_file) {
    return;
  }
  const uint16_t voltage_mv = hub_battery_get_voltage();
  const uint16_t current_ma = hub_battery_get_current();
  fprintf(output_file, "BAT_voltage,%u\n", (unsigned)voltage_mv);
  fprintf(output_file, "BAT_current,%u\n", (unsigned)current_ma);
}
