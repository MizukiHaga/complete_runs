#ifndef FIXEDRUN_H
#define FIXEDRUN_H

/* 決め打ち走行。L/R は ETROBO_COURSE_L で切り替える。
 * Complete: シーン8（R/L は ETROBO_COURSE_L）
 * Short:    シーン9（R/L は ETROBO_COURSE_L） */
class FixedRunComplete {
public:
  void run();
};

class FixedRunShort {
public:
  void run();
};

#endif
