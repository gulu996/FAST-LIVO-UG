#include "gnss_fusion_policy.h"

#include <cassert>
#include <iostream>

int main()
{
  GnssFusionPolicy legacy;
  assert(legacy.componentEnabled(true));
  assert(!legacy.componentEnabled(false));
  assert(legacy.absoluteUpdateEnabled(true));

  GnssFusionPolicy disabled;
  disabled.managed = true;
  disabled.enabled = false;
  assert(!disabled.componentEnabled(true));
  assert(disabled.absoluteUpdateEnabled(true));

  GnssFusionPolicy enabled;
  enabled.managed = true;
  enabled.enabled = true;
  assert(enabled.componentEnabled(true));
  assert(!enabled.absoluteUpdateEnabled(true));
  assert(!enabled.absoluteUpdateEnabled(false));

  std::cout << "gnss_fusion_policy_self_test: PASS\n";
  return 0;
}
