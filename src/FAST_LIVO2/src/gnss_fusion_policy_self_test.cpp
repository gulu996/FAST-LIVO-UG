#include "gnss_fusion_policy.h"

#include <cassert>
#include <iostream>

int main()
{
  GnssFusionPolicy legacy;
  assert(legacy.componentEnabled(true));
  assert(!legacy.componentEnabled(false));

  GnssFusionPolicy disabled;
  disabled.managed = true;
  disabled.enabled = false;
  assert(!disabled.componentEnabled(true));

  GnssFusionPolicy enabled;
  enabled.managed = true;
  enabled.enabled = true;
  assert(enabled.componentEnabled(true));

  std::cout << "gnss_fusion_policy_self_test: PASS\n";
  return 0;
}
