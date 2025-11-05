#pragma once
#include <cstdlib>
#include <cmath>

/** [fMin, fMax] 난수 */
inline double randomDouble(double fMin, double fMax)
{
  const double f = static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX);
  return fMin + f * (fMax - fMin);
}
