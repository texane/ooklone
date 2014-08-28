#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static inline unsigned int pow2(unsigned int x)
{
  return 1 << x;
}

int main(int ac, char** av)
{
#define N_SWITCH 12 /* switch position count */
#define N_ADC 10 /* adc bits count */
#define V_REF 3.3

  const double adc_hi = (double)pow2(N_ADC);
  const double adc_step = adc_hi / (double)(N_SWITCH - 1);
  const double rdiv_2 = 10000;

  double ri[N_SWITCH];

  size_t i;

  /* start at i = 1, cf. notes */
  for (i = 1; i != N_SWITCH; ++i)
  {
    ri[i] = (adc_hi * rdiv_2) / (adc_step * (double)i) - rdiv_2;
  }

  /* fix ri[0] to ri[1] * 20 */
  ri[0] = ri[1] * 20.0;

  /* print values */
  for (i = 0; i != N_SWITCH; ++i)
  {
    const double vi = V_REF * rdiv_2 / (ri[i] + rdiv_2);
    const unsigned int adc = vi * adc_hi / V_REF;
    printf("i = %02zu, r = % 9.01lf, adc = 0x%08x, v = % 1.01lf\n", i, ri[i], adc, vi);
  }

  return 0;
}
