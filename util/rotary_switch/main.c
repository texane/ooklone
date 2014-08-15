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
  const double adc_step = adc_hi / (double)N_SWITCH;
  const double rdiv_2 = 1000;

  double ri[N_SWITCH];

  size_t i;

  /* start at i = 1, cf. notes */
  for (i = 1; i != N_SWITCH; ++i)
  {
    ri[i] = (adc_hi * rdiv_2) / (adc_step * (double)i) - rdiv_2;
  }

  /* fix ri[0] to ri[1] * 10 */
  ri[0] = ri[1] * 10.0;

  /* print values */
  for (i = 0; i != N_SWITCH; ++i)
  {
    const double vi = V_REF * rdiv_2 / (ri[i] + rdiv_2);
    printf("i = %02zu r_i = % 9.01lf, v_i = % 1.01lf\n", i, ri[i], vi);
  }

  return 0;
}
