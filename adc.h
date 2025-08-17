/* 
 * File:   adc.h
 * Author: DG4SN
 *
 * Created on 29. M�rz 2022, 17:57
 */

#ifndef ADC_H
#define	ADC_H

#ifdef	__cplusplus
extern "C"
{
#endif

  
extern int ADC_r_raw;
extern int ADC_f_raw;
  
  void ADC_Init(void);
  void ADC_Run(void);
  


#ifdef	__cplusplus
}
#endif

#endif	/* ADC_H */

