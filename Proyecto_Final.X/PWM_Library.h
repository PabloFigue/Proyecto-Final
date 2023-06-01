/* 
 * File:   PWM_Library.h
 * Author: pablo
 *
 * Created on 15 de mayo de 2023, 12:18 a.m.
 */

#ifndef PWM_LIBRARY_H
#define	PWM_LIBRARY_H

#ifdef	__cplusplus
extern "C" {
#endif

void PWM_config(int canal, float periodo_ms);
void PWM_duty (int canal, int DutyCycle);
void PWM_manual_config(float periodo_ms);
void PWM_manual(int limite_pot, int puerto);


#ifdef	__cplusplus
}
#endif

#endif	/* PWM_LIBRARY_H */

