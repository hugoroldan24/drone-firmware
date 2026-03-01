#ifndef PWM_H_
#define PWM_H_

void pwm_init();
void start_PWM();
void update_PWM_Outputs(TIM_TypeDef* timer, PWM_Outputs_t ccr);


#endif /* PWM_H_ */

