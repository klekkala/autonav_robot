#include<lm4f120h5qr.h>
#include<math.h>

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

int left_count_a=0;
int right_count_a=0;
//int left_count_b=0;
//int right_count_b=0;


//motor_driver_port_0123
void PortD_Init(void)
{
		volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000008;
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTD_LOCK_R = 0x4C4F434B;
  GPIO_PORTD_CR_R |= 0x0F;
  GPIO_PORTD_AMSEL_R &= 0x00;
  GPIO_PORTD_PCTL_R &= 0x00000000;
  GPIO_PORTD_DIR_R |= 0x0F;
  GPIO_PORTD_AFSEL_R &= 0x00;
  GPIO_PORTD_DEN_R |= 0x0F;

}
// cliff avoiding senors_1234
void PortC_Init(void);
//obstacle avoiding sensors_1234
void PortB_Init(void);

// testing motors 04
void PortF_Init(void)
{
		volatile unsigned long delay1;
  SYSCTL_RCGC2_R |= 0x00000020;
  delay1 = SYSCTL_RCGC2_R;
  GPIO_PORTF_LOCK_R = 0x4C4F434B;
  GPIO_PORTF_AMSEL_R &= 0x00;
  GPIO_PORTF_PCTL_R &= 0x000F0000;
  GPIO_PORTF_DIR_R &= ~0x10;
  GPIO_PORTF_AFSEL_R &= 0x10;
  GPIO_PORTF_DEN_R |= 0x10;
  GPIO_PORTF_PUR_R |=0x10;
}

//interrupts are activated in this port
void PortE_Init(void)
{
  SYSCTL_RCGC2_R |= 0x00000010;
  GPIO_PORTE_LOCK_R = 0x4C4F434B;
  GPIO_PORTE_CR_R |= 0x01;
  GPIO_PORTE_AMSEL_R =0;
  GPIO_PORTE_PCTL_R &= ~0x0000000F;
  GPIO_PORTE_DIR_R &= ~0x30;
  GPIO_PORTE_AFSEL_R &= ~0x30;
  GPIO_PORTE_DEN_R |= 0x30;
  GPIO_PORTE_PUR_R |=0x30;
  GPIO_PORTE_IS_R &= ~0x30;
  GPIO_PORTE_IBE_R |= 0x30;
  GPIO_PORTE_IEV_R &= ~0x30;
  GPIO_PORTE_ICR_R= 0x30;
  GPIO_PORTE_IM_R |=0x30;
  NVIC_PRI7_R=(NVIC_PRI7_R&0xFF00FFFF)|0x00A00000;
  NVIC_EN0_R=0x00000010;
  EnableInterrupts();

}

float distance_g=40;
float total_distance;

void convert(float distance,float angle)
{
	store_x(distance*angle);
	store_y(distance*angle);
}


void GPIOPortF_Handler(void)
{
	GPIO_PORTF_ICR_R|=0x10;
	left_count_a++;
}

void GPIOPortE_Handler(void)
{
	GPIO_PORTE_ICR_R|=0x10;
	right_count_a++;
	raw_angle=1.6-left_count_a;
	angle=raw_angle*0.06244;
	store_angle(angle);


	if(right_count_a>left_count_a)
		{
			raw_distance=left_count_a;
			distance=left_count_a*0.6875;
			convert(distance,angle);
		}
	else
	    {
	    	raw_distance=left_count_a;
			distance=right_count_a*1.25;
			convert(distance,angle);
		}
			right_count_a=0;
		left_count_a=0;
}

float mod(float a)
{
	if(a<0)
		return -a;
	else
		return a;
}

void run_motor_right(float error_angle)
{
	int i;
		for(i=0;i<error_angle;i++)
		{
			GPIO_PORTD_DATA_R|=0x04;
		}
	    for(i=0;i<10;i++)
	    {
			GPIO_PORTD_DATA_R&= ~0x04;
	    }
 }
void run_motor_left(float error_angle)
{
	int i;
		for(i=0;i<error_angle;i++)
		{
			GPIO_PORTD_DATA_R|=0x01;
		}
	    for(i=0;i<100;i++)
	    {
			GPIO_PORTD_DATA_R&= ~0x01;
	    }

}

void traverse_angle(float error_angle)
{
	if(error_angle<0)
	    run_motor_left(mod(error_angle));

	else if(error_angle>0)
	    run_motor_right(mod(error_angle));
}

void traverse_distance(float error_x,float error_y)
{
	float res;
	int i;
	//res=sqrt(pow(error_x,2)+pow(error_y,2));
	res=mod(x)+mod(y);
		for(i=0;i<res;i++)
		{
			GPIO_PORTD_DATA_R|=0x05;
		}
	    for(i=0;i<100;i++)
	    {
			GPIO_PORTD_DATA_R&= ~0x05;
	    }

    }

void ISR(void){
//do some math and calclations

    counter+=1;
    total_distance+=counter*(constant);
}
int main()
{
    float K_p,K_i,K_d; //these are the coefficients
    float error_prop,error_integral,error_diff; //these are the errors
    float distance_g,delta_t;
    float total_error,error_old;

    /* tuning the parameters
    K_p=1;
    K_i=1;
    K_d=0.1;
    */

    /* initializing important parameters*/
    delta_t=10;
    total_error = 0;
    error_old = 0;
    error = distance_g;
    error_distance = distance_g;

    while(error_distance!=0)
    {
        error_prop=K_p*(error);
        error_diff=K_d*((error_old-error)/delta_t);
        error_integ=K_i*(total_error*delta_t);

        total=error_prop+error_diff+error_integ;
        traverse_distance(0,total);
        error_old=error_distance;
        total_error+=error_distance;
        error_distance=distance_g-total_distance;
        error=error_distance;
    }
    return 0;
}

