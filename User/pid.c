#include "pid.h"
#include <math.h>
#include <main.h>

PID_typedef Motor_pid[14];

/**
  * @brief          ���ֵ����
  * @retval         none
  */
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
  * @brief          pid�����ʼ��
  * @retval         none
  */		
void PID_total_init(void)
{
	for(uint8_t i=0;i<3;i++){
		PID_Init(&Motor_pid[i],PID_POSITION_SPEED ,0.08,0.01,0.0,1000,1000,0);
	}
	PID_Init(&Motor_pid[7],PID_POSITION_SPEED,2,0.1,0,8000,16384,0);
	PID_Init(&Motor_pid[12],PID_POSITION_ANGLE,10,0.1,0,8000,30000,0);
}

/**
  * @brief          pid struct data init���ṹ���ʼ�����������ز���ͨ��������������pid�ṹ��
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_POSITION_SPEED: λ��ʽPID���ٶ�
  *                 			PID_POSITION_ANGLE: λ��ʽPID���Ƕ�
	*  											PID_DELTA_SPEED		�����ʽPID���ٶ�
  * @param[in]      kp:PID����p
	* @param[in] 			ki:PID����i
	* @param[in] 			kd:PID����d
  * @param[in]      Max_iout:pid���������
  * @param[in]      Max_out:pid������
	* @param[in]			deadband:PID����
  * @retval         none
  */
void PID_Init(PID_typedef *PID,PID_mode Mode,float kp,float ki,float kd,float Max_iout,float Max_out,float deadband)
{
	if(PID == NULL)	return;
	
	PID->mode = Mode;																														//�Ѻ������������Ӧ�Ľṹ����
	PID->Kp = kp;
	PID->Ki = ki;
	PID->Kd = kd;
	PID->Max_iout = Max_iout;
	PID->Max_out = Max_out;
	PID->DeadBand = deadband;
}

/**
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      measure:������������
	* @param[in] 			target: Ŀ��ֵ
  * @retval         none
  */
float PID_calc(PID_typedef *PID, float measure, float target)
{
	if(PID == NULL)
		return 0;
	
	PID->measure += measure;																											//�������ݣ���ͬ
	PID->target = target;
	PID->error[2] = PID->error[1];																							//���ֵ���ݣ�1Ϊ��һ�Σ�2Ϊ���ϴ�
	PID->error[1] = PID->error[0];																							//���ֵ���ݣ�0Ϊ������1Ϊ��һ��
	PID->error[0] =target - measure;																						//���ֵ���㣬Ŀ��ֵ-����ֵ
	
	if(fabsf(PID->error[0]) > PID->DeadBand || PID->DeadBand==0){								//�ж���������������->����

		if(PID->mode == PID_POSITION_SPEED || PID->mode == PID_POSITION_ANGLE){		//λ��ʽPID����
			
			if(PID->mode == PID_POSITION_ANGLE){																		//λ��ʽPID-�ǶȻ�
				if(PID->error[0]>4096)	PID->error[0]=PID->error[0]-8191;							//�ǶȻ�������������-4096 ~~ +4096
				else if(PID->error[0]<-4096)	PID->error[0]=PID->error[0]+8191;
			}
			PID->Pout = PID->Kp * PID->error[0];																		//p���
			PID->Iout += PID->Ki * PID->error[0];
																		//i������ۼ�																			//ǰ������ֵ���ݣ�0Ϊ���²�ֵ
			PID->D_item = (PID->error[0] - PID->error[1]);													//΢�������
			PID->Dout = PID->Kd * PID->D_item;																			//d�����ͨ��΢����
			LimitMax(PID->Iout,PID->Max_iout);																			//i�������
			PID->OUT = PID->Pout + PID->Iout + PID->Dout;														//�����
			LimitMax(PID->OUT,PID->Max_out);																				//�������
		}
		else if(PID->mode == PID_DELTA_SPEED){																		//����ʽPID-�ٶȻ�
			PID->Pout = PID->Kp * (PID->error[0] - PID->error[1]);									//p���
			PID->Iout = PID->Ki * PID->error[0];																		//i���
			PID->D_item = (PID->error[0] - 2.0f*PID->error[1] + PID->error[2]);			//΢�������
			PID->Dout = PID->Kd * PID->D_item;																			//d�����ͨ��΢����
			PID->OUT += PID->Pout + PID->Iout + PID->Dout;													//�����
			LimitMax(PID->OUT, PID->Max_out);																				//�������
		}
	}
	else{
        PID->OUT=0;																														//����������ڣ����Ϊ0
	}
	
	return PID->OUT;																														//�������ֵ
}














