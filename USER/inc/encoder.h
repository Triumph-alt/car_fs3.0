#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "headfile.h"
#include "maths.h"

#define  LEFT_DIR    P35
#define  RIGHT_DIR   P53

typedef struct
{
    int encoder_original;     //������һ��ʼ��������ֵ
    int encoder_integral;     //����������ֵ
    int encoder_final;        //�������˲��������쳣֮���ֵ
} Encoder_t;

typedef struct
{
	int count;
	int encoderlast;
} EncoderDebo_t;

extern Encoder_t EncoderL, EncoderR;
extern EncoderDebo_t EncoderDeboL, EncoderDeboR;

void encoder_init(void);
int get_left_encoder(void);
int get_right_encoder(void);
int encoder_debounce(EncoderDebo_t* instance, int encoder);

#endif