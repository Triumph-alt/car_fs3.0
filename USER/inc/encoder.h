#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "headfile.h"
#include "maths.h"

#define  LEFT_DIR    P35
#define  RIGHT_DIR   P53

typedef struct
{
    int encoder_original;     //编码器一开始读出来的值
    int encoder_integral;     //编码器积分值
    int encoder_final;        //编码器滤波、消除异常之后的值
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