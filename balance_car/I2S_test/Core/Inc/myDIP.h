#ifndef __MY_DIP_H__
#define __MY_DIP_H__

#include "main.h"

typedef struct Picture{
	uint8_t length;
	uint8_t width;
	uint8_t *pPic;

}Picture;

uint8_t Read_Point_b(Picture Pic,uint8_t x,uint8_t y);
void Draw_Point(Picture Pic,uint16_t x, uint16_t y, uint16_t Color);
void Draw_Cross(Picture Pic,uint16_t x, uint16_t y, uint16_t Color);
void Draw_Circle(Picture Pic,uint16_t x,uint16_t y,uint16_t R, uint16_t Color);
void Draw_Rectangle(Picture Pic,uint16_t x,uint16_t y,uint16_t Rectangle_length,uint16_t Rectangle_width,uint16_t Rectangle_LineThickness,uint16_t Color);
void Draw_Line_SL(Picture Pic,double Line_slope, double Line_intercept, uint16_t Line_Thickness,uint16_t Color);
void Draw_Line_RTh(Picture Pic,double R, double Theta, uint16_t Line_Thickness,uint16_t Color);

void Img_Binarize(Picture Pic);
void Img_Binarize_bit(Picture* pPic,uint8_t* pBuffer);
void Least_square(double StoreIn[],uint8_t xy_Array[][2],uint32_t Num);
void Find_Edge(uint8_t StoreIn[][2],Picture Pic,int R);
void HoughTransform_2Line(double StoreIn[][2],Picture Pic,uint8_t Edge_xyArray[][2],uint32_t EdgeNum,uint8_t ThetaDim,double DistStep);
#endif
