#include "main.h"
#include "myDIP.h"

uint8_t Read_Point_b(Picture Pic,uint8_t x,uint8_t y){
	uint8_t value;
	x=x%Pic.length;
	y=y%Pic.width;
	if(Pic.pPic[2*Pic.length*(y-1)+2*(x-1)]==0) value=0;
	else value=1;
	return value;
}

void Draw_Point(Picture Pic,uint16_t x, uint16_t y, uint16_t Color){
	uint8_t Color_L = *(uint8*)&Color;
	uint8_t Color_H = *((uint8*)&Color+1);
	if(0<x && x<Pic.length && 0<y && y<Pic.width){
		Pic.pPic[2*Pic.length*(y-1)+2*(x-1)]=Color_H;
		Pic.pPic[2*Pic.length*(y-1)+2*(x-1)+1]=Color_L;
	}
}

void Draw_Circle(Picture Pic,uint16_t x,uint16_t y,uint16_t R, uint16_t Color){
	for(int b=0;b<R/sqrt(2)+1;b++){
		double a=sqrt(R*R-b*b);
		if(a-(int)a>0.5) a=(int)a+1;
		else(a=(int)a);
		Draw_Point(Pic,x-a,y+b,Color);
		Draw_Point(Pic,x-a,y-b,Color);
		Draw_Point(Pic,x+a,y+b,Color);
		Draw_Point(Pic,x+a,y-b,Color);
		Draw_Point(Pic,x-b,y+a,Color);
		Draw_Point(Pic,x-b,y-a,Color);
		Draw_Point(Pic,x+b,y+a,Color);
		Draw_Point(Pic,x+b,y-a,Color);
	}
}
void Draw_Cross(Picture Pic,uint16_t x, uint16_t y, uint16_t Color){
	int Cross_length=10;
	for(int i=0;i<2*Cross_length+1;i++){
			Draw_Point(Pic,x-Cross_length+i,y,Color);
	}
	for(int i=0;i<2*Cross_length+1;i++){
			Draw_Point(Pic,x,y-Cross_length+i,Color);
	}
}

void Draw_Rectangle(Picture Pic,uint16_t x,uint16_t y,uint16_t Rectangle_length,uint16_t Rectangle_width,uint16_t Line_Thickness, uint16_t Color){		//只有RGB565图像可以用//x:横着数第x个像素；y:竖着数第y个像素
	uint8_t Color_L = *(uint8*)&Color;
	uint8_t Color_H = *((uint8*)&Color+1);
	for(; Line_Thickness>0; Line_Thickness--){
	
		for(uint16_t i=0; i<Rectangle_length*2;i++){
			Pic.pPic[2*Pic.length*(y-1) + 2*(x-1)+i] = Color_H;
			Pic.pPic[2*Pic.length*(y-1+Rectangle_width-1) + 2*(x-1)+i] = Color_H;
			i++;
			Pic.pPic[2*Pic.length*(y-1) + 2*(x-1)+i] = Color_L;
			Pic.pPic[2*Pic.length*(y-1+Rectangle_width-1) + 2*(x-1)+i] = Color_L;
		}//画上下边

		for(uint16_t i=0; i<Rectangle_width-2; i++){
			Pic.pPic[2*(x-1) + 2*Pic.length*(y+i)] = Color_H;
			Pic.pPic[2*(x-1)+1 + 2*Pic.length*(y+i)] = Color_L;
			
			Pic.pPic[2*(x+Rectangle_length-1-1) + 2*Pic.length*(y+i)] = Color_H;
			Pic.pPic[2*(x+Rectangle_length-1-1)+1 + 2*Pic.length*(y+i)] = Color_L;
		}//画左右边
		
		if(x-1>0) x-=1;
		if(y-1>0) y-=1;
		Rectangle_width+=2;
		Rectangle_length+=2;
		
	}
}

void Draw_Line_SL(Picture Pic,double Line_slope, double Line_intercept, uint16_t Line_Thickness, uint16_t Color){
	if(Line_slope >= 1 || Line_slope < -1){
		double x;
		for(int y=1;y<=Pic.width;y++){
			x=(y-Line_intercept)/Line_slope;
			
			if(x>0){
				if((x-(int)x) < 0.5) {
					for(int i=0;i<Line_Thickness;i++)  Draw_Point(Pic,(int)x-Line_Thickness/2+i,y,Color);
				}else{
					for(int i=0;i<Line_Thickness;i++)  Draw_Point(Pic,(int)x-Line_Thickness/2+1+i,y,Color);
				}
			}
		}
	}
	
	if(Line_slope < 1 && Line_slope >= -1){
		double y;
		for(int x=1;x<=Pic.length;x++){
			y=Line_slope*x+Line_intercept;
			
			if(y>0){
				if((y-(int)y) < 0.5) {
					for(int i=0;i<Line_Thickness;i++)  Draw_Point(Pic,x,(int)y-Line_Thickness/2+i,Color);
				}else{	
					for(int i=0;i<Line_Thickness;i++)  Draw_Point(Pic,x,(int)y-Line_Thickness/2+1+i,Color);
				}
			}
		}
	}
	
}

void Draw_Line_RTh(Picture Pic,double R, double Theta, uint16_t Line_Thickness,uint16_t Color){
	
	if(Theta>=PI/4 && Theta<=PI*3/4){
		double y;
		for(int x=1;x<Pic.length;x++){
			y=(R-x*cos(Theta))/sin(Theta);
			if((y-(int)y) < 0.5){
				for(int i=0;i<Line_Thickness;i++) Draw_Point(Pic,x,(int)y-Line_Thickness/2+i,Color);
			}else{
				for(int i=0;i<Line_Thickness;i++) Draw_Point(Pic,x,(int)y-Line_Thickness/2+1+i,Color);
			}
		}
	}
	
	if(Theta<PI/4 || Theta>=PI*3/4){
		double x;
		for(int y=1;y<Pic.width;y++){
			x=(R-y*sin(Theta))/cos(Theta);
			if((x-(int)x) < 0.5){
				for(int i=0;i<Line_Thickness;i++) Draw_Point(Pic,(int)x-Line_Thickness/2+i,y,Color);
			}else{
				for(int i=0;i<Line_Thickness;i++) Draw_Point(Pic,(int)x-Line_Thickness/2+1+i,y,Color);
			}
		}
	}	
	
}

/***************************************************************算法函数*********************************************************************************************/
//图像二值化
void Img_Binarize(Picture Pic){
	uint32_t Gray_Sum=0;
	uint8_t temp=0;
	uint32_t Gray_Avr;
	
	for(int i=0; i<Pic.length*Pic.width*2; i++){
		uint8_t recR = Pic.pPic[i] & 0xf8;
		uint8_t recG = (((Pic.pPic[i]<<8) + Pic.pPic[i+1]) >> 3) & 0xfc;
		uint8_t recB = Pic.pPic[i+1] & 0xf8;
		Pic.pPic[i] = (recR*76 + recG*150 + recB*30) >> 8;
		Gray_Sum+=Pic.pPic[i];
		i++;
	}//灰度
	Gray_Avr = Gray_Sum / (Pic.length*Pic.width);	
	for(int i=0; i<Pic.length*Pic.width*2; i++){
		temp=Pic.pPic[i];
		Pic.pPic[i]=0;
		if(temp<Gray_Avr-25){ 
			Pic.pPic[i]=0;
			Pic.pPic[i+1]=0;
		}else{
			Pic.pPic[i]=255;
			Pic.pPic[i+1]=255;	
		}
		i++;
	}//二值化
}
/*
void Img_Binarize_bit(Picture Pic){//低位在前，D0 D1 D2 D3......
	uint32_t Gray_Avr;
	uint8_t temp=0;
	uint32_t Gray_Sum=0;
	for(int i=0; i<Pic.length*Pic.width*2; i++){
		uint8_t recR = Pic.pPic[i] & 0xf8;
		uint8_t recG = (((Pic.pPic[i]<<8) + Pic.pPic[i+1]) >> 3) & 0xfc;
		uint8_t recB = Pic.pPic[i+1] & 0xf8;
		Pic.pPic[i] = (recR*76 + recG*150 + recB*30) >> 8;
		Gray_Sum+=Pic.pPic[i];
		i++;
	}//灰度
	Gray_Avr = Gray_Sum / (Pic.length*Pic.width);	
	for(int i=0; i<Pic.length*Pic.width*2; i++){
		if(Pic.pPic[i]<Gray_Avr-25){ 
			Pic.pPic[i/2/8]=Pic.pPic[i/2/8] &~(1<<i/2%8);
		}else{
			Pic.pPic[i/2/8]=Pic.pPic[i/2/8] | 1<<i/2%8;
		}
		i++;
	}//二值化

}
*/

void Img_Binarize_bit(Picture* pPic,uint8_t* pBuffer) {
    uint32_t Gray_Avr;
    uint32_t Gray_Sum=0;
    for(int i=0; i<pPic->length*pPic->width*2; i++){
        uint8_t recR = pPic->pPic[i] & 0xf8;
        uint8_t recG = (((pPic->pPic[i]<<8) + pPic->pPic[i+1]) >> 3) & 0xfc;
        uint8_t recB = pPic->pPic[i+1] & 0xf8;
        pPic->pPic[i] = (recR*76 + recG*150 + recB*30) >> 8;
        Gray_Sum+=pPic->pPic[i];
        i++;
    } // 灰度
    Gray_Avr = Gray_Sum / (pPic->length * pPic->width);	
    for(int i=0; i<pPic->length*pPic->width*2; i++){
        if(pPic->pPic[i] < Gray_Avr) { 
          pBuffer[i/2/8] = pBuffer[i/2/8] & ~(1<<i/2%8);
        } else {
					pBuffer[i/2/8] = pBuffer[i/2/8] | 1<<i/2%8;
        }
        i++;
    } // 二值化
}

//最小二乘法
void Least_square(double StoreIn_SL[],uint8_t xyArray[][2],uint32_t xyNum){
	uint8_t x_Sum;
	uint8_t y_Sum;
	for(int i=0;i<xyNum;i++){
		x_Sum+=xyArray[i][0];
		y_Sum+=xyArray[i][1];
	}
	uint8_t x_Avr=x_Sum/xyNum;
	uint8_t y_Avr=y_Sum/xyNum;
	uint8_t Var=0;
	uint8_t Cov=0;
	
	for(int i=0; i<xyNum; i++){
		Var+=(xyArray[xyNum][0]-x_Avr)* (xyArray[xyNum][0]-x_Avr);
		Cov+=(xyArray[xyNum][0]-x_Avr)* (xyArray[xyNum][1]-y_Avr);
	}//最小二乘法
	StoreIn_SL[0]=(double)Cov/(double)Var;//slope
	StoreIn_SL[1]=y_Avr-StoreIn_SL[0]*x_Avr;//intercept
}

//画圆寻找二值图边缘点
void Find_Edge(uint8_t StoreIn_XY[][2],Picture BaryPic,int R){
	uint8_t i=1;
	uint8_t j=0;
	uint8_t EdgeNum=0;
	while(j<BaryPic.width){
		if(Read_Point_b(BaryPic,i,BaryPic.width-j)==Read_Point_b(BaryPic,i+1,BaryPic.width-j)) i++;
		else {
			StoreIn_XY[EdgeNum][0]=i;
			StoreIn_XY[EdgeNum][1]=BaryPic.width-j;
			EdgeNum++;
			i=0;
			j+=R;
		}
	}
	
}

//此为只找两条线。如果需要所有线，非极大值抑制遍历每个3*3领域找保留最大值，然后再阈值化即可
void HoughTransform_2Line(double StoreIn_RTh[][2],Picture Pic,uint8_t Edge_xyArray[][2],uint32_t EdgeNum,uint8_t ThetaDim,double DistStep){

	uint16_t Long_side;
	if(Pic.length > Pic.width) Long_side=Pic.length; else Long_side=Pic.width;
	uint16_t Max_Dist=Long_side+sqrt(Pic.length*Pic.length+Pic.width*Pic.width);;
	uint16_t DistDim=(int)(Max_Dist/DistStep);
	
	//uint8_t accumulator[ThetaDim][DistDim];
	static uint8_t accumulator[90][(361+255)/2]={0};	
	for(int i=0;i<45*(361+255)/4;i++) *(&accumulator[0][0]+i)=0;
//	static volatile uint16_t accltor_MAX=0;
//	static volatile uint16_t accltor_MAX_rNum[2];
//	static volatile uint16_t accltor_MAX_thNum[2];
//	static  double accltor_MAX_rth[4]={0};//r1 th1 r2 th2
	
//	static double SinTheta[90];
//	static double CosTheta[90];
	
	uint16_t accltor_MAX=0;
  uint16_t accltor_MAX_rNum[2];
  uint16_t accltor_MAX_thNum[2];
	double SinTheta[90];
	double CosTheta[90];
	
	for(int i=0;i<ThetaDim;i++){
		SinTheta[i]=sin(i*PI/ThetaDim);
		CosTheta[i]=cos(i*PI/ThetaDim);
	}
	
	double xcos;
	double ysin;
  int Dis;
	for(int Num=0;Num<EdgeNum;Num++){
		for(int i=0;i<ThetaDim;i++){
				xcos=Edge_xyArray[Num][0]*CosTheta[i];
				ysin=Edge_xyArray[Num][1]*SinTheta[i];
			if((xcos+ysin)-(int)(xcos+ysin)>=0.5){
				Dis=(xcos+ysin+1+Long_side)/DistStep;
			}else{
				Dis=(xcos+ysin+Long_side)/DistStep;
			}
				accumulator[i][Dis]+=1;
		}
	}//投票
	
	accltor_MAX=accumulator[0][0];
	for(int th=0;th<ThetaDim;th++){
		for(int r=0;r<DistDim;r++){
			if(accltor_MAX<accumulator[th][r]){
				accltor_MAX=accumulator[th][r];
				accltor_MAX_rNum[0]=r;
				accltor_MAX_thNum[0]=th;
			}
		}
	}//极值1
	
	for(int rr=0;rr<DistDim;rr++){
		for(int thh=0;thh<ThetaDim;thh++){
			if(accumulator[thh][rr]<accltor_MAX/4) accumulator[thh][rr]=0;
		}
	}//阈值化
		
	int tempth=0;
	int tempr=0;
	int th_halfwindow=6;
	int r_halfwindow=10;
	for(int i=-th_halfwindow; i<=th_halfwindow; i++){//th抑制长度
		for(int j=-r_halfwindow; j<=r_halfwindow; j++){//r抑制长度
			if(accltor_MAX_thNum[0]+i<0)  tempth=0; 
			else if(accltor_MAX_thNum[0]+i>ThetaDim-1) tempth=ThetaDim-1;
			else tempth=accltor_MAX_thNum[0]+i;
			
			if(accltor_MAX_rNum[0]+j<0)  tempr=0; 
			else if(accltor_MAX_rNum[0]+j>DistDim-1) tempr=DistDim-1;
			else tempr=accltor_MAX_rNum[0]+j;			
			accumulator[tempth][tempr]=0;
		}
	}//抑制
	
	accltor_MAX=accumulator[0][0];
	for(int r=0;r<DistDim;r++){
		for(int th=0;th<ThetaDim;th++){
			if(accltor_MAX<accumulator[th][r]) {
				accltor_MAX=accumulator[th][r];
				accltor_MAX_rNum[1]=r;
				accltor_MAX_thNum[1]=th;
			}
		}
	}//极值2
	
	StoreIn_RTh[0][0]=(accltor_MAX_rNum[0])*DistStep-Long_side;//r1
	StoreIn_RTh[0][1]=accltor_MAX_thNum[0]*PI/ThetaDim;//th1
	StoreIn_RTh[1][0]=(accltor_MAX_rNum[1])*DistStep-Long_side;//r2
	StoreIn_RTh[1][1]=accltor_MAX_thNum[1]*PI/ThetaDim;//th2
}
