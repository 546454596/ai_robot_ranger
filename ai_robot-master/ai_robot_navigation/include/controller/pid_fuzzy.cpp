#include <string.h>     
#include <stdio.h> 
#include <math.h>
#include "pid_fuzzy.h"

//注1：自适应模糊pid最重要的就是论域的选择，要和你应该控制的对象相切合
//注2：以下各阀值、限幅值、输出值均需要根据具体的使用情况进行更改
//注3：因为我的控制对象惯性比较大，所以以下各部分取值较小
//论域e:[-5,5]  ec:[-0.5,0.5]

//误差的阀值，小于这个数值的时候，不做PID调整，避免误差较小时频繁调节引起震荡
#define Emin 0.0
#define Emid 0.08
#define Emax 0.6

#define NB 0
#define NM 1
#define NS 2
#define ZO 3
#define PS 4
#define PM 5
#define PB 6


int kp[7][7]={	{PB,PB,PM,PM,PS,ZO,ZO},
				{PB,PB,PM,PS,PS,ZO,ZO},
				{PM,PM,PM,PS,ZO,NS,NS},
				{PM,PM,PS,ZO,NS,NM,NM},
				{PS,PS,ZO,NS,NS,NM,NM},
				{PS,ZO,NS,NM,NM,NM,NB},
				{ZO,ZO,NM,NM,NM,NB,NB}    };

int kd[7][7]={	{PS,NS,NB,NB,NB,NM,PS},
				{PS,NS,NB,NM,NM,NS,ZO},
				{ZO,NS,NM,NM,NS,NS,ZO},
				{ZO,NS,NS,NS,NS,NS,ZO},
				{ZO,ZO,ZO,ZO,ZO,ZO,ZO},
				{PB,NS,PS,PS,PS,PS,PB},
				{PB,PM,PM,PM,PS,PS,PB}    };

int ki[7][7]={	{NB,NB,NM,NM,NS,ZO,ZO},
				{NB,NB,NM,NS,NS,ZO,ZO},
				{NB,NM,NS,NS,ZO,PS,PS},
				{NM,NM,NS,ZO,PS,PM,PM},
				{NM,NS,ZO,PS,PS,PM,PB},
				{ZO,ZO,PS,PS,PM,PB,PB},
				{ZO,ZO,PS,PM,PM,PB,PB}    };

/**************求隶属度（三角形）***************/
float FTri(float x,float a,float b,float c)//FuzzyTriangle
{
	if(x<=a)
		return 0;
	else if((a<x)&&(x<=b))
		return (x-a)/(b-a);
	else if((b<x)&&(x<=c))
		return (c-x)/(c-b);
	else if(x>c)
		return 0;
	else
		return 0;
}
/*****************求隶属度（梯形左）*******************/
float FTraL(float x,float a,float b)//FuzzyTrapezoidLeft
{
	if(x<=a)  
		return 1;
	else if((a<x)&&(x<=b))
		return (b-x)/(b-a);
	else if(x>b)
		return 0;
	else
		return 0;
}
/*****************求隶属度（梯形右）*******************/
float FTraR(float x,float a,float b)//FuzzyTrapezoidRight
{
	if(x<=a)
		return 0;
	if((a<x)&&(x<b))
		return (x-a)/(b-a);
	if(x>=b)
		return 1;
	else
		return 1;
}
/****************三角形反模糊化处理**********************/
float uFTri(float x,float a,float b,float c)
{ 
	float y,z;
	z=(b-a)*x+a;
	y=c-(c-b)*x;
	return (y+z)/2;
}
/*******************梯形（左）反模糊化***********************/
float uFTraL(float x,float a,float b)
{
	return b-(b-a)*x;
}
/*******************梯形（右）反模糊化***********************/
float uFTraR(float x,float a,float b)
{
	return (b-a)*x +a;
}
/**************************求交集****************************/
float fand(float a,float b)
{
	return (a<b)?a:b;
}
/**************************求并集****************************/
float forr(float a,float b)
{
	return (a<b)?b:a;
}

void FuzzyPID::reset_I()
{
    _integrator = 0;
    // mark derivative as invalid
    _last_derivative = NAN;
}

void FuzzyPID::load_gains(const char* filePath)
{
    std::ifstream file(filePath);
    file >> _kp >> _ki >> _kd >> _imax;
    cout <<" kP:" << _kp << " kI:" << _ki << " kD:" << _kd << " _imax:" << _imax << endl;
     _imax = fabs(_imax);

    file.close();
}

void FuzzyPID::save_gains(const char* filePath)
{
    std::ofstream file(filePath);
    file << _kp <<" "<< _ki<<" " << _kd <<" "<< _imax;
     file.close();
}

float FuzzyPID::get_p(float error) const
{
    return (float)error * _kp;
}

float FuzzyPID::get_i(float error, float dt)
{
    if((_ki != 0) && (dt != 0)) {
        if(fabs(error)<PIDController_INTEGRAL_E){
        _integrator += ((float)error * _ki) * dt;
        if (_integrator < -_imax) {
            _integrator = -_imax;
        } else if (_integrator > _imax) {
            _integrator = _imax;
        }
        return _integrator;
        }
        else
            {
            return 0;
        }

    }
    return 0;
}

float FuzzyPID::get_d(float input, float dt)
{
    if ((_kd != 0) && (dt != 0)) {
        float derivative;
        if (isnan(_last_derivative)) {
            // we've just done a reset, suppress the first derivative
            // term as we don't want a sudden change in input to cause
            // a large D output change
            derivative = 0;
            _last_derivative = 0;
        } else {
            // calculate instantaneous derivative
            derivative = (input - _last_input) / dt;
        }

        // discrete low pass filter, cuts out the
        // high frequency noise that can drive the controller crazy
        derivative = _last_derivative + _d_lpf_alpha * (derivative - _last_derivative);

        // update state
        _last_input             = input;
        _last_derivative    = derivative;


        // add in derivative component
        return _kd * derivative;
    }
    return 0;
}

float FuzzyPID::get_pid(float error, float dt)
{
    	float out_var;//out调整量
	
	//计算隶属度表
	float es[7],ecs[7],e,ec;
	int MaxX=0,MaxY=0;
	
	//记录隶属度最大项及相应推理表的p、i、d值
	float lsd;
	int temp_p,temp_d,temp_i;
	float detkp,detkd,detki;//推理后的结果
	e=error;
	ec=error-_last_input;
	//当温度差的绝对值小于Emax时，对pid的参数进行调整
	if(fabs(error)<=Emax)
	{
	//计算iError在es与ecs中各项的隶属度
	es[NB]=FTraL(e*5,-3,-1);  //e 
	es[NM]=FTri(e*5,-3,-2,0);
	es[NS]=FTri(e*5,-3,-1,1);
	es[ZO]=FTri(e*5,-2,0,2);
	es[PS]=FTri(e*5,-1,1,3);
	es[PM]=FTri(e*5,0,2,3);
	es[PB]=FTraR(e*5,1,3);

	ecs[NB]=FTraL(ec*30,-3,-1);//ec
	ecs[NM]=FTri(ec*30,-3,-2,0);
	ecs[NS]=FTri(ec*30,-3,-1,1);
	ecs[ZO]=FTri(ec*30,-2,0,2);
	ecs[PS]=FTri(ec*30,-1,1,3);
	ecs[PM]=FTri(ec*30,0,2,3);
	ecs[PB]=FTraR(ec*30,1,3);
	
	//计算隶属度表，确定e和ec相关联后表格各项隶属度的值
    for(int i=0;i<7;i++)
        for(int j=0;j<7;j++)
			form[i][j]=fand(es[i],ecs[j]);
	
	//取出具有最大隶属度的那一项
    for(int i=0;i<7;i++)
        for(int j=0;j<7;j++)
            if(form[MaxX][MaxY]<form[i][j]) {
				MaxX=i;
				MaxY=j;
			}
	//进行模糊推理，并去模糊
	lsd=form[MaxX][MaxY];
	temp_p=kp[MaxX][MaxY];
	temp_d=kd[MaxX][MaxY];   
	temp_i=ki[MaxX][MaxY];
	
	if(temp_p==NB)
		detkp=uFTraL(lsd,-0.3,-0.1);
	else if(temp_p==NM)
		detkp=uFTri(lsd,-0.3,-0.2,0);
	else if(temp_p==NS)
		detkp=uFTri(lsd,-0.3,-0.1,0.1);
	else if(temp_p==ZO)
		detkp=uFTri(lsd,-0.2,0,0.2);
	else if(temp_p==PS)
		detkp=uFTri(lsd,-0.1,0.1,0.3);
	else if(temp_p==PM)
		detkp=uFTri(lsd,0,0.2,0.3);
	else if(temp_p==PB)
		detkp=uFTraR(lsd,0.1,0.3);

	if(temp_d==NB)
		detkd=uFTraL(lsd,-3,-1);
	else if(temp_d==NM)
		detkd=uFTri(lsd,-3,-2,0);
	else if(temp_d==NS)
		detkd=uFTri(lsd,-3,1,1);
	else if(temp_d==ZO)
		detkd=uFTri(lsd,-2,0,2);
	else if(temp_d==PS)
		detkd=uFTri(lsd,-1,1,3);
	else if(temp_d==PM)
		detkd=uFTri(lsd,0,2,3);
	else if(temp_d==PB)
		detkd=uFTraR(lsd,1,3);

	if(temp_i==NB)
		detki=uFTraL(lsd,-0.06,-0.02);
	else if(temp_i==NM)
		detki=uFTri(lsd,-0.06,-0.04,0);
	else if(temp_i==NS)
		detki=uFTri(lsd,-0.06,-0.02,0.02);
	else if(temp_i==ZO)
		detki=uFTri(lsd,-0.04,0,0.04);
	else if(temp_i==PS)
		detki=uFTri(lsd,-0.02,0.02,0.06);
	else if(temp_i==PM)
		detki=uFTri(lsd,0,0.04,0.06);
	else if (temp_i==PB)
		detki=uFTraR(lsd,0.02,0.06);

	//pid三项系数的修改
	_kp+=detkp;
    if(fabs(_ki)<1e-9)_ki=0;
	else _ki+=detki;

    if(fabs(_kd)<1e-9)_kd=0;
	else _kd+=detkd;
	//_kd=0;//取消微分作用
	
	//对Kp,Ki进行限幅
	if(_kp<0)_kp=0;
	if(_ki<0)_ki=0;
	}
	
    return get_p(error) + get_i(error, dt) + get_d(error, dt);
}

