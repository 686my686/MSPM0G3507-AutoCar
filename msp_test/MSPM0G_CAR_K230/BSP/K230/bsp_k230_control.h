#ifndef __BSP_K230_CONTROL_H_
#define __BSP_K230_CONTROL_H_


#include "bsp.h"

extern int pid_output,pid_output1;

#define out_MAX    (70)
#define out_MIN    (-70)

void Limit(int pidout);
void Visual_Line_Track(int x,int w);
void Color_Trace(int x,int w,int h);
void QRCode_Action(const char* msg);
void Human_Face_Track(int x,int w, int h);
void AprilTag_Track(int x,int w,int h);
void Licence_Track(int x1,int y1,int x2,int y2,int x3);
void Autonomous_Avoid(int w,int h);
void RoadSign_Rec(const char* msg);
void Color_Rec(const char* msg);
void GazeDire_Track(int x_start, int x_end);
void Hand_Track(int x,int w,int h);
void Target_Track(int x);
void HumanBody_Track(int x,int w,int h);
void OCRrec_Actions(const char* msg);
void hand_Rec(const char* msg);
void Rock_Result(const char* msg);

#endif

