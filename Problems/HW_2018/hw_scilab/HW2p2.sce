//  EE543 HW2.2   W18

clear;
exec('kinfunc.sce',-1);


T5 = roty4(30)*trans4([1,0,0],10)
T4 = rotz4(-90)
T3 = trans4([1,0,0],20)
T2 = rotz4(20)
T1 = trans4([1,0,0],-10)

Tab = T1*T2*T3*T4*T5

disp(Tab)
