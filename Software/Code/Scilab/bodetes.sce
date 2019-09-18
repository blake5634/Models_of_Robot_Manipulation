

clear all; 
xdel(winsid());   // close all graphics windows which might be open


s = %s;

tf = (s+5)/((s+0.1)*(s+5000));

sys = syslin('c', tf);

bode(sys)\\,[0.01, 100000.0]);

