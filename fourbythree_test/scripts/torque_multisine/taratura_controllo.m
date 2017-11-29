clear all;close all;clc

if 0
  syms k s Kp Ki
  
  sys=k/s;
  C=Kp+Ki/s;
  
  F=simplify((sys*C)/(1+sys*C))
else
  s=tf('s');
  wn=2*pi*5;
  xci=0.9;
  k=1e4;
  Ki_vel=wn^2/k;
  Kp_vel=2*xci*wn/k;
  
  P=k/s;
  C=Kp_vel+Ki_vel/s;
  
  Fvel=feedback(P*C,1);
  step(Fvel)
  figure
  margin(P*C)
  
  Kp_pos=5;
  
  Fpos=feedback(Fvel*Kp_pos,1);
  figure
  step(Fpos)
  figure
  margin(Fvel*Kp_pos)
  
end