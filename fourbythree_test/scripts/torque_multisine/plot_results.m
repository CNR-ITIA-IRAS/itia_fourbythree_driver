clear all;clc;close all;

load multisine_jnt2_P2

iJoint=2;

%cDq{iJoint}=1i*2*pi*cfreq{iJoint}.*cq{iJoint};
%cDtheta{iJoint}=(1i*2*pi*cfreq{iJoint}).*ctheta{iJoint};
%cDdelta{iJoint}=(1i*2*pi*cfreq{iJoint}).*cdelta{iJoint};


figure(1)
hfig(1)=subplot(211);
semilogx(cfreq{iJoint}, 20*log10(abs(cDq{iJoint}./cdelta{iJoint})),'o','Color','b')
ylabel('Magnitude $-\frac{\dot{q}}{\delta}$','interpreter','latex','FontSize',18);
xlabel('Frequency [Hz]','interpreter','latex','FontSize',18);
grid on
hold on

hfig(2)=subplot(212);
semilogx(cfreq{iJoint}, 180/pi*unwrap(angle(-cDq{iJoint}./cdelta{iJoint})),'o','Color','b')
ylabel('Phase $-\frac{\dot{q}}{\delta}$','interpreter','latex','FontSize',18);
xlabel('Frequency [Hz]','interpreter','latex','FontSize',18);
grid on
hold on
figure(2)
hfig(3)=subplot(211);
semilogx(cfreq{iJoint}, 20*log10(abs(cDq{iJoint}./ctau{iJoint})),'o','Color','b')
ylabel('Magnitude $\frac{\dot{q}}{\tau}$','interpreter','latex','FontSize',18);
xlabel('Frequency [Hz]','interpreter','latex','FontSize',18);
hold on

grid on

hfig(4)=subplot(212);
semilogx(cfreq{iJoint}, 180/pi*unwrap(angle(cDtheta{iJoint}./ctau{iJoint})),'o','Color','b')
grid on
ylabel('Phase $-\frac{\dot{q}}{\tau}$','interpreter','latex','FontSize',18);
xlabel('Frequency [Hz]','interpreter','latex','FontSize',18);
hold on
% 
% figure(3)
% hfig(5)=subplot(211);
% semilogx(cfreq{iJoint}, 20*log10(abs(cdelta{iJoint}./ctau{iJoint})),'o','Color','b')
% ylabel('Magnitude $\frac{{\delta}}{\tau}$','interpreter','latex','FontSize',18);
% xlabel('Frequency [Hz]','interpreter','latex','FontSize',18);
% grid on
% 
% hfig(6)=subplot(212);
% semilogx(cfreq{iJoint}, 180/pi*unwrap(angle(cdelta{iJoint}./ctau{iJoint})),'o','Color','b')
% ylabel('Phase $\frac{{\delta}}{\tau}$','interpreter','latex','FontSize',18);
% xlabel('Frequency [Hz]','interpreter','latex','FontSize',18);
% grid on


figure(4)
hfig(7)=subplot(211);
semilogx(cfreq{iJoint}, 20*log10(abs(cDtheta{iJoint}./ctau{iJoint})),'o','Color','b')
ylabel('Magnitude $\frac{\dot{\theta}}{\tau}$','interpreter','latex','FontSize',18);
xlabel('Frequency [Hz]','interpreter','latex','FontSize',18);
grid on
hold on

hfig(8)=subplot(212);
semilogx(cfreq{iJoint}, 180/pi*unwrap(angle(cDtheta{iJoint}./ctau{iJoint})),'o','Color','b')
ylabel('Phase $\frac{\dot{\theta}}{\tau}$','interpreter','latex','FontSize',18);
xlabel('Frequency [Hz]','interpreter','latex','FontSize',18);
grid on


linkaxes(hfig,'x')

%%
% k=100;
% wn=15.6*2*pi;
% slope=1e4;
% K=10^(-4/20); 
% zero=-200*2*pi;

k=920;
wn=16.6*2*pi;
%slope=1e4;
slope=1e4;
K=10^(-10/20); 
zero=-50*2*pi;
fl=1.2;

%K=10^(-5/20); 
%zero=-50*2*pi;

s=tf('s');

% fm=(zero - K*k)/(K*zero) - (2*k*zero - 2*K*k^2 + K*zero*((slope*(4*k + K*slope))/K)^(1/2) + K*slope*zero)/(2*K*k*zero)
% Jl=(k*(- K*k^2 + zero*k + K*slope*zero))/(K*slope*wn^2*zero) - (k*(2*k*zero - 2*K*k^2 + K*zero*((slope*(4*k + K*slope))/K)^(1/2) + K*slope*zero))/(2*K*slope*wn^2*zero)
% Jm=(- K*k^2 + 2*zero*k + K*slope*zero)/(wn^2*zero) - (2*k*zero - 2*K*k^2 + K*zero*((slope*(4*k + K*slope))/K)^(1/2) + K*slope*zero)/(2*wn^2*zero)
% h=(2*k*zero - 2*K*k^2 + K*zero*((slope*(4*k + K*slope))/K)^(1/2) + K*slope*zero)/(2*K*k*zero)
% 
% tf_Dq_delta=-(Jm*k - Jl*fm*s + Jm*h*s)/(Jl*(fm + Jm*s));
% %tf_Dtheta_tau=(Jm*(Jl*s^2 + h*s + k))/((fm + Jm*s)*(Jl*k + Jm*k + Jl*h*s + Jm*h*s + Jl*Jm*s^2));
% tf_Dq_tau=(Jm*k - Jl*fm*s + Jm*h*s)/((fm + Jm*s)*(Jl*k + Jm*k + Jl*h*s + Jm*h*s + Jl*Jm*s^2));
% 
% frDq_tau=freqresp(tf_Dq_tau,2*pi*cfreq{iJoint});
% frDq_tau=frDq_tau(:);
% 
% frDq_delta=freqresp(tf_Dq_delta,2*pi*cfreq{iJoint});
% frDq_delta=frDq_delta(:);
% 
% 
% semilogx(hfig(1),cfreq{iJoint}, 20*log10(abs(frDq_delta)),'-','Color','m','LineWidth',2)
% semilogx(hfig(3),cfreq{iJoint}, 20*log10(abs(frDq_tau)),'-','Color','m','LineWidth',2)

% fm=(zero - K*k)/(K*zero) - (2*k*zero - 2*K*k^2 - K*zero*((slope*(4*k + K*slope))/K)^(1/2) + K*slope*zero)/(2*K*k*zero)
% Jl=(k*(- K*k^2 + zero*k + K*slope*zero))/(K*slope*wn^2*zero) - (k*(2*k*zero - 2*K*k^2 - K*zero*((slope*(4*k + K*slope))/K)^(1/2) + K*slope*zero))/(2*K*slope*wn^2*zero)
% Jm=(- K*k^2 + 2*zero*k + K*slope*zero)/(wn^2*zero) - (2*k*zero - 2*K*k^2 - K*zero*((slope*(4*k + K*slope))/K)^(1/2) + K*slope*zero)/(2*wn^2*zero)
% h=(2*k*zero - 2*K*k^2 - K*zero*((slope*(4*k + K*slope))/K)^(1/2) + K*slope*zero)/(2*K*k*zero)

fm=-(K*fl - 1)/K
Jl=(k*(k + K*slope - K*fl*k))/(K*slope*wn^2)
Jm=(k + K*slope - K*fl*k)/(wn^2 - K*fl*wn^2)
h=(k*zero*K^2*fl^2 - k*slope*K^2 - 2*k*zero*K*fl + k*zero)/(K^2*slope*zero)

%%
A=[
  [ 0,                             0,                       1,                       0]
[ 0,                             0,                       0,                       1]
[ 0,                          k/Jm,                  -fm/Jm,                    h/Jm]
[ 0, -(Jm*h + Jl*k + Jm*k)/(Jl*Jm), (Jl*fm - Jm*fl)/(Jl*Jm), -(Jm*fl + Jl*h)/(Jl*Jm)]
];
B=[0;0;1/Jm;-1/Jm];
C=[eye(4);[1 1 0 0];[0 0 1 1];[0 0 0 0]];
D=[zeros(6,1);1];
sys=ss(A,B,C,D);
sys.StateName={'theta','delta','Dtheta','Ddelta'}; 
sys.InputName={'tau'};
sys.OutputName={'theta','delta','Dtheta','Ddelta','q','Dq','tau'};


tf_Dq_delta=-(Jm*k - Jl*fm*s + Jm*h*s)/(Jl*(fm + Jm*s));
tf_Dtheta_tau=(Jm*(Jl*s^2 + h*s + k))/((fm + Jm*s)*(Jl*k + Jm*k + Jl*h*s + Jm*h*s + Jl*Jm*s^2));
tf_Dq_tau=(Jm*k - Jl*fm*s + Jm*h*s)/((fm + Jm*s)*(Jl*k + Jm*k + Jl*h*s + Jm*h*s + Jl*Jm*s^2));

%frDq_tau=freqresp(tf_Dq_tau,2*pi*cfreq{iJoint});
frDq_tau=freqresp(sys(6,1),2*pi*cfreq{iJoint});
frDq_tau=frDq_tau(:);

frDq_delta=freqresp(tf_Dq_delta,2*pi*cfreq{iJoint});
frDq_delta=frDq_delta(:);

%frDtheta_tau=freqresp(tf_Dtheta_tau,2*pi*cfreq{iJoint});
frDtheta_tau=freqresp(sys(3,1),2*pi*cfreq{iJoint});
frDtheta_tau=frDtheta_tau(:);

semilogx(hfig(1),cfreq{iJoint}, 20*log10(abs(frDq_delta)),'-','Color','r','LineWidth',2)
semilogx(hfig(3),cfreq{iJoint}, 20*log10(abs(frDq_tau)),'-','Color','r','LineWidth',2)
semilogx(hfig(7),cfreq{iJoint}, 20*log10(abs(frDtheta_tau)),'-','Color','r','LineWidth',2)

