clear all;close all;clc;

load test_torque_multisine
pathname='/home/maxwell/test/fourbythree/torque_multisine/two_joints/';
iJoint=1;
testname=sprintf('test_joint%d',iJoint);
elastic_js=bin_convert([pathname,testname,'_JointState__elastic_joint_states.bin'],4*3+1);
ffw_js=bin_convert([pathname,testname,'_JointState__joint_feedforward.bin'],2*3+1);
sp_js=bin_convert([pathname,testname,'_JointState__sp_joint_states.bin'],2*3+1);

data=bin_resampling({elastic_js,ffw_js,sp_js},1e-3);

t=data{1}(:,1)-data{1}(1,1);
idxs=find((t>10).*(t<95));
theta=data{1}(idxs,1+[1 3]);
Dtheta=data{1}(idxs,5+[1 3]);
delta=data{1}(idxs,1+[2 4]);
Ddelta=data{1}(idxs,5+[2 4]);
q=theta+delta;
Dq=Dtheta+Ddelta;

tau=data{1}(idxs,9+[1 3]);
ffw_torque=data{2}(idxs,1+4+[1:2]);
t=t(idxs)-t(idxs(1));

%%
%plot(t,ffw_torque(:,1),t,tau(:,1))

%freq=(1:200)'*0.1;
freq=sort(nat_freq)/2/pi;
c_ffw=fourier_coeff(t,freq,ffw_torque(:,iJoint));
c_tau=fourier_coeff(t,freq,tau(:,iJoint));
c_Dtheta=fourier_coeff(t,freq,Dtheta(:,iJoint));
c_Dq=fourier_coeff(t,freq,Dq(:,iJoint));
c_Ddelta=fourier_coeff(t,freq,Ddelta(:,iJoint));
c_q=fourier_coeff(t,freq,Dq(:,iJoint));
c_delta=fourier_coeff(t,freq,Ddelta(:,iJoint));

ffw=fourier_series(t,freq,c_ffw);
tau_fourier=fourier_series(t,freq,c_tau);
Dtheta_fourier=fourier_series(t,freq,c_Dtheta);
%
figure(1)
subplot(211)
plot(t,tau(:,1)-tau_fourier);
subplot(212)
plot(t,ffw_torque(:,1)-ffw);

%%
figure(2)
semilogx(freq, 20*log10(abs(c_Dtheta./c_tau)),'-')
hold all
semilogx(freq, 20*log10(abs(c_Ddelta./c_tau)),'-')
semilogx(freq, 20*log10(abs(c_Dq./c_tau)),'-')
semilogx(freq, 20*log10(abs(c_Dq./c_Dtheta)),'-')
semilogx(freq, 20*log10(abs(c_Dq./c_Ddelta)),'-')
semilogx(freq, 20*log10(abs(c_Dq./c_delta)),'-')
semilogx(freq, 20*log10(abs(400./(2*pi*freq))),'--')

legend('Dtheta/tau','Ddelta/tau','Dq/tau','Dq/Dtheta','Dq/Ddelta','Dq/delta');
% semilogx(fVct, 20*log10(reshape(abs(freqresp(sys,2*pi*fVct)),length(fVct),1)))
% semilogx(fVct, 20*log10(reshape(abs(freqresp(sys_est_2,2*pi*fVct)),length(fVct),1)))
% semilogx(fVct, 20*log10(reshape(abs(freqresp(sys_est_4,2*pi*fVct)),length(fVct),1)))
return
%%
figure
L=length(t);
Fs=1e3;
NFFT = 2^nextpow2(L); % Next power of 2 from length of y
Yfft = fft(ffw_torque(:,1),NFFT)/L;
Ytau = fft(tau(:,1),NFFT)/L;
YDtheta = fft(Dtheta(:,1),NFFT)/L;
f = Fs/2*linspace(0,1,NFFT/2+1);

% Plot single-sided amplitude spectrum.
plot(f,2*abs(Yfft(1:NFFT/2+1))) 
hold all
plot(f,2*abs(YDtheta(1:NFFT/2+1))*100) 
title('Single-Sided Amplitude Spectrum of y(t)')
xlabel('Frequency (Hz)')
ylabel('|Y(f)|')