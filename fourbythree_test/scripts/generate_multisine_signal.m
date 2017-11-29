%clc;
close all;
clear all;

%% ADDPATH
addpath('/home/maxwell/itia_ws/src/itia_identification/itia_frequency_identification/scripts');

%% SET SIGNAL PARAMETERS
folder_base = '/home/maxwell/itia_ws/data/fourbythree/freq_test/TwoJointSignalJnt2';
folder_date = datestr(now,'yyyymmdd');
trj_namespace = 'fourbythree_trj';

fLim2 = [[0.5 1]; %001
  [1 2]; %006
  [1.5 3]; %007
  [1 4]; %002
  [2 8]; %003
  [4 16]; %004
  [5 10]; %011
  [8 20]; %005
  [15 25]; %008
  [20 35]; %009
  [25 40]]; %010

for iF=1:size(fLim2,1)
  trj_name = ['jnt2_',num2str(iF,'%03i'),'.bin'];
  fLim=fLim2(iF,:);
  
  Ts = 1e-3;   % sample time [s]  C5Gopen sampling period
  Fs = 1/Ts;        %sampling frequeny [Hz]     !!!! 10 Hz max freq
  complete_filename =  [ folder_base,filesep,trj_name];
  nRep = 20;
  
  % max_tau=48;
  max_tau=20;
  %max_qp=(60/180*pi); %JNT0
  %max_qp=(30/180*pi); %JNT1
  max_qp=(50/180*pi); %JNT1
  
  max_q=deg2rad(10);
  
  %fLim = [0.5 5];
  fBase=[];
  type=2; %==1 CF on tau, ==2 CF on qp, ==3 CF on q
  
  tf = max(30,nRep*1/fLim(1)); %final time [sec]
  time = (0:Ts:tf)';
  
  
  fVct=logspace(log10(fLim(1)),log10(fLim(2)),300)';
  nFreq=50;
  
  idxs=randperm(length(fVct));
  idxs=sort(idxs(1:nFreq));
  freq=[fBase;fVct(idxs)];
  amplitude=ones(size(freq));
  s=2*pi*freq;
  
  fprintf('[ %s ] Saving data in: \t %s \n', mfilename, complete_filename );
  
  %%
  
  % h=1/0.06206; Jnt0
  %h= 1/0.178; %Jnt1
  h= 1/0.01; %Jnt2
  J=0.03404/0.06206;
  % tau = h*qp_ff+J*qpp_ff
  % ctau=h*cqpp/s+J*cqpp -> cqpp=ctau/(h/s+J)
  
  clear qpp_ff qp_ff q_ff tau_ff
  if type==1
    %ctau=h*cqpp/s+J*cqpp -> cqpp=ctau/(h/s+J)
    [tau_ff(:,1),c_tau] = multisine_rand_phase( time, freq, amplitude, 1000);
    c_qpp=c_tau./(h./s+J);
    qpp_ff=fourier_series(time,freq,c_qpp);
    qp_ff=fourier_series(time,freq,c_qpp./(1j*2*pi*freq));
    q_ff=fourier_series(time,freq,c_qpp./((1j*2*pi*freq)).^2);
    csignal=fourier_coeff(time,freq,q_ff);
  elseif type==2
    %ctau=h*cqpp/s+J*cqpp
    [qp_ff(:,1),c_qp] = multisine_rand_phase( time, freq, amplitude, 1000);
    c_tau=c_qp.*(h+J*s);
    c_qpp=c_qp.*s;
    qpp_ff=fourier_series(time,freq,c_qpp);
    tau_ff=fourier_series(time,freq,c_tau);
    qp_ff=fourier_series(time,freq,c_qpp./(1j*2*pi*freq));
    q_ff=fourier_series(time,freq,c_qpp./((1j*2*pi*freq)).^2);
    csignal=fourier_coeff(time,freq,q_ff);
  elseif type==3
    %ctau=h*cqpp/s+J*cqpp
    [q_ff(:,1),c_q] = multisine_rand_phase( time, freq, amplitude, 1000);
    c_qp=c_q.*s;
    c_qpp=c_qp.*s;
    c_tau=c_qp.*(h+J*s);
    qpp_ff=fourier_series(time,freq,c_qpp);
    tau_ff=fourier_series(time,freq,c_tau);
    qp_ff=fourier_series(time,freq,c_qp);
    %q_ff=fourier_series(time,freq,c_qpp./((1j*2*pi*freq)).^2);
    csignal=fourier_coeff(time,freq,q_ff);
  end
  
  ratio_q=(max(q_ff)-mean(q_ff))/max_q;
  ratio_qp=max(abs(qp_ff))/max_qp;
  ratio_tau=max(abs(tau_ff))/max_tau;
  
  ratio=max([ratio_q;ratio_qp;ratio_tau]);
  
  q_ff=q_ff/ratio;
  qp_ff=qp_ff/ratio;
  tau_ff=tau_ff/ratio;
  
  fprintf('position = [%f,%f] [deg], RMS=%f [deg]\n',min(q_ff)*180/pi,max(q_ff)*180/pi,rms(q_ff)*180/pi)
  fprintf('max velocity = %f [deg/s] RMS=%f [deg/s]\n',max(qp_ff)*180/pi,rms(qp_ff)*180/pi)
  fprintf('max torque   = %f [Nm] RMS=%f [Nm]\n',max(tau_ff),rms(tau_ff))
  %%
  subplot(311)
  plot(time,q_ff)
  hold on
  plot(xlim',max_q*[1 1]'+mean(q_ff),'--k')
  plot(xlim',-max_q*[1 1]'+mean(q_ff),'--k')
  hold off
  
  subplot(312)
  plot(time,qp_ff)
  hold on
  plot(xlim',max_qp*[1 1]','--k')
  plot(xlim',-max_qp*[1 1]','--k')
  hold off
  
  subplot(313)
  plot(time,tau_ff)
  hold on
  plot(xlim',max_tau*[1 1]','--k')
  plot(xlim',-max_tau*[1 1]','--k')
  hold off
  
  fid=fopen(complete_filename ,'w');
  data=fwrite(fid,[q_ff;qp_ff;tau_ff],'double');
  fclose(fid);
  
  save([complete_filename(1:end-4),'.mat'],'csignal','freq');
end