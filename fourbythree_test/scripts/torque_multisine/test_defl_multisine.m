clear all;close all;clc;

config{1}=ReadYaml('/home/maxwell/projects/itia_ws/src/itia_fourbythree_driver/fourbythree_test/cfg/defl_multisine_jnt1.yaml',1);

pathname='/home/maxwell/test/fourbythree/defl_multisine/full_arm/jnt1/';


first_time(1:2)=true;

%test_prefix={'multisine_20170323_P1_test','multisine_20170323_test'};

test_prefix={'multisine__deflection'};
marker={'o','d'};

cDq     = {[],[]};
cDtheta = {[],[]};
cdelta  = {[],[]};
ctau    = {[],[]};
cffw    = {[],[]};
cfreq   = {[],[]};
%multisine__deflection_jnt1_test1__JointState__elastic_joint_states.bin
for iPrefix=1:length(test_prefix)
  for iTest=1:14
    for iJoint=1:2
      complete_testname=sprintf('%s_jnt%d_test%d_',test_prefix{iPrefix},iJoint,iTest);
      testname=sprintf('deflection_jnt%d_test%d_',iJoint,iTest);
      if exist([pathname complete_testname '_JointState__elastic_joint_states.bin'],'file')
        break;
      end
    end
    
    elastic_js=bin_convert([pathname,complete_testname,'_JointState__elastic_joint_states.bin'],4*3+1);
    ffw_js=bin_convert([pathname,complete_testname,'_JointState__rigid_joint_feedforward.bin'],2*3+1);
    sp_js=bin_convert([pathname,complete_testname,'_JointState__sp_joint_states.bin'],2*3+1);
    data=bin_resampling({elastic_js,ffw_js,sp_js},1e-3);
    
    clear nat_freq test_duration
    for iConfig=1:length(config)
      try
        nat_freq=[config{iConfig}.(testname).natural_frequency{:}];
        test_duration=config{iConfig}.(testname).test_duration;
        break;
      end
    end
    t=data{1}(:,1)-data{1}(1,1);
    idxs=find((t>10).*(t<(test_duration-5)));
    theta=data{1}(idxs,1+[1 3]);
    Dtheta=data{1}(idxs,5+[1 3]);
    delta=data{1}(idxs,1+[2 4]);
    Ddelta=data{1}(idxs,5+[2 4]);
    q=theta+delta;
    Dq=Dtheta+Ddelta;
    
    
    tau=data{1}(idxs,9+[1 3]);
    ffw_pos=data{2}(idxs,1+[1:2]);
    t=t(idxs)-t(idxs(1));
    
    
    %%
%     iJoint= find(sum(abs(ffw_torque)),1);
    
    
    
    freq=sort(nat_freq)/2/pi;
    c_ffw=fourier_coeff(t,freq,ffw_pos(:,iJoint));
    c_tau=fourier_coeff(t,freq,tau(:,iJoint));
    c_Dtheta=fourier_coeff(t,freq,Dtheta(:,iJoint));
    c_Dq=fourier_coeff(t,freq,Dq(:,iJoint));
    c_q=fourier_coeff(t,freq,q(:,iJoint));
    c_delta=fourier_coeff(t,freq,delta(:,iJoint));
    c_Ddelta=fourier_coeff(t,freq,Ddelta(:,iJoint));
    
    ffw=fourier_series(t,freq,c_ffw);
    tau_fourier=fourier_series(t,freq,c_tau);
    Dtheta_fourier=fourier_series(t,freq,c_Dtheta);
    
    cDq{iJoint}=[cDq{iJoint};c_Dq];
    cDtheta{iJoint}=[cDtheta{iJoint};c_Dtheta];
    cdelta{iJoint}=[cdelta{iJoint};c_delta];
    ctau{iJoint}=[ctau{iJoint};c_tau];
    cffw{iJoint}=[cffw{iJoint};c_ffw];
    cfreq{iJoint}=[cfreq{iJoint};freq'];
    
    figure(3)
    plot(t,ffw,t,delta(:,iJoint))
    pause
    
    figure(iJoint)
    subplot(211)
    semilogx(freq, 20*log10(abs(c_Dtheta./c_tau)),marker{iPrefix},'Color','g')
    hold all
    semilogx(freq, 20*log10(abs(c_delta./c_tau)),marker{iPrefix},'Color','k')
    semilogx(freq, 20*log10(abs(c_Dq./c_tau)),marker{iPrefix},'Color','r')
    semilogx(freq, 20*log10(abs(c_Dq./c_delta)),marker{iPrefix},'Color','b')
    semilogx(freq, 20*log10(abs(c_delta./c_ffw)),marker{iPrefix},'Color','m')
    subplot(212)
    semilogx(freq, 180/pi*(angle(c_Dtheta./c_tau)),marker{iPrefix},'Color','g')
    hold all
    semilogx(freq, 180/pi*(angle(c_delta./c_tau)),marker{iPrefix},'Color','k')
    semilogx(freq, 180/pi*(angle(c_Dq./c_tau)),marker{iPrefix},'Color','r')
    semilogx(freq, 180/pi*(angle(c_Dq./c_delta)),marker{iPrefix},'Color','b')
    
    if first_time(iJoint)
      freq=logspace(-1,2)';
      subplot(211)
      semilogx(freq, 20*log10(abs(1e4./(2*pi*freq))),'-m')
      
      legend('Dtheta/tau','delta/tau','Dq/tau','Dq/delta','delta/ffw');
      first_time(iJoint)=false;
    end
    grid on
    drawnow
    %return
  end
end
figure(1)
h(1)=gca;
figure(2)
h(2)=gca;

linkaxes(h,'x')


for idx=1:length(cfreq)
  [cfreq{idx},idxs]=sort(cfreq{idx});
  cDq{idx}=cDq{idx}(idxs);
  cDtheta{idx}=cDtheta{idx}(idxs);
  ctau{idx}=ctau{idx}(idxs);
  cdelta{idx}=cdelta{idx}(idxs);
end

%save home cDq cDtheta cdelta ctau cfreq
return
%%
semilogx(cfreq{iJoint}, 20*log10(abs(cDq{iJoint}./ctau{iJoint})),marker{iPrefix},'Color','g')
    