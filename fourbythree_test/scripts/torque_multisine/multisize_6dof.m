ccc
ndof=6;
iJoint=2;
marker={'o','d'};

pos_prefix='';
first_time(1:6)=true;

config=ReadYaml(sprintf('/home/maxwell/projects/itia_ws/src/itia_fourbythree_driver/fourbythree_test/cfg/torque_multisine_joint%d.yaml',iJoint),1);

pathname='/home/maxwell/test/fourbythree/torque_multisine/test_p1/';
if ~isempty(pos_prefix)
  test_prefix={sprintf('multisine_jnt%d_%s',iJoint,pos_prefix)};
else
  test_prefix={sprintf('multisine_jnt%d',iJoint)};
end

cDq     = {[],[],[],[],[],[]};
cDtheta = {[],[],[],[],[],[]};
cDdelta  = {[],[],[],[],[],[]};

cq     = {[],[],[],[],[],[]};
ctheta = {[],[],[],[],[],[]};
cdelta  = {[],[],[],[],[],[]};

ctau    = {[],[],[],[],[],[]};
cfreq   = {[],[],[],[],[],[]};

spring=920;
for iPrefix=1:length(test_prefix)
  for iTest=1:length(config.multisine.test_namespace)
    testname=config.multisine.test_namespace{iTest};
    complete_testname=sprintf('%s__%s',test_prefix{iPrefix},testname);
    if ~exist([pathname complete_testname '_JointState__elastic_joint_states.bin'],'file')
      error;
    end
    elastic_js=bin_convert([pathname,complete_testname,'_JointState__elastic_joint_states.bin'],2*ndof*3+1);
    ffw_js=bin_convert([pathname,complete_testname,'_JointState__joint_feedforward.bin'],ndof*3+1);
    sp_js=bin_convert([pathname,complete_testname,'_JointState__sp_joint_states.bin'],ndof*3+1);
    data=bin_resampling({elastic_js,ffw_js,sp_js},1e-3);
    
    clear nat_freq test_duration
    nat_freq=[config.(testname).natural_frequency{:}];
    test_duration=config.(testname).test_duration;
    
    t=data{1}(:,1)-data{1}(1,1);
    idxs=find((t>10).*(t<(test_duration-5)));
    theta=data{1}(idxs,1+[1:2:(2*ndof)]);
    Dtheta=data{1}(idxs,1+2*ndof+[1:2:(2*ndof)]);
    delta=data{1}(idxs,1+[2:2:(2*ndof)]);
    Ddelta=data{1}(idxs,1+2*ndof+[2:2:(2*ndof)]);
    q=theta+delta;
    Dq=Dtheta+Ddelta;
    
    
    tau=data{1}(idxs,1+4*ndof+[1:2:(2*ndof)]);
    ffw_torque=data{2}(idxs,1+2*ndof+[1:ndof]);
    t=t(idxs)-t(idxs(1));
    
    
    
    
    
    freq=sort(nat_freq)/2/pi;
    c_ffw=fourier_coeff(t,freq,ffw_torque(:,iJoint));
    c_tau=fourier_coeff(t,freq,tau(:,iJoint));
    
    c_Dtheta=fourier_coeff(t,freq,Dtheta(:,iJoint));
    c_theta=fourier_coeff(t,freq,theta(:,iJoint));

    c_Dq=fourier_coeff(t,freq,Dq(:,iJoint));
    c_q=fourier_coeff(t,freq,q(:,iJoint));
    
    c_delta=fourier_coeff(t,freq,delta(:,iJoint));
    c_Ddelta=fourier_coeff(t,freq,Ddelta(:,iJoint));
    
    ffw=fourier_series(t,freq,c_ffw);
    tau_fourier=fourier_series(t,freq,c_tau);
    Dtheta_fourier=fourier_series(t,freq,c_Dtheta);
    
    cDq{iJoint}=[cDq{iJoint};c_Dq];
    cDtheta{iJoint}=[cDtheta{iJoint};c_Dtheta];
    cDdelta{iJoint}=[cDdelta{iJoint};c_Ddelta];
    
    cq{iJoint}=[cq{iJoint};c_q];
    ctheta{iJoint}=[ctheta{iJoint};c_theta];
    cdelta{iJoint}=[cdelta{iJoint};c_delta];
    
    ctau{iJoint}=[ctau{iJoint};c_tau];
    cfreq{iJoint}=[cfreq{iJoint};freq'];
    
    figure(1)
    h(1)=subplot(211);
    semilogx(freq, 20*log10(abs(c_Dtheta./c_tau)),marker{iPrefix},'Color','g')
    hold all
    semilogx(freq, 20*log10(abs(-spring*c_delta./c_tau)),marker{iPrefix},'Color','k')
    semilogx(freq, 20*log10(abs(c_Dq./c_tau)),marker{iPrefix},'Color','r')
    semilogx(freq, 20*log10(abs(c_Dq./(-spring*c_delta))),marker{iPrefix},'Color','b')
    grid on
    h(2)=subplot(212);
    semilogx(freq, 180/pi*(angle(c_Dtheta./c_tau)),marker{iPrefix},'Color','g')
    hold all
    semilogx(freq, 180/pi*(angle(-spring*c_delta./c_tau)),marker{iPrefix},'Color','k')
    semilogx(freq, 180/pi*(angle(c_Dq./c_tau)),marker{iPrefix},'Color','r')
    semilogx(freq, 180/pi*(angle(c_Dq./(-spring*c_delta))),marker{iPrefix},'Color','b')
    
    if first_time(iJoint)
      legend('Dtheta/tau','tau_{el}/tau','Dq/tau','Dq/tau_{el}');
      first_time(iJoint)=false;
    end
    grid on
    drawnow
    %return
  end
end

linkaxes(h,'x')


for idx=1:length(cfreq)
  [cfreq{idx},idxs]=sort(cfreq{idx});
  cq{idx}    =cq{idx}(idxs);
  ctheta{idx}=ctheta{idx}(idxs);
  cdelta{idx}=cdelta{idx}(idxs);

  cDq{idx}=cDq{idx}(idxs);
  cDtheta{idx}=cDtheta{idx}(idxs);
  cDdelta{idx}=cDdelta{idx}(idxs);
  ctau{idx}=ctau{idx}(idxs);
end

save(test_prefix{1},'cDq','cDtheta','cDdelta','cq','ctheta','cdelta','ctau','cfreq')
return
%%
semilogx(cfreq{iJoint}, 20*log10(abs(cDq{iJoint}./ctau{iJoint})),marker{iPrefix},'Color','g')
