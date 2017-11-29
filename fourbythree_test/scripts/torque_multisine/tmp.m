ccc
ndof=6;
iJoint=2;
marker={'o','d'};

pos_prefix='P2';
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

iPrefix=1;
iTest=1;
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
ffw_torque=data{2}(idxs,1+ndof+[1:ndof]);
t=t(idxs)-t(idxs(1));


Dsys=tf([1 0],[0.01 1]);
DDsys=Dsys^2;
Dtheta2=lsim(Dsys,theta(:,2),t);
DDtheta2=lsim(DDsys,theta(:,2),t);

h(1)=subplot(311);
plot(t,Dtheta(:,2),t,Dtheta2)
ylim([-.1 .1])
h(2)=subplot(312);
plot(t,theta(:,2))
h(3)=subplot(313);
plot(t,tau(:,2))
linkaxes(h,'x')

% J*DDtheta=tau+f*Dtheta+c*sign(Dtheta)+k*delta+h*Ddelta
k=920;
M=[tau(:,2) Dtheta2 sign(Dtheta2) Ddelta(:,2) delta(:,2) sign(delta(:,2))];

idxs=abs(Dtheta2)>2.0e-2;
par=M(idxs,:)\(DDtheta2(idxs,1))
return
