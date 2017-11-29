function sys=analysisData(test_ident,test_vali,freq_ident,freq_vali,iAx_exitaction,iAx_output,type)
Ts=1e-3;
plot_fig=1;
stop_band_coeff=Ts / (0.2/freq_ident(end) + Ts);
start_band_coeff=Ts / (5/freq_ident(1) + Ts);
b=conv(stop_band_coeff,[1 -1]);
a=conv([1 stop_band_coeff-1],[1 start_band_coeff-1]);

idxs=5000:(length( test_ident.t)-5000);

if type==1
y_motor_ident = test_ident.fb_eff(idxs,4*iAx_exitaction-3);
y_motor_vali  = test_vali.fb_eff(idxs,4*iAx_exitaction-3);
y_link_ident  = test_ident.fb_pos(idxs,4*iAx_output-2);
y_link_vali   = test_vali.fb_pos(idxs,4*iAx_output-2);
elseif type==2
y_motor_ident = test_ident.fb_pos(idxs,4*iAx_exitaction-3);
y_motor_vali  = test_vali.fb_pos(idxs,4*iAx_exitaction -3);
y_link_ident  = test_ident.fb_pos(idxs,4*iAx_output);
y_link_vali   = test_vali.fb_pos(idxs,4*iAx_output);
else
  y_motor_ident = test_ident.fb_eff(idxs,4*iAx_exitaction-3);
y_motor_vali  = test_vali.fb_eff(idxs,4*iAx_exitaction-3);
y_link_ident  = test_ident.fb_pos(idxs,4*iAx_output);
y_link_vali   = test_vali.fb_pos(idxs,4*iAx_output);
end
t             = test_ident.t(idxs);
y_motor_ident=y_motor_ident-mean(y_motor_ident);
y_link_ident=y_link_ident-mean(y_link_ident);

y_motor_ident=filtfilt(b,a,y_motor_ident);
y_link_ident=filtfilt(b,a,y_link_ident);

y_motor_vali=y_motor_vali-mean(y_motor_vali);
y_link_vali=y_link_vali-mean(y_link_vali);

y_motor_vali=filtfilt(b,a,y_motor_vali);
y_link_vali=filtfilt(b,a,y_link_vali);

idxs=1000:(length(y_motor_ident)-1000);
y_motor_ident=y_motor_ident(idxs);
y_link_ident=y_link_ident(idxs);
y_motor_vali=y_motor_vali(idxs);
y_link_vali=y_link_vali(idxs);
t=t(idxs);


cy_motor_ident=fourier_coeff(t,freq_ident,y_motor_ident);
cy_link_ident=fourier_coeff(t,freq_ident,y_link_ident);

cy_motor_vali=fourier_coeff(t,freq_vali,y_motor_vali);
cy_link_vali=fourier_coeff(t,freq_vali,y_link_vali);

ym_motor_vali=fourier_series(t,freq_vali,cy_motor_vali);
ym_link_vali=fourier_series(t,freq_vali,cy_link_vali);
ym_motor_ident=fourier_series(t,freq_ident,cy_motor_ident);
ym_link_ident=fourier_series(t,freq_ident,cy_link_ident);


if type==1
  [sys,n_masses,n_additional_zeros,correlation]=select_number_of_masses(t,y_link_vali,freq_ident,freq_vali,cy_link_ident,cy_motor_ident,cy_link_vali,cy_motor_vali,[0:4],0:2,1:2,1);
elseif type==2
  [sys,n_masses,n_additional_zeros,correlation]=select_number_of_masses(t,y_link_vali,freq_ident,freq_vali,cy_link_ident,cy_motor_ident,cy_link_vali,cy_motor_vali,1:4,0:2,0,1);
else
  [sys,n_masses,n_additional_zeros,correlation]=select_number_of_masses(t,y_link_vali,freq_ident,freq_vali,cy_link_ident,cy_motor_ident,cy_link_vali,cy_motor_vali,[0:4],0:2,1:2,1);
end
if correlation<0.5
  sys=tf(0,1);
  n_masses=-1;
  correlation=0;
end

H=freqresp(sys,freq_ident*2*pi);
H=H(:);

str=sprintf('link %d/motor %d',iAx_output,iAx_exitaction);
legenda={[str, ' identification'],[str, ' validation'],sprintf('%s estimated %d masses, CORR=%f%%',str,n_masses,correlation*100)};

if plot_fig
  title(sprintf('Output: Joint %d',iAx_output))
  semilogx(freq_ident, 20*log10(abs(cy_link_ident./cy_motor_ident)),'o')
  hold all
  semilogx(freq_vali, 20*log10(abs(cy_link_vali./cy_motor_vali)),'d')
  semilogx(freq_ident, 20*log10(abs(H)),'-','LineWidth',2)
  grid on
  box on
  xlim([freq_ident(1) freq_ident(end)])
  ylabel('Magnitude');
  xlabel('Frequency [Hz]');
  leg=legend();
  if isempty(leg)
    legenda_tot=legenda;
  else
    legenda_tot=[leg.String,legenda];
  end
  legend(legenda_tot,'Location','southwest');
  %set(gca,'XTick',[0.5:0.1:1 2:10])
  set(gca,'YTickLabel',10.^(get(gca,'YTick')/20))
  drawnow
  
%   title(sprintf('Input: Joint %d',iAx_exitaction))
%   semilogx(freq_ident, 20*log10(abs(cy_link_ident./cy_motor_ident)),'o')
%   hold all
%   semilogx(freq_vali, 20*log10(abs(cy_link_vali./cy_motor_vali)),'d')
%   semilogx(freq_ident, 20*log10(abs(H)),'-','LineWidth',2)
%   grid on
%   box on
%   xlim([freq_ident(1) freq_ident(end)])
%   ylabel('Magnitude');
%   xlabel('Frequency [Hz]');
%   leg=legend();
%   if isempty(leg)
%     legenda_tot=legenda;
%   else
%     legenda_tot=[leg.String,legenda];
%   end
%   legend(legenda_tot,'Location','southwest');
%   set(gca,'XTick',[0.5:0.1:1 2:10 20 30])
%   set(gca,'YTickLabel',10.^(get(gca,'YTick')/20))
%   drawnow
end
