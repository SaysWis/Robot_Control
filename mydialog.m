function[Kp,Ki]= mydialog
prompt = {'Proportional Gain (Kp):','Integral Gain (Ki):'};
dlgtitle = 'PID Controller Gains';
dims = [1 50; 1 50];
definput = {'0.8','0.001'};
answer = inputdlg(prompt,dlgtitle,dims,definput);
values = str2double(answer);

%%
Kp = values(1);
Ki = values(2);
end