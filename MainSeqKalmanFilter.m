clear
%%%%��������%%%%
F=[0.95];%%״̬����
H = [1 0.2 0.02]';%�۲����
Q = [2];   
v=[2 1 50]
R = diag(v); 
%��ʼֵ����
P0 = 4;
x0 = 1;
x = [x0];
%����ֵ������
meassureNum = 3;
%%�۲�ֵ
y = [6 3 -100]';
[pRes0 kRes xRes] =SeqKalmanFilter(F,H,Q,v,R,P0,x0,y,meassureNum,x);