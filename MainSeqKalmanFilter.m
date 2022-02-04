clear
%%%%参数设置%%%%
F=[0.95];%%状态矩阵
H = [1 0.2 0.02]';%观测矩阵
Q = [2];   
v=[2 1 50]
R = diag(v); 
%初始值设置
P0 = 4;
x0 = 1;
x = [x0];
%测量值的数量
meassureNum = 3;
%%观测值
y = [6 3 -100]';
[pRes0 kRes xRes] =SeqKalmanFilter(F,H,Q,v,R,P0,x0,y,meassureNum,x);