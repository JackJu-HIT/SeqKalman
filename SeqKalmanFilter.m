%@Function：本程序主要实现序惯卡尔曼滤波算法
%@Author：Juchunyu
%@Date：2022-2-4 09:18:00
function [pRes kRes x] =SeqKalmanFilter(F,H,Q,v,R,P0,x0,y,meassureNum,x)
    pRes = [];
    kRes = [];
    xRes=[];
    %predict model
    P = F*P0*F'+Q
    X = F*x0
    for i=1:meassureNum
        %参数初始化处理
        if i==1
            p=P(1,1);
        else
            p=P_;
        end  
        [mh,nh]=size(H);
        if(i>mh)
            h=0;
        else
            h=H(i,1) 
        end
        r= R(i,i)
        %计算测量更新
        k = p*h/(h*p*h'+r)%卡尔曼增益
        kRes = [kRes;k];%记录卡尔曼增益k的计算结果
        xPredict = x(i)+k*(y(i,:)-h*x(i,1))%卡尔曼预测出的状态
        x=[x;xPredict]
        [M,N] = size(k*h)
        P_ = (ones(M,N)-k*h)*p   %卡尔曼输出的协方差
        pRes=[pRes;P_];%记录卡尔曼协方差的计算结果
    end
end