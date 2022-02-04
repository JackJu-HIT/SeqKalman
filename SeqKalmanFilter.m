%@Function����������Ҫʵ����߿������˲��㷨
%@Author��Juchunyu
%@Date��2022-2-4 09:18:00
function [pRes kRes x] =SeqKalmanFilter(F,H,Q,v,R,P0,x0,y,meassureNum,x)
    pRes = [];
    kRes = [];
    xRes=[];
    %predict model
    P = F*P0*F'+Q
    X = F*x0
    for i=1:meassureNum
        %������ʼ������
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
        %�����������
        k = p*h/(h*p*h'+r)%����������
        kRes = [kRes;k];%��¼����������k�ļ�����
        xPredict = x(i)+k*(y(i,:)-h*x(i,1))%������Ԥ�����״̬
        x=[x;xPredict]
        [M,N] = size(k*h)
        P_ = (ones(M,N)-k*h)*p   %�����������Э����
        pRes=[pRes;P_];%��¼������Э����ļ�����
    end
end