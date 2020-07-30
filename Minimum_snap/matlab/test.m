clc; clear all;
syms p7 p6 p5 p4 p3 p2 p1 p0 t T1 ts;
f=p7*t^7+p6*t^6+p5*t^5+p4*t^4+p3*t^3+p2*t^2+p1*t+p0;
f_diff=diff(f,t,3)
% squ_f_4diff=f_4diff^2;
% res_temp=collect(squ_f_4diff,t);
% 
% res1=int(squ_f_4diff,0,T1);
% res=collect(res1,T1);
% 
% p=[p4 p5 p6 p7];
% Q=[576*T1     1440*T1^2   2880*T1^3   5040*T1^4;
%    1440*T1^2  4800*T1^3   10800*T1^4  20160*T1^5;
%    2880*T1^3  10800*T1^4  25920*T1^5  50400*T1^6;
%    5040*T1^4  20160*T1^5  50400*T1^6  100800*T1^7];
% p1=[p4;
%     p5;
%     p6;
%     p7];
% 
% J=p*Q*p1;
% J1=collect(J,T1);
% if(J1==res)
%     aaa=11111;
% end
% 
% 
% 
% 
% clc;clear all;

% 100800*p7^2*ts^7 + 100800*p6*p7*ts^6 + (25920*p6^2 + 40320*p5*p7)*ts^5 + 
% (10080*p4*p7 + 21600*p5*p6)*ts^4 + (4800*p5^2 + 5760*p4*p6)*ts^3 + 2880*p4*p5*ts^2 + 576*p4^2*ts
% 
%         Q1=zeros(4,4);Q2=zeros(4,4);Q3=zeros(4,4);
%         Q4=[576*ts   1440*ts^2   2880*ts^3   5040*ts^4;
%            1440*ts^2  4800*ts^3   10800*ts^4  20160*ts^5;
%            2880*ts^3  10800*ts^4  25920*ts^5  50400*ts^6;
%            5040*ts^4  20160*ts^5  50400*ts^6  100800*ts^7];
%        Q_k=[Q1,Q2;Q3,Q4];
%   p=[p0 p1 p2 p3 p4 p5 p6 p7];
%   p1=[p0;p1;p2;p3;p4;p5;p6;p7];
%   J=p*Q_k*p1;
%   collect(J,ts)
%   

% clc; clear all;
% a=zeros(4,1);
% for i=1:1:4
%     a(i,1)=1;
% end

