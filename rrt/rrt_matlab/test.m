clear all;close all;clc;
x_I=1; y_I=1;           % 设置初始点
T.v(1).x = x_I;         % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
T.v(1).y = y_I; 


T.v(2).x = x_I;         % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
T.v(2).y = y_I; 

T.v(3).x = x_I;         % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
T.v(3).y = y_I;

size=size(T.v)
ind=size(1,2);

