function res=clac(point1,point2)%����xnear��x_random��������������
temp=[];step=30;res=[];
temp(1,1)=point2(1,1)-point1(1,1);
temp(1,2)=point2(1,2)-point1(1,2);
ang=atan2(temp(1,2),temp(1,1));
res(1,1)=step*cos(ang);
res(1,2)=step*sin(ang);
end