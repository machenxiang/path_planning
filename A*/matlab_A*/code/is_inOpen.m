function flag=is_inOpen(open,opencount,x,y)
%flag=1ÔÚopen
%flag2ÔÚcloseÖĞ
for i=1:1:opencount
    if(open(i,2)==x&&open(i,3)==y&&open(i,1)==1)
        flag=1;
    elseif(open(i,2)==x&&open(i,3)==y&&open(i,1)==0)
        flag=2;
    else
        flag=0;
    end
end
end