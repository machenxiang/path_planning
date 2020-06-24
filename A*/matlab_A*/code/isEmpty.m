function res=isEmpty(open,opencount)
for i=1:1:opencount
    if(open(i,1)==1)
        res=0;
        break;
    else
        res=1;
    end
end
end
