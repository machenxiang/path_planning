function Q = getQ(n_seg, n_order, ts)   %n_seg is the number of the segement
    Q = [];
    for i = 1:n_seg
        Q_k = [];
        %#####################################################
        % STEP 1.1: calculate Q_k of the k-th segmen
        Q1=zeros(4,4);

       t=ts(i);
       Q4=[100800*t^7 50400*t^6 20160*t^5 5040*t^4;
           50400*t^6  25920*t^5 10800*t^4 2880*t^3;
           20160*t^5  10800*t^4 4800*t^3  1440*t^2;
           5040*t^4   2880*t^3  1440*t^2  573*t];
        Q_k=[Q4,Q1;
            Q1,Q1];
       
        Q = blkdiag(Q, Q_k); %build diag matrix according the input
    end
end
%the order of the unknow 

