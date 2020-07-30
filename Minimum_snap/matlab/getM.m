function M = getM(n_seg, n_order, ts)
    M = [];
    for k = 1:n_seg
        M_k = [];
        %#####################################################
        % STEP 1.1: calculate M_k of the k-th segment 
        t=ts(k);
        M_k=[0 0 0 0 0 0 0 1;%0th derivative
             0 0 0 0 0 0 1 0;%1th derivative
             0 0 0 0 0 2 0 0;%2th derivative
             0 0 0 0 6 0 0 0;
             t^7 t^6 t^5 t^4 t^3 t^2 t^1 t^0;
             7*t^6 6*t^5 5*t^4 4*t^3 3*t^2 2*t 1 0;
             42*t^5 30*t^4 20*t^3 12*t^2 6*t 2 0 0;
             210*t^4 120*t^3 60*t^2 24*t 6 0 0 0];%3th 
        M = blkdiag(M, M_k);
    end
end