function ct = getCt(n_seg, n_order)
    %#####################################################
    % STEP 2.1: finish the expression of Ct
    num_mid_point=n_seg-1;% number of mid point
    num_fix_cons=2*4+n_seg-1;%number of fix constraint
    num_free=3*(n_seg-1);%number of dp
    rows=8*n_seg;
    cols=4*n_seg+4;
    
    ct=zeros(rows,cols);
    %the p v a j of the start point
    ct_start=zeros(4,cols);
    ct_start(:,1:4)=eye(4);
    ct(1:4,:)=ct_start;
    %the p v a j of the end point
    ct_end=zeros(4,cols);
    ct_end(:,(num_fix_cons-3):num_fix_cons)=eye(4);
    ct((rows-3):rows,:)=ct_end;
    for i=1:num_mid_point
    %the pos of the mid point,this place write 2 because the value between
    %the 2 point is same
    ct(5+8*(i-1),4+i)=1; 
    ct(4+(5+8*(i-1)),4+i)=1;
    
    %the v of the mid point
    ct(6+8*(i-1),num_fix_cons+1+(i-1)*3)=1;
    ct(4+(6+8*(i-1)),num_fix_cons+1+(i-1)*3)=1;
    
    %the a of the mid point
    ct(7+8*(i-1),num_fix_cons+2+(i-1)*3)=1;
    ct(4+(7+8*(i-1)),num_fix_cons+2+(i-1)*3)=1;
    
    %the jerk of the mid point
    ct(8+8*(i-1),num_fix_cons+3+(i-1)*3)=1;
    ct(4+(8+8*(i-1)),num_fix_cons+3+(i-1)*3)=1;
    end
    
end


