function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
%waypoints is the path get from the mouse
% n_seg segment number n point just have n-1 segements n_order is the order
%waypoint is x or y aixs of the point 
% according the document of quadprog Aeq*x=beq x=[]

    n_all_poly = n_seg*(n_order+1);% the number of the unknow,the 
    %#####################################################
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    beq_start = zeros(4, 1);
    % STEP 2.1: write expression of Aeq_start and beq_start
    Aeq_start = [0 0 0 0 0 0 0 1 zeros(1,n_all_poly-8);
                 0 0 0 0 0 0 1 0 zeros(1,n_all_poly-8);
                 0 0 0 0 0 2 0 0 zeros(1,n_all_poly-8);
                 0 0 0 0 6 0 0 0 zeros(1,n_all_poly-8)];
    beq_start=start_cond';
    
    %#####################################################
    %first derivative 7*p7*t^6 + 6*p6*t^5 + 5*p5*t^4 + 4*p4*t^3 + 3*p3*t^2 + 2*p2*t + p1
    %second 42*p7*t^5 + 30*p6*t^4 + 20*p5*t^3 + 12*p4*t^2 + 6*p3*t + 2*p2
    %third 210*p7*t^4 + 120*p6*t^3 + 60*p5*t^2 + 24*p4*t + 6*p3
    
    % p,v,a constraint in end 
    Aeq_end = zeros(4, n_all_poly);
    beq_end = zeros(4, 1);
    % STEP 2.2: write expression of Aeq_end and beq_end
    %ts(end) is the last segment time
    t=ts(end);
    Aeq_end = [zeros(1,(n_seg-1)*8) t^7      t^6      t^5      t^4      t^3      t^2    t^1     1;
               zeros(1,(n_seg-1)*8) 7*t^6    6*t^5    5*t^4    4*t^3    3*t^2    2*t    1       0;
               zeros(1,(n_seg-1)*8) 42*t^5   30*t^4   20*t^3   12*t^2   6*t      2      0       0;  
               zeros(1,(n_seg-1)*8) 210*t^4  120*t^3  60*t^2   24*t     6        0      0       0 ];       
    beq_end=end_cond';
    
    %#####################################################
    % position constrain in all middle waypoints
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    % STEP 2.3: write expression of Aeq_wp and beq_wp
    for i=1:1:n_seg-1
        t=ts(i);
        beq_wp(i)=waypoints(i+1);%middle waypoints begin at 2 end at n_seg 
        Aeq_wp(i,(i-1)*8+1:(i-1)*8+8)=[t^7 t^6 t^5 t^4 t^3 t^2 t^1 1];
    end
    %#####################################################
    % position continuity constrain between each 2 segments
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    % STEP 2.4: write expression of Aeq_con_p and beq_con_p
    for i=1:1:n_seg-1
        t=ts(i);
        Aeq_con_p(i,(i-1)*8+1:(i-1)*8+8)=[t^7 t^6 t^5 t^4 t^3 t^2 t^1 1];
        Aeq_con_p(i,(i-1)*8+9:(i-1)*8+16)=[0 0 0 0 0 0 0 -1];
    end
    
    %#####################################################
    % velocity continuity constrain between each 2 segments
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    %first derivative 7*p7*t^6 + 6*p6*t^5 + 5*p5*t^4 + 4*p4*t^3 + 3*p3*t^2 + 2*p2*t + p1
    % STEP 2.5: write expression of Aeq_con_v and beq_con_v
    for i=1:1:n_seg-1
        t=ts(i);
        Aeq_con_v(i,(i-1)*8+1:(i-1)*8+8)=[7*t^6 6*t^5 5*t^4 4*t^3 3*t^2 2*t 1 0];
        Aeq_con_v(i,(i-1)*8+9:(i-1)*8+16)=[0 0 0 0 0 0 -1 0];
    end
    %#####################################################
    % acceleration continuity constrain between each 2 segments
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    %second 42*p7*t^5 + 30*p6*t^4 + 20*p5*t^3 + 12*p4*t^2 + 6*p3*t + 2*p2
    % STEP 2.6: write expression of Aeq_con_a and beq_con_a
    for i=1:1:n_seg-1
        t=ts(i);
        Aeq_con_a(i,(i-1)*8+1:(i-1)*8+8)=[42*t^5 30*t^4 20*t^3 12*t^2 6*t 2 0 0];
        Aeq_con_a(i,(i-1)*8+9:(i-1)*8+16)=[0 0 0 0 0 -2 0 0];
    end

    
    %#####################################################
    % jerk continuity constrain between each 2 segments
    Aeq_con_j = zeros(n_seg-1, n_all_poly);
    beq_con_j = zeros(n_seg-1, 1);
    %third 210*p7*t^4 + 120*p6*t^3 + 60*p5*t^2 + 24*p4*t + 6*p3
    % STEP 2.7: write expression of Aeq_con_j and beq_con_j
    for i=1:1:n_seg-1
        t=ts(i);
        Aeq_con_j(i,(i-1)*8+1:(i-1)*8+8)=[210*t^4 120*t^3 60*t^2 24*t 6 0 0 0];
        Aeq_con_j(i,(i-1)*8+9:(i-1)*8+16)=[0 0 0 0 -6 0 0 0];
    end
    
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
    beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];

    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];

end

