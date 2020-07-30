clc;clear;close all;
path = ginput() * 100.0;

n_order = 7;
n_seg = size(path, 1) - 1;
n_poly_perseg = n_order + 1;

ts = zeros(n_seg, 1);
% calculate time distribution based on distance between 2 points
dist = zeros(n_seg, 1);
dist_sum = 0;
T = 25;

t_sum = 0;
for i = 1:n_seg
    dist(i) = sqrt((path(i+1, 1) - path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
    dist_sum = dist_sum + dist(i);
end
for i = 1:n_seg-1
    ts(i) = dist(i) / dist_sum * T;
    t_sum = t_sum + ts(i);
end
ts(n_seg) = T - t_sum;
% or you can simply average the time
% for i = 1:n_seg
%     ts(i) = 1.0;
% end

poly_coef_x_close_form = MinimumSnapCloseformSolver(path(:, 1), ts, n_seg, n_order);
poly_coef_y_close_form  = MinimumSnapCloseformSolver(path(:, 2), ts, n_seg, n_order);

poly_coef_x_QP = MinimumSnapQPSolver(path(:, 1), ts, n_seg, n_order);
poly_coef_y_QP = MinimumSnapQPSolver(path(:, 2), ts, n_seg, n_order);

X_n_close_form = [];
Y_n_close_form = [];
X_n_QP = [];
Y_n_QP = [];
k = 1;
tstep = 0.01;
for i=0:n_seg-1
    %#####################################################
    % STEP 4: get the coefficients of i-th segment of both x-axis
    % and y-axis


    Pxi_close_form=poly_coef_x_close_form(i*8+1:i*8+8);
    Pyi_close_form=poly_coef_y_close_form(i*8+1:i*8+8);
    
    Pxi_QP=poly_coef_x_QP(i*8+1:i*8+8);
    Pyi_QP=poly_coef_y_QP(i*8+1:i*8+8);
    for t=0:tstep:ts(i+1)
        X_n_close_form(k)  = polyval(Pxi_close_form,t);
        Y_n_close_form(k)  = polyval(Pyi_close_form,t);
        X_n_QP(k)  = polyval(Pxi_QP, t);
        Y_n_QP(k)  = polyval(Pyi_QP, t);
        k = k+1;
    end
end

plot(X_n_QP, Y_n_QP , 'Color', [0 0 1], 'LineWidth', 2);
hold on;
plot(path(:,1),path(:,2),'Color',[1 0 0],'LineWidth',2);
hold on
plot(X_n_close_form, Y_n_close_form ,'Color',[0 1 0],'LineWidth',1);
hold on
scatter(path(1:size(path,1),1),path(1:size(path,1),2));

function poly_coef = MinimumSnapCloseformSolver(waypoints, ts, n_seg, n_order)
    start_cond = [waypoints(1), 0, 0, 0];
    end_cond =   [waypoints(end), 0, 0, 0];
    %#####################################################
    % you have already finished this function in hw1
    Q = getQ(n_seg, n_order, ts);
    %#####################################################
    % STEP 1: compute M
    M = getM(n_seg, n_order, ts);
    %#####################################################
    % STEP 2: compute Ct
    Ct = getCt(n_seg, n_order);
    C = Ct';
    R = C * inv(M)' * Q * inv(M) * Ct;%ppt page 46
    R_cell = mat2cell(R, [n_seg+7 3*(n_seg-1)], [n_seg+7 3*(n_seg-1)]);
    R_pp = R_cell{2, 2};
    R_fp = R_cell{1, 2};
    %#####################################################
    % STEP 3: compute dF
    dF = [];
    
    dF(1:4,1) = start_cond';% dF for first pos,vel,acc,jerk
    dF(5:4+(n_seg-1),1) = waypoints(2:end-1);% dF for middle pos
    dF(5+(n_seg-1):8+(n_seg-1)) = end_cond';% dF for last pos,vel,acc,jerk
    
    dP = -inv(R_pp) * R_fp' * dF;% ppt page 46 dp* is the answer
    poly_coef = inv(M) * Ct * [dF;dP];%page 45 M*p=d--->p=inv(M)*d---->d=Ct*[dF;dP]---->p=inv(M) * Ct * [dF;dP]
end


function poly_coef = MinimumSnapQPSolver(waypoints, ts, n_seg, n_order)
    start_cond = [waypoints(1), 0, 0, 0];
    end_cond   = [waypoints(end), 0, 0, 0];
    %#####################################################
    % STEP 1: compute Q of p'Qp
    Q = getQ(n_seg, n_order, ts);
    %#####################################################
    % STEP 2: compute Aeq and beq 
    [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond);
    f = zeros(size(Q,1),1);
    poly_coef = quadprog(Q,f,[],[],Aeq, beq);
end
