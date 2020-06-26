%***************************************
%Author: Chaoqun Wang
%Date: 2019-10-15
%***************************************
%% ���̳�ʼ��
clear all; close all;
x_I=1; y_I=1;           % ���ó�ʼ��
x_G=700; y_G=700;       % ����Ŀ���
Thr=20;                 %����Ŀ�����ֵ
Delta= 30;              % ������չ����
%% ������ʼ��ʹ����matlab�ṹ��
T.v(1).x = x_I;         % T������Ҫ��������v�ǽڵ㣬�����Ȱ���ʼ����뵽T������
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     % ��ʼ�ڵ�ĸ��ڵ���Ȼ���䱾��
T.v(1).yPrev = y_I;
T.v(1).dist=0;          %�Ӹ��ڵ㵽�ýڵ�ľ��룬�����ȡŷ�Ͼ���
T.v(1).indPrev = 0;     %�ڵ����������������Ľڵ�����������������
%% ��ʼ������������ҵ����
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,1);%��ͼx�᳤��
yL=size(Imp,2);%��ͼy�᳤��
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');% ��������Ŀ���
count=1;
for iter = 1:3000
    x_rand=[];
    %Step 1: �ڵ�ͼ���������һ����x_rand
    %��ʾ���ã�x_rand(1),x_rand(2)����ʾ�����в����������
    x_rand(1,1)=rand(1)*xL;
    x_rand(1,2)=rand(1)*yL;
    %scatter(x_rand(1,1),x_rand(1,2));
    
    %Step 2: ���������������ҵ�����ڽ���x_near 
    %��ʾ��x_near�Ѿ�����T��
    x_near=[];
    dis=distance(T.v(1).x,T.v(1).y,x_rand(1),x_rand(2));
    min_index=1;
    for iter1=1:count%�ҵ�����ýڵ������
        temp_dis=distance(T.v(iter1).x,T.v(iter1).y,x_rand(1),x_rand(2));
        if(temp_dis<dis)
            min_index=iter1;
            dis=temp_dis;
        end
    end
    x_near(1,1)=T.v(min_index).x;
    x_near(1,2)=T.v(min_index).y;
    
    x_new=[];
    %Step 3: ��չ�õ�x_new�ڵ�
    %��ʾ��ע��ʹ����չ����Delta
    
    %���ڵ��Ƿ���collision-free
    %if ~collisionChecking(x_near,x_new,Imp) 
    %    continue;
    %end
    res=clac(x_near,x_rand);
    x_new(1,1)=x_near(1,1)+res(1,1);
    x_new(1,2)=x_near(1,2)+res(1,2);
    if ~collisionChecking(x_near,x_new,Imp) 
       continue;
    end
    count=count+1;%�ڵ��������ڵ�����������Ч�ڵ㼴ΪT�нڵ�����
    %Step 4: ��x_new������T 
    %��ʾ���½ڵ�x_new�ĸ��ڵ���x_near
    T.v(count).x = x_new(1,1);         % T������Ҫ��������v�ǽڵ㣬�����Ȱ���ʼ����뵽T������
    T.v(count).y = x_new(1,2); 
    T.v(count).xPrev = x_near(1,1);     % ��ʼ�ڵ�ĸ��ڵ���Ȼ���䱾��
    T.v(count).yPrev = x_near(1,2);
    T.v(count).dist=Delta;          %�Ӹ��ڵ㵽�ýڵ�ľ��룬�����ȡŷ�Ͼ���
    T.v(count).indPrev =min_index;     
    
    

    
    %Step 5:����Ƿ񵽴�Ŀ��㸽�� 
    %��ʾ��ע��ʹ��Ŀ�����ֵThr������ǰ�ڵ���յ��ŷʽ����С��Thr����������ǰforѭ��
    if(distance(x_new(1,1),x_new(1,2),x_G,y_G)<Thr)
%         G=[];
%         G(1,1)=x_G;G(1,y)=y_G;
        plot([x_near(1,1),x_new(1,1)],[x_near(1,2),x_new(1,2)],'g', 'Linewidth', 1.5);
        hold on;
        break;
    end
    plot([x_near(1,1),x_new(1,1)],[x_near(1,2),x_new(1,2)],'g', 'Linewidth', 1.5);
    hold on;
    
   %Step 6:��x_near��x_new֮���·��������
   %��ʾ 1��ʹ��plot���ƣ���ΪҪ�����ͬһ��ͼ�ϻ����߶Σ�����ÿ��ʹ��plot����Ҫ����hold on����
   %��ʾ 2�����ж��յ���������forѭ��ǰ���ǵð�x_near��x_new֮���·��������
   
   pause(0.01); %��ͣ0.1s��ʹ��RRT��չ�������׹۲�
end
%% ·���Ѿ��ҵ��������ѯ
if iter < 2000
    path.pos(1).x = x_G; path.pos(1).y = y_G;
    path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
    pathIndex = T.v(end).indPrev; % �յ����·��
    j=0;
    while 1%ǰ��pos�Ѿ���������2
        path.pos(j+3).x = T.v(pathIndex).x;
        path.pos(j+3).y = T.v(pathIndex).y;
        pathIndex = T.v(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end  % ���յ���ݵ����
    path.pos(end+1).x = x_I; path.pos(end).y = y_I; % ������·��
    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
end


