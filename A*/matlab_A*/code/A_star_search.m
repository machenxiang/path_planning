function path = A_star_search(map,MAX_X,MAX_Y)
%%
%This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset
    size_map = size(map,1);%return the number of row
    Y_offset = 0;
    X_offset = 0;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y));
    %the index of matlab is begin at 1
    %Initialize MAP with location of the target
    %map��һ�д��xֵ���ڶ��д��yֵ
    xval=floor(map(size_map, 1)) + X_offset;%floor get the nearest small integer
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;
    MAP(xval,yval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1)) + X_offset;
    yval=floor(map(1, 2)) + Y_offset;
    xStart=xval;
    yStart=yval;
    MAP(xval,yval)=1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    OPEN=[];
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=[];

    %Put all obstacles on the Closed list
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i;
                CLOSED(k,2)=j;
                k=k+1;
            end
        end
    end
    CLOSED_COUNT=size(CLOSED,1);%closedSet������ΪCLOSED��������
    %set the starting node as the first node
    xNode=xval;
    yNode=yval;
    OPEN_COUNT=1;
    goal_distance=distance(xNode,yNode,xTarget,yTarget);%����ʹ�õ���ŷʽ����,hֵ
    path_cost=0;%gֵ
    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance);
    %OPEN(OPEN_COUNT,1)=0;%��openɾ��
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=xNode;
    CLOSED(CLOSED_COUNT,2)=yNode;
    NoPath=1;

%%
%This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%aaa=node_index(OPEN,xNode,yNode);
    while(~isEmpty(OPEN,OPEN_COUNT)) %you have to dicide the Conditions for while loop exit 
        min_index=min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
        currentX=OPEN(min_index,2);
        currentY=OPEN(min_index,3);
        OPEN(min_index,1)=0;%ɾ���ڵ�
        if(currentX==xTarget&&currentY==yTarget)%��ǰ�ڵ�ΪĿ����ֹѭ��
            goal_index=node_index(OPEN,currentX,currentY);
            aaa=9999;
            break
        end
        for i=-1:1:1
            for j=-1:1:1
                next_x=currentX+i;next_y=currentY+j;
                if(next_x<1||next_y<1||(next_x==currentX&&next_y==currentY)||next_x>10||next_y>10)%Խ������ϰ�������
                    continue;
                elseif(MAP(next_x,next_y)==-1)%�����ϰ�Ҳ����
                    continue;
                elseif(is_inOpen(OPEN,OPEN_COUNT,next_x,next_y)==2)%��close������
                    continue;
                elseif(is_inOpen(OPEN,OPEN_COUNT,next_x,next_y)==1)%��openlist��
                    if(OPEN(node_index(OPEN,next_x,next_y),7)>(OPEN(node_index(OPEN,currentX,currentY),7)+distance(currentX,currentY,next_x,next_y)))
                        OPEN(node_index(OPEN,next_x,next_y),4)=currentX;%���ָ���·���޸ĸ��ڵ�
                        OPEN(node_index(OPEN,next_x,next_y),5)=currentY;
                        OPEN(node_index(OPEN,next_x,next_y),7)=(OPEN(node_index(OPEN,currentX,currentY),7))+distance(currentX,currentY,next_x,next_y);
                        OPEN(node_index(OPEN,next_x,next_y),8)=(OPEN(node_index(OPEN,next_x,next_y),7))+OPEN(node_index(OPEN,next_x,next_y),6);
                    else
                            continue
                    end
                  
                else%����open������ӽ�open��
                    OPEN_COUNT=OPEN_COUNT+1;
                    g=OPEN(node_index(OPEN,currentX,currentY),7)+distance(currentX,currentY,next_x,next_y);
                    h=hValue(next_x,next_y,xTarget,yTarget);
                    f=g+h;
                    OPEN(OPEN_COUNT,:)=insert_open(next_x,next_y,currentX,currentY,h,g,f);
                end
            end
        end
        
                
                
        
     %
     %finish the while loop
     %
     
    end %End of While Loop
    
    %Once algorithm has run The optimal path is generated by starting of at the
    %last node(if it is the target node) and then identifying its parent noade
    %until it reaches the start node.This is the optimal path
    
    %
    %How to get the optimal path after A_star search?
    %please finish it
    %
   path = [];
   temp_x=OPEN(goal_index,2);
   temp_y=OPEN(goal_index,3);
   temp_index=node_index(OPEN,temp_x,temp_y);
   k=1;
    while(temp_x~=xStart&&temp_y~=yStart)
        temp_index=node_index(OPEN,temp_x,temp_y);
        path(k,1)=OPEN(temp_index,4);
        path(k,2)=OPEN(temp_index,5);
        k=k+1;
        temp_x=OPEN(temp_index,4);
        temp_y=OPEN(temp_index,5);
    end
end%���end�Ǹ�function��
