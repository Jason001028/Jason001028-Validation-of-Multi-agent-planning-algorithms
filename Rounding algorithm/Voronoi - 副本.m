
clear all
close all

pursuers_num=3;
evaders_num=6;
agents_sum=pursuers_num+evaders_num;
t_step=0.01;
capture_dis=0.03;
counter=0;
L=1; %�߳�Ϊ1��������
square_x=[0 0 L L 0]; % 5�����������,��һ�������һ���غϲ����γɷ��ͼ��
square_y=[0 L L 0 0];

%%��ʼ�����������
for i=1:agents_sum
    agents(i).pos=rand(1,2);
    agents(i).active=1; % evader ���flag
    agents(i).min_dis=100;
    agents(i).distance = 0;
end
figure()
while 1
    if sum([agents(pursuers_num+1:agents_sum).active])==0 % ���е�evader����ץ����
        % �����Ƶ
        v = VideoWriter('Hunt based on voronoi.avi');
        v.FrameRate=4;
        open(v);
        writeVideo(v, F);
        close(v);
    % ����׷���ߵľ����ܺ�
    total_distance_pursuers = sum([agents(1:pursuers_num).distance]);
    distances=zeros(pursuers_num);
    num_dis=1;
    % ������״ͼ
    for i=1:agents_sum
        if agents(i).active==1
            distances(num_dis)=agents(i).distance;
            num_dis=num_dis+1;
        end
    end
    figure;
    
    %���������ɫ,����һ�� pursuers_num x 3 �ľ�������ÿ�д���һ�������ɫ��RGBֵ����
    colors = rand(pursuers_num, 3);
    for i=1:pursuers_num
        bar(distances(i),colors(i,:),BarWidth=1.2);
    end
    ylabel('Total Distance Traveled');
    xlabel('Agent Index');
    title('Distance Traveled by Each Agent');
    % ��ʾ׷���ߵľ����ܺ�
    disp(['Total distance traveled by all pursuers: ', num2str(total_distance_pursuers)]);
        return;

    else
        temp_pos=[];

        % ����active��agents��voronoiͼ
        index_active = find([agents(:).active]);
        temp_pos = [agents(index_active).pos];
        temp_pos = reshape(temp_pos, 2, length(index_active))'; % ��Ҫת��
        
        % ���� Voronoi ͼ
        [vx, vy] = voronoi(temp_pos(:,1), temp_pos(:,2));
        [V, C] = voronoin(temp_pos);
        
        

        % ��pursuer��evader��λ��
        plot(temp_pos(1:pursuers_num,1), temp_pos(1:pursuers_num,2), 'go', ...
             temp_pos(pursuers_num+1:end,1), temp_pos(pursuers_num+1:end,2), 'r*', ...
             vx, vy, 'k-');

        
        % ��lable
        plabels = arrayfun(@(n) {sprintf('X%d', n)}, (1:length(index_active))');
        hold on
        Hpl = text(temp_pos(:,1), temp_pos(:,2), plabels, ...
              'HorizontalAlignment','left', ...
              'BackgroundColor', 'none');
        xlim([0 1]);
        ylim([0 1]);
        

        % �������������ڵ�Ԫ�����㣬����up�Ĺؼ���Ԥ�����������е�Ԫ������Ԫ��C����������Ķ��������Զ�Ķ���ID��ɾ������ֻ�������������ڵĶ���
        for i=1:length(C)
            out_index=[];
            inf_index=[];
            if sum(C{i}==1) % �ҵ�Ԫ��������Զ�ĵ������
                inf_index=find(C{i}==1);
            end
            for j=1:length(C{i}) % �ҵ�Ԫ������������ĵ������
                [in on]= inpolygon(V(C{i}(j),1),V(C{i}(j),2),square_x,square_y);
                if in==0 && on==0 % ���ⲿ
                    out_index=[out_index j];
                end
            end
            out_index=[inf_index out_index]; % �������Զ����ⲿ�������������һ������������Ⱥ�˳��ᵼ��ɾ����Ԫ��������Ŀ�����仯
            C{i}(out_index)=[]; % �������V�Ķ��㣬����C{i}�ж�����޷�����
        end
        
        %��vx��vy�е�����ҵ㣬�ڶ������һ�����յ㣬��һ�����һ������㣬����������߶��Ƿ��������ε��߶��н���
        cross_point=[];
        for i=1:length(vx)
            [xi,yi] = polyxpoly(vx(:,i),vy(:,i),square_x,square_y,'unique');
            cross_point=[cross_point;[xi yi]];
        end
        
        % �ѽ���������εĶ��㰴˳�򴢴浽V�У��������V�Ķ��㣬����C{i}�ж�����޷�������������ID
        old_V=V;
        V=[V;cross_point];
        
        % V������ӵĵ������������Ԫ������С�ڵ��ڣ�������ΪԪ������ID
        C_pos=temp_pos; % ����Ԫ����λ��
        for i=length(old_V)+1:length(V)
            V_dist=sum(abs(V(i,:)-C_pos).^2,2).^(1/2);
            V_index=find(V_dist==min(V_dist)); % ���������Ԫ��
            if length(V_index)==2
                C{V_index(1)}=[C{V_index(1)} i];
                C{V_index(2)}=[C{V_index(2)} i];
            else
                C{V_index}=[C{V_index} i]; % ���������뵽Ԫ���Ķ����б���
                V_dist(V_index)=max(V_dist); 
                V_index=find(V_dist==min(V_dist)); % �����ڶ�С��index��ʵ����������ͬ��
                C{V_index}=[C{V_index} i];
            end
        end
        
        % �������εĽǵ���뵽V�У���ӵ������Ԫ����һ�㶥��ֻ��һ��ռ�У��������Ҳ����������ռ��
        square=[square_x;square_y]';
        square(end,:)=[]; % ȥ���������ظ��Ľǵ�
        old_V=V;
        V=[V; square];
        for i=length(old_V)+1:length(V)
            V_dist=sum(abs(V(i,:)-C_pos).^2,2).^(1/2);
            V_index=find(V_dist==min(V_dist)); % ���������Ԫ��
            if length(V_index)==2
                C{V_index(1)}=[C{V_index(1)} i];
                C{V_index(2)}=[C{V_index(2)} i];
            else
                C{V_index}=[C{V_index} i]; % ���������뵽Ԫ���Ķ����б���
            end
        end
        
        % ����Ԫ����neighbor�����ģ�
        common_index=[];
        common_id=[];
        for i=1:length(C) % ������agents_sum��������������evaderҲ�����ȥ
            for j=1:length(C)
                if i~=j
                    temp_common=intersect(C{i},C{j});
                    if length(temp_common)==2
                        common_index=[common_index;temp_common];
                        common_id=[common_id;j];
                    end
                end
            end
            if length(common_id)~=0 % �������ĳ����common_idΪ�յ��³���������
                agents(i).neightbor.id=common_id;
                agents(i).neightbor.shared_boundary=common_index;
                agents(i).neightbor.mid_point= (V(common_index(:,1),:) + V(common_index(:,2),:))/2;  
                common_index=[]; %���
                common_id=[];
            end
        end     
        
        % ���ݹ�ʽ����up
        for i=1:pursuers_num
            if sum(agents(i).neightbor.id>pursuers_num) % ���pursuer��Χ��evader�����е�
                near_id=find(agents(i).neightbor.id>pursuers_num); % ��neighbor��evader��id
                temp_err=agents(i).pos-agents(i).neightbor.mid_point(near_id,:);
                temp_dis=sum(abs(temp_err).^2,2).^(1/2);
                min_index = find(temp_dis==min(min(temp_dis))) + sum(agents(i).neightbor.id<=pursuers_num); % ����pursuer��ID,����Ҫ����Ϊtemp_dis��ȥ��ǰ��id
                agents(i).target=agents(i).neightbor.mid_point(min_index,:); % �ҵ������evader
                agents(i).up=(agents(i).target-agents(i).pos)/norm((agents(i).target-agents(i).pos));
            else % ���pursuer��Χû��evader�������evader�߹�ȥ
                for j=(pursuers_num+1):agents_sum
                    if agents(j).active
                        dis=norm(agents(i).pos - agents(j).pos);
                        if dis < agents(i).min_dis
                            agents(i).min_dis=dis; %����ÿ��evader�����е�pursuer����С����
                            agents(i).target=agents(j).pos;
                            agents(i).up=(agents(i).target-agents(i).pos)/norm((agents(i).target-agents(i).pos));
                        end
                    end
                end
            end
        end
        
        % ����ue  ��ʼ��ע�͵���
        % A = polyarea(X,Y)
        for i=(pursuers_num+1):agents_sum
            if agents(j).active
                if agents(i).neightbor.id>pursuers_num
                    near_id=find(agents(i).neightbor.id<pursuers_num);
                    agents(i).target=min(agents(i).neightbor.mid_point(near_id));
                    agents(i).ue=(agents(i).target-agents(i).pos)/norm((agents(i).target-agents(i).pos));
                else
                    for j=1:pursuers_num 
                        dis=norm(agents(i).pos - agents(j).pos); 
                        if dis < agents(i).min_dis
                            agents(i).min_dis=dis; %����ÿ��evader�����е�pursuer����С����
                            agents(i).target=agents(j).pos;
                            agents(i).ue=(agents(i).target-agents(i).pos)/norm((agents(i).target-agents(i).pos));                            
                        end
                    end
                end
            end
        end
        
        %�����ƶ�·��
        for i=1:pursuers_num
            hold on
            plot([agents(i).pos(1,1);agents(i).target(1,1)], [agents(i).pos(1,2);agents(i).target(1,2)],'g-')
        end

        % ֻ����active��pursuersλ��
        for i=1:pursuers_num
            if agents(i).active
                old_pos = agents(i).pos; % �����λ��
                if isempty(agents(i).up)
                    agents(i).up = [0, 0]; % ����Ĭ�Ϸ�������
                end
                new_pos = agents(i).pos + t_step * agents(i).up; % ������λ��
                agents(i).pos = checkBounds(new_pos, L); % Ӧ�ñ߽���
                agents(i).distance = agents(i).distance + norm(agents(i).pos - old_pos); % �ۼ���ʻ����
            end
        end
        
        % ֻ����active��evadersλ��
        for i=(pursuers_num+1):agents_sum
            if agents(i).active
                old_pos = agents(i).pos; % �����λ��
                if isempty(agents(i).ue)
                    agents(i).ue = [0, 0]; % ����Ĭ�Ϸ�������
                end
                new_pos = agents(i).pos + t_step * agents(i).ue; % ������λ��
                agents(i).pos = checkBounds(new_pos, L); % Ӧ�ñ߽���
            end
        end


        
    % % ����ѭ���и�����ʻ���루������
    % for i=1:agents_sum
    %     if agents(i).active
    %         old_pos = agents(i).pos; % �����λ��
    %         % ����λ�ô���
    %         % ... existing code for updating agents(i).pos ...
    %         agents(i).distance = agents(i).distance + norm(agents(i).pos - old_pos); % �ۼ���ʻ����
    %     end
    % end

    % ����ѭ���и���λ��ʱӦ�ñ߽���
    for i=1:pursuers_num
        if agents(i).active
            new_pos = agents(i).pos + t_step * agents(i).up; % ������λ��
            agents(i).pos = checkBounds(new_pos, L); % Ӧ�ñ߽���
        end
    end
    
    for i=(pursuers_num+1):agents_sum
        if agents(i).active
            new_pos = agents(i).pos + t_step * agents(i).ue; % ������λ��
            agents(i).pos = checkBounds(new_pos, L); % Ӧ�ñ߽���
        end
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �ж��Ƿ񴥷�������
for i=(pursuers_num+1):agents_sum
    if agents(i).active
        for j=1:pursuers_num
            dis=norm(agents(i).pos - agents(j).pos); 
            if dis < agents(i).min_dis
                agents(i).min_dis=dis; %����ÿ��evader�����е�pursuer����С����
            end
        end
        % �ж�evader���ɹ�����Ĵ�������
        if agents(i).min_dis < capture_dis
            agents(i).active=0;
            for j=1:pursuers_num
                agents(j).min_dis=100;% evader��������Ҫ������pursuer��min_dis���룬����ᳯ��������evader����ǰ��
            end
            if agents(i).active % ȷ��ֻ�л�Ծ���ӱ��߲ű�����
                hold on
                plot(agents(i).pos(1,1),agents(i).pos(1,2),'g*')
            end
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %     % �ж��Ƿ񴥷�������
    %     for i=(pursuers_num+1):agents_sum
    %         if agents(i).active
    %             for j=1:pursuers_num
    %                 dis=norm(agents(i).pos - agents(j).pos); 
    %                 if dis < agents(i).min_dis
    %                     agents(i).min_dis=dis; %����ÿ��evader�����е�pursuer����С����
    %                 end
    %             end
    %             % �ж�evader���ɹ�����Ĵ�������
    %             if agents(i).min_dis < capture_dis
    %                 agents(i).active=0;
    %                 for j=1:pursuers_num
    %                     agents(j).min_dis=100;% evader��������Ҫ������pursuer��min_dis���룬����ᳯ��������evader����ǰ��
    %                 end
    %                 hold on
    %                 plot(agents(i).pos(1,1),agents(i).pos(1,2),'g*')
    %             end
    %         end
    %     end
    % end
  
    end
    %% ��������

% % �������ӱ��߱�����������״ͼ
% if sum([agents(pursuers_num+1:agents_sum).active]) == 0
%     % ... existing code for video output ...
% 
%     % ����׷���ߵľ����ܺ�
%     total_distance_pursuers = sum([agents(1:pursuers_num).distance]);
% 
%     % ������״ͼ
%     distances = [agents.distance];
%     figure;
%     bar(distances);
%     ylabel('Total Distance Traveled');
%     xlabel('Agent Index');
%     title('Distance Traveled by Each Agent');
% 
%     % ��ʾ׷���ߵľ����ܺ�
%     disp(['Total distance traveled by all pursuers: ', num2str(total_distance_pursuers)]);
% 
%     return; % ��������
% end




    
    %% 
    
    hold off
    counter=counter+1;
    F(counter) = getframe(gcf); %gcf��������ǰ����ı�����ֱ���þ��У�
    pause(0.01)
end



% �߽���͵�������
function pos = checkBounds(pos, L)
    if numel(pos) < 2
        error('Position must be a 2-element vector.');
    end
    pos(1) = max(min(pos(1), L), 0); % ���x����
    pos(2) = max(min(pos(2), L), 0); % ���y����
end

