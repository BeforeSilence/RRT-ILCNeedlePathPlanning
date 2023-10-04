function [initialPoint,spinning_z,L,arc_success,world] = RRT_Tree(Goal,Obstacle,R)

% Max Iteration Times
maxIteration = 10000;

% Boundary and Target
world.boundary = [0 0 -100;50 50 100];

world.obstacles=Obstacle;
       
world.goal=Goal;    

[tree,arc_good,g] = initialTree();

%Initial flag
reachTarget = 0;
%Initial Iteration 
Iteration = 0;

% Path Counter
count = 0;

while(count < 5 &&Iteration<=maxIteration)
    % Random Point
    pRand=randPoint(world);
    % Find Root Point
    [idx,qNearest]=nearestNeighber(tree,pRand);
    % APF Force
    pAdjust =adjustByPotentialForce(qNearest,pRand,world);
    %��֪�����͸��ڵ�����߼����ӦԲ�ĽǺ�Բ��
    [pNew,arc_glo,g,tree]=inverseCircle(idx,qNearest,pAdjust,R,g,tree);  
    % Boundary Check
    flagBoun = If_boundary(arc_glo,world);
    % Collision Check
    flagColl = If_collision(arc_glo,world);
    % Delete New Point
    if flagBoun == 1 ||  flagColl == 1 || size(arc_glo,1) < 10 %��ӻ�����С�ж�˵������Ч·��
        g{size(tree,1)} = [];
        tree(end,:) = []; %Delete
        %g{end} = [];
        continue;
    else
        arc_good{size(tree,1)-1} =arc_glo; %Success Arc
    end
    %ͨ���жϣ���һ���жϸýڵ��Ƿ����һ������е�,����˼������goal��Բ�Ĵ�����һ����pRand���ж������ɵĵ���world.goal֮��ľ���
    [pNew,arc_glo,g,tree]=inverseCircle(idx,qNearest,world.goal(1:3),R,g,tree);
    if norm(pNew(1:3) - world.goal(1:3)) < world.goal(4)
        %��Ե��⣺������ɵĻ�����û�г����߽�
        flagBoun = If_boundary(arc_glo,world);
        %��ײ���
        flagColl = If_collision(arc_glo,world);
        %���ж��Ƿ����㲻�����߽缰��ײ��⣺
        if flagBoun == 0 &&  flagColl == 0 && size(arc_glo,1) > 10  %��������Ϊ0
            arc_good{size(tree,1)-1} =arc_glo; %��¼�ɹ���arc
            count = count + 1; %�ɹ��켣�� +1
            fprintf('�ѳɹ��滮����%d���켣��\n',count);
            %��¼�ɹ���arc���ɵĹ켣��
            [arc_success{count},spinning_z{count},L{count},initialPoint{count}] = findSuccessArc(arc_good,tree,R);
            %��¼��Ӧ����ز�����
            %reachTarget = 1;
            %Ϊ�˱��������������ظ��켣��ÿ������һ���ɹ��켣�󣬸��������³�ʼ��
            [tree,arc_good,g] = initialTree();
        end

    else
         tree(end,:) = []; %ɾ���õ�
         g{end} = [];
    end
    
    Iteration = Iteration + 1; %����ͼ�������
end

%���Ƴɹ��켣��
figure()
for i = 1:count
    plot3(arc_success{i}(:,1),arc_success{i}(:,2),arc_success{i}(:,3));hold on;
    axis equal
end
plotObstacle(world); hold on;
plotTarget(world); hold on;

end

%�Ӻ��������������
function pRand=randPoint(world)
x=world.boundary(1,1)+world.boundary(2,1)*rand();
y=world.boundary(1,2)+world.boundary(2,2)*rand();
z=world.boundary(1,3)+world.boundary(2,3)*rand();
pRand=[x,y,z];
end

%�Ӻ�����Ѱ�������������������ڵ���Ϊ���ڵ�
function [idx,qNearest]=nearestNeighber(tree,p)  % Ѱ��������P����ĵ�
tree1=tree(:,1:3);  % ֻ�����������Ϊ��Ѱ����������ĵ�
tree2=zeros(size(tree1,1),3); % ͬΪ������������ĵ�
for i=1:size(tree1,1)
    tree2(i,:)=p; % ÿ����Ϊ�����
end
 Lv=tree2-tree1;
 length=Lv.*Lv;
 length=sum(length,2);
 [lmax,idx]=min(length);
 qNearest=tree(idx,:);
end

%�Ӻ�������֪�����͸��ڵ�����߼����ӦԲ�ĽǺ�Բ��
function [pNew,arc_glo,g,tree]=inverseCircle(idx,qNearest,pRand,R,g,tree)
%g��qNearest��Ӧ��g
g_f = g{idx};
angel_zf = qNearest(4);%���ڵ���z����ת�Ƕ�
trans_f = qNearest(1:3); %���ڵ�����
L_vect = pRand - qNearest(1:3); %����������ɵ�����
L =  norm(L_vect); %�ҳ�
y_vect = [0,1,0];
angel_y = 2*asin(L/(2*R)); %��һ����y����ת�ǣ���Բ��Բ�Ľǣ������ֵ
arc_local = localcircle(angel_y,R); %�ֲ�Բ������
o_local = [R,0,0]; %�ֲ�����ϵ��Բ��
%angel_z = pi/2 - acos(dot(y_vect,L_vect)/(norm(y_vect)*norm(L_vect))); %��һ����z����ת�ǣ�����ֵ
angel_z = pi/2 - sign(L_vect(1))*acos(dot(y_vect,L_vect)/(norm(y_vect)*norm(L_vect))); %����x�����ſ��ǣ���ת��ΧΪ��0��2pi��
if idx == 1 %û�и��ڵ�g�����
    spinningMatrix_y = [1 0 0;0 1 0;0 0 1];
    spinningMatrix_z = [cos(angel_z) -sin(angel_z) 0;sin(angel_z) cos(angel_z) 0; 0 0 1];
    g_f = spinningMatrix_z * g_f;
else
    angel_z = angel_z - angel_zf; %ת��Ϊ���ֵ
    spinningMatrix_z = [cos(angel_z) -sin(angel_z) 0;sin(angel_z) cos(angel_z) 0; 0 0 1];
    g_f = spinningMatrix_z * g_f;
end
arc_glo = arc_local * g_f + trans_f;
o_glo = o_local * g_f + trans_f;
pNew = [arc_glo(end,:),angel_y,angel_z,idx]; %�µĽڵ�λ��
tree = [tree;pNew];%����tree
%�����ӽڵ��g_c �� 
spinningMatrix_y = [cos(angel_y) 0 sin(angel_y);0 1 0;-sin(angel_y) 0 cos(angel_y)];%��һ����y����ת����
g_c = spinningMatrix_y * g_f;
g{size(tree,1)} = g_c; %����tree��ÿ���ڵ��Ӧ������ϵ
end

%�Ӻ������ֲ�Բ������
function  A = localcircle(alpha,R)
th = pi:0.001:pi+alpha;
x1 = R*cos(th) + R;
z1 = R*sin(th);
y1 = zeros(size(x1));
A = [x1' y1' z1'];
end

%�Ӻ�������Ե��⣺������ɵĻ�����û�г����߽�
function flagBoun = If_boundary(arc_glo,world)
n = size(arc_glo,1);
flagBoun = 0; %�ȼ���û�г����߽�
for i = 1:n
    if arc_glo(i,1)< world.boundary(1,1) ||  arc_glo(i,1) > world.boundary(1,1) + world.boundary(2,1)
        flagBoun = 1;
        return
    end
    if arc_glo(i,2)< world.boundary(1,2) ||  arc_glo(i,2) > world.boundary(1,2) + world.boundary(2,2)
        flagBoun = 1;
        return
    end
    if arc_glo(i,3)< world.boundary(1,3) ||  arc_glo(i,3) > world.boundary(1,3) + world.boundary(2,3)+0.01
        flagBoun = 1;
        return
    end   
end
end

%�Ӻ�������ײ��⣺
function flagColl =If_collision(arc_glo,world)
n = size(arc_glo,1);
numObs = size(world.obstacles,1);
flagColl = 0; %�ȼ���û�������ϰ���
for i = 1:n
    for j =1:numObs
        if norm(arc_glo(i,:) - world.obstacles(j,1:3)) < world.obstacles(j,4)
            flagColl = 1;
            return
        end
    end
end
end

%�Ӻ����������ϰ����
function plotObstacle(world)

numObs = size(world.obstacles,1);
for i = 1:numObs
    [x,y,z]=sphere(50);
    x=world.obstacles(i,1)+world.obstacles(i,4)*x;           
    y=world.obstacles(i,2)+world.obstacles(i,4)*y;
    z=world.obstacles(i,3)+world.obstacles(i,4)*z;
    surf(x,y,z,'FaceColor','red')
    shading flat 
end
end

%�Ӻ��������ưе�
function plotTarget(world)
    [x,y,z]=sphere(50);
    x=world.goal(1)+world.goal(4)*x;           
    y=world.goal(2)+world.goal(4)*y;
    z=world.goal(3)+world.goal(4)*z;
    surf(x,y,z,'FaceColor',[1 0 0])   
    shading flat 
end
    
%�Ӻ���������APF����
function pAdjust =adjustByPotentialForce(p,pRand,world)
obstacles=world.obstacles;
goal=world.goal;
kp=1;
kr=1;
m0=5;

Fp=goal(1,1:3)-p(1,1:3);  % ����
Fp=kp*Fp; %ϵ����������

Fr=zeros(size(Fp));   % ��ʼ������Ϊ0
for i=1:size(obstacles,1)
    m=norm(obstacles(i,1:3)-p(1,1:3));
    if(m>m0+obstacles(i,4))
        Frtemp=zeros(size(Fp));
    else
        Frnorm=kr*(1/m-1/m0)/(m^2);  % �����ľ���ֵ
        Frtemp=(obstacles(i,1:3)-p(1,1:3))*Frnorm/m;
    end
    Fr=Fr+Frtemp;
end
f=Fr+Fp; %��������
direction0=pRand(1,1:3)-p(1,1:3); % ��ʼ���������ֱ��ǰ���ķ���
direction=f + direction0;   % ���������������ߵķ���
direction=direction/norm(direction)*norm(direction0);  % ǰ���Ĳ�����Ϊ֮ǰ����
pAdjust = p(1,1:3)+direction;  % �µ��λ��
end

%�Ӻ�����Ѱ��һ���ɹ���arc�켣
%����ֱ�Ϊ�켣·����������ת���Ʋ������봩����ȿ��Ʋ���
function [arc_success,spinning_z,L,initialPoint] = findSuccessArc(arc_good,tree,R) 
t = size(tree,1) ;  %���һ���ڵ��ǳɹ��Ľڵ�
idx_s = []; %��¼�ɹ��ı��
while t > 81 % ˵���Ѿ�Ѱ���˸��ڵ�
    idx_s = [idx_s;t];
    t = tree(t,6);
end
initialPoint = tree(t,1:3);
idx_s = [idx_s; t]; %�ѵ�һ��ֵ�ӽ�ȥ
idx_sr = idx_s - 1; %ת����acr��Ӧ�ľ�����
arc_success = [];
spinning_z = [];
L =[];
for i = length(idx_sr):-1:1
    arc_success = [arc_success;arc_good{idx_sr(i)}]; %idx_sr(1)<81 �� arc_good{idx_sr(1)Ϊ�� ���ǲ�Ӱ��
end

for j = length(idx_s):-1:1
    spinning_z = [spinning_z;tree(idx_s(j),5)];
    L = [L;tree(idx_s(j),4)*R]; %�������� Բ�Ľ� * R  ע���ǻ�����
end
end

%�Ӻ�������ʼ��tree
function [tree,arc_good,g] = initialTree()
%��ʼ���������ø��ڵ����Ϊ9*9=49����,��ʵ�����
a = 5:5:45;
b = 1:1:9;
for k = 1: length(a)
    tree(k*length(a)-8:k*length(a),1) = a;
    tree(k*length(a)-8:k*length(a),2) = a(k);
    tree(k*length(a)-8:k*length(a),[3:5]) = 0;
    tree(k*length(a)-8:k*length(a),6) =k * b;
end
%tree = [25,25,0,0,0,1]; %��ʼ���ڵ�,�ֱ�Ϊx,y,z,angle_y,angle_z,���һλ��ʾ�����ĸ�����
for p = 1:81 %��ʼ��ÿ�����ڵ��Ӧ��ת������
    g{p} = [1,0,0; 0,1,0;0,0,1]; %��ʼת������
end
arc_good = [];
end