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
    %已知随机点和根节点的连线计算对应圆心角和圆心
    [pNew,arc_glo,g,tree]=inverseCircle(idx,qNearest,pAdjust,R,g,tree);  
    % Boundary Check
    flagBoun = If_boundary(arc_glo,world);
    % Collision Check
    flagColl = If_collision(arc_glo,world);
    % Delete New Point
    if flagBoun == 1 ||  flagColl == 1 || size(arc_glo,1) < 10 %添加弧长过小判断说明是无效路径
        g{size(tree,1)} = [];
        tree(end,:) = []; %Delete
        %g{end} = [];
        continue;
    else
        arc_good{size(tree,1)-1} =arc_glo; %Success Arc
    end
    %通过判断，下一步判断该节点是否可以一步到达靶点,核心思想是用goal的圆心代替上一步的pRand，判断新生成的点与world.goal之间的距离
    [pNew,arc_glo,g,tree]=inverseCircle(idx,qNearest,world.goal(1:3),R,g,tree);
    if norm(pNew(1:3) - world.goal(1:3)) < world.goal(4)
        %边缘检测：检测生成的弧线有没有超出边界
        flagBoun = If_boundary(arc_glo,world);
        %碰撞检测
        flagColl = If_collision(arc_glo,world);
        %再判断是否满足不超过边界及碰撞检测：
        if flagBoun == 0 &&  flagColl == 0 && size(arc_glo,1) > 10  %弧长不能为0
            arc_good{size(tree,1)-1} =arc_glo; %记录成功的arc
            count = count + 1; %成功轨迹数 +1
            fprintf('已成功规划出第%d条轨迹！\n',count);
            %记录成功的arc构成的轨迹：
            [arc_success{count},spinning_z{count},L{count},initialPoint{count}] = findSuccessArc(arc_good,tree,R);
            %记录相应的针控参数：
            %reachTarget = 1;
            %为了避免总是生成是重复轨迹，每次生成一条成功轨迹后，各变量重新初始化
            [tree,arc_good,g] = initialTree();
        end

    else
         tree(end,:) = []; %删除该点
         g{end} = [];
    end
    
    Iteration = Iteration + 1; %否则就继续迭代
end

%绘制成功轨迹：
figure()
for i = 1:count
    plot3(arc_success{i}(:,1),arc_success{i}(:,2),arc_success{i}(:,3));hold on;
    axis equal
end
plotObstacle(world); hold on;
plotTarget(world); hold on;

end

%子函数：生成随机点
function pRand=randPoint(world)
x=world.boundary(1,1)+world.boundary(2,1)*rand();
y=world.boundary(1,2)+world.boundary(2,2)*rand();
z=world.boundary(1,3)+world.boundary(2,3)*rand();
pRand=[x,y,z];
end

%子函数：寻找随机点最近的已有树节点作为根节点
function [idx,qNearest]=nearestNeighber(tree,p)  % 寻找树上离P最近的点
tree1=tree(:,1:3);  % 只有坐标的树，为了寻找树上最近的点
tree2=zeros(size(tree1,1),3); % 同为向量，找最近的点
for i=1:size(tree1,1)
    tree2(i,:)=p; % 每各都为随机点
end
 Lv=tree2-tree1;
 length=Lv.*Lv;
 length=sum(length,2);
 [lmax,idx]=min(length);
 qNearest=tree(idx,:);
end

%子函数：已知随机点和根节点的连线计算对应圆心角和圆心
function [pNew,arc_glo,g,tree]=inverseCircle(idx,qNearest,pRand,R,g,tree)
%g是qNearest对应的g
g_f = g{idx};
angel_zf = qNearest(4);%根节点绕z轴旋转角度
trans_f = qNearest(1:3); %根节点坐标
L_vect = pRand - qNearest(1:3); %二者连线组成的向量
L =  norm(L_vect); %弦长
y_vect = [0,1,0];
angel_y = 2*asin(L/(2*R)); %下一步的y轴旋转角，即圆弧圆心角，是相对值
arc_local = localcircle(angel_y,R); %局部圆弧生成
o_local = [R,0,0]; %局部坐标系的圆心
%angel_z = pi/2 - acos(dot(y_vect,L_vect)/(norm(y_vect)*norm(L_vect))); %这一步的z轴旋转角，绝对值
angel_z = pi/2 - sign(L_vect(1))*acos(dot(y_vect,L_vect)/(norm(y_vect)*norm(L_vect))); %加入x正负号考虑，旋转范围为（0，2pi）
if idx == 1 %没有根节点g的情况
    spinningMatrix_y = [1 0 0;0 1 0;0 0 1];
    spinningMatrix_z = [cos(angel_z) -sin(angel_z) 0;sin(angel_z) cos(angel_z) 0; 0 0 1];
    g_f = spinningMatrix_z * g_f;
else
    angel_z = angel_z - angel_zf; %转换为相对值
    spinningMatrix_z = [cos(angel_z) -sin(angel_z) 0;sin(angel_z) cos(angel_z) 0; 0 0 1];
    g_f = spinningMatrix_z * g_f;
end
arc_glo = arc_local * g_f + trans_f;
o_glo = o_local * g_f + trans_f;
pNew = [arc_glo(end,:),angel_y,angel_z,idx]; %新的节点位置
tree = [tree;pNew];%更新tree
%生成子节点的g_c 和 
spinningMatrix_y = [cos(angel_y) 0 sin(angel_y);0 1 0;-sin(angel_y) 0 cos(angel_y)];%下一步的y轴旋转矩阵
g_c = spinningMatrix_y * g_f;
g{size(tree,1)} = g_c; %更新tree上每个节点对应的坐标系
end

%子函数：局部圆弧生成
function  A = localcircle(alpha,R)
th = pi:0.001:pi+alpha;
x1 = R*cos(th) + R;
z1 = R*sin(th);
y1 = zeros(size(x1));
A = [x1' y1' z1'];
end

%子函数：边缘检测：检测生成的弧线有没有超出边界
function flagBoun = If_boundary(arc_glo,world)
n = size(arc_glo,1);
flagBoun = 0; %先假设没有超出边界
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

%子函数：碰撞检测：
function flagColl =If_collision(arc_glo,world)
n = size(arc_glo,1);
numObs = size(world.obstacles,1);
flagColl = 0; %先假设没有碰到障碍物
for i = 1:n
    for j =1:numObs
        if norm(arc_glo(i,:) - world.obstacles(j,1:3)) < world.obstacles(j,4)
            flagColl = 1;
            return
        end
    end
end
end

%子函数：绘制障碍物点
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

%子函数：绘制靶点
function plotTarget(world)
    [x,y,z]=sphere(50);
    x=world.goal(1)+world.goal(4)*x;           
    y=world.goal(2)+world.goal(4)*y;
    z=world.goal(3)+world.goal(4)*z;
    surf(x,y,z,'FaceColor',[1 0 0])   
    shading flat 
end
    
%子函数：计算APF合力
function pAdjust =adjustByPotentialForce(p,pRand,world)
obstacles=world.obstacles;
goal=world.goal;
kp=1;
kr=1;
m0=5;

Fp=goal(1,1:3)-p(1,1:3);  % 引力
Fp=kp*Fp; %系数乘以引力

Fr=zeros(size(Fp));   % 初始化斥力为0
for i=1:size(obstacles,1)
    m=norm(obstacles(i,1:3)-p(1,1:3));
    if(m>m0+obstacles(i,4))
        Frtemp=zeros(size(Fp));
    else
        Frnorm=kr*(1/m-1/m0)/(m^2);  % 斥力的绝对值
        Frtemp=(obstacles(i,1:3)-p(1,1:3))*Frnorm/m;
    end
    Fr=Fr+Frtemp;
end
f=Fr+Fp; %合力方向
direction0=pRand(1,1:3)-p(1,1:3); % 初始按照随机点直接前进的方向
direction=f + direction0;   % 合力和两个点连线的方向
direction=direction/norm(direction)*norm(direction0);  % 前进的步长设为之前步长
pAdjust = p(1,1:3)+direction;  % 新点的位置
end

%子函数：寻找一条成功的arc轨迹
%输出分别为轨迹路径集，针旋转控制参数，针穿刺深度控制参数
function [arc_success,spinning_z,L,initialPoint] = findSuccessArc(arc_good,tree,R) 
t = size(tree,1) ;  %最后一个节点是成功的节点
idx_s = []; %记录成功的编号
while t > 81 % 说明已经寻到了根节点
    idx_s = [idx_s;t];
    t = tree(t,6);
end
initialPoint = tree(t,1:3);
idx_s = [idx_s; t]; %把第一个值加进去
idx_sr = idx_s - 1; %转换到acr对应的矩阵编号
arc_success = [];
spinning_z = [];
L =[];
for i = length(idx_sr):-1:1
    arc_success = [arc_success;arc_good{idx_sr(i)}]; %idx_sr(1)<81 则 arc_good{idx_sr(1)为空 但是不影响
end

for j = length(idx_s):-1:1
    spinning_z = [spinning_z;tree(idx_s(j),5)];
    L = [L;tree(idx_s(j),4)*R]; %弧长等于 圆心角 * R  注意是弧度制
end
end

%子函数：初始化tree
function [tree,arc_good,g] = initialTree()
%初始化树，设置根节点个数为9*9=49个点,与实验对上
a = 5:5:45;
b = 1:1:9;
for k = 1: length(a)
    tree(k*length(a)-8:k*length(a),1) = a;
    tree(k*length(a)-8:k*length(a),2) = a(k);
    tree(k*length(a)-8:k*length(a),[3:5]) = 0;
    tree(k*length(a)-8:k*length(a),6) =k * b;
end
%tree = [25,25,0,0,0,1]; %起始树节点,分别为x,y,z,angle_y,angle_z,最后一位表示加在哪个点上
for p = 1:81 %初始化每个树节点对应的转换矩阵
    g{p} = [1,0,0; 0,1,0;0,0,1]; %初始转换矩阵
end
arc_good = [];
end