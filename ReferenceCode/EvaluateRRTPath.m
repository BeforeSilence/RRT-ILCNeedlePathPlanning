function evres = EvaluateRRTPath(Goal,Obstacle,initialPoint,spinning_z,L,arc_success,flag)
% 轨迹总共条数：
n = size(L,2);
% 中靶精度初始化
reachGoalAccuracy = [];
% 避障能力初始化
abiliAvoidObstacle = [];
numOfObstacle = size(Obstacle,1);
% 旋转次数初始化
numOfSpinning = [];
% 综合评分
totalEvaluate = [];
kg = 1;
ko = -0.3; %避障系数 因为避障距离越大越好，因此系数为负数
ksp  = 1; % 旋转次数系数

%%对以上指标进行计算：
for i = 1:n
    reachGoalAccuracy = [reachGoalAccuracy; norm(arc_success{i}(end,:) - Goal(1:3))];
    numOfSpinning = [numOfSpinning;size(L{i},1)];
    for j = 1:numOfObstacle
        minLength(j) = minLengthPoint2Line(Obstacle(j,:),arc_success{i});%始终为正
    end
    abiliAvoidObstacle = [abiliAvoidObstacle;min(minLength)];%这里得到的是所有障碍物点到轨迹最近的距离，注意这个值越大越好
    totalScore = kg*reachGoalAccuracy(i) + ko * abiliAvoidObstacle(i) + ksp * numOfSpinning(i);
    totalEvaluate = [totalEvaluate;totalScore];
end
    
switch flag
    case 1
        Index = find(reachGoalAccuracy == min(reachGoalAccuracy));
        evres = Index(1);
        sprintf('中靶精度最高的轨迹为第%d条',Index(1))
    case 2
        Index = find(numOfSpinning == min(numOfSpinning));
        evres = Index(1);
        sprintf('旋转次数最少的轨迹为第%d条',Index(1))
    case 3
        Index = find(abiliAvoidObstacle == max(abiliAvoidObstacle));
        evres = Index(1);
        sprintf('避障能力最强的轨迹为第%d条',Index(1))
    case 4
        Index = find(totalEvaluate == min(totalEvaluate));
        evres = Index(1);
        sprintf('综合水平第一的轨迹为第%d条',Index(1))
end
        
end

%%子函数： 障碍物点到轨迹的距离计算：
function minLength = minLengthPoint2Line(Obstacle,arc_success)
l = size(arc_success,1);
for i = 1:l
    distancePoint2Line(i) = norm(arc_success(i,:)-Obstacle(1:3));
end
minLength = min(distancePoint2Line);
end