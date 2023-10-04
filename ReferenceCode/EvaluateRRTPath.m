function evres = EvaluateRRTPath(Goal,Obstacle,initialPoint,spinning_z,L,arc_success,flag)
% �켣�ܹ�������
n = size(L,2);
% �ао��ȳ�ʼ��
reachGoalAccuracy = [];
% ����������ʼ��
abiliAvoidObstacle = [];
numOfObstacle = size(Obstacle,1);
% ��ת������ʼ��
numOfSpinning = [];
% �ۺ�����
totalEvaluate = [];
kg = 1;
ko = -0.3; %����ϵ�� ��Ϊ���Ͼ���Խ��Խ�ã����ϵ��Ϊ����
ksp  = 1; % ��ת����ϵ��

%%������ָ����м��㣺
for i = 1:n
    reachGoalAccuracy = [reachGoalAccuracy; norm(arc_success{i}(end,:) - Goal(1:3))];
    numOfSpinning = [numOfSpinning;size(L{i},1)];
    for j = 1:numOfObstacle
        minLength(j) = minLengthPoint2Line(Obstacle(j,:),arc_success{i});%ʼ��Ϊ��
    end
    abiliAvoidObstacle = [abiliAvoidObstacle;min(minLength)];%����õ����������ϰ���㵽�켣����ľ��룬ע�����ֵԽ��Խ��
    totalScore = kg*reachGoalAccuracy(i) + ko * abiliAvoidObstacle(i) + ksp * numOfSpinning(i);
    totalEvaluate = [totalEvaluate;totalScore];
end
    
switch flag
    case 1
        Index = find(reachGoalAccuracy == min(reachGoalAccuracy));
        evres = Index(1);
        sprintf('�ао�����ߵĹ켣Ϊ��%d��',Index(1))
    case 2
        Index = find(numOfSpinning == min(numOfSpinning));
        evres = Index(1);
        sprintf('��ת�������ٵĹ켣Ϊ��%d��',Index(1))
    case 3
        Index = find(abiliAvoidObstacle == max(abiliAvoidObstacle));
        evres = Index(1);
        sprintf('����������ǿ�Ĺ켣Ϊ��%d��',Index(1))
    case 4
        Index = find(totalEvaluate == min(totalEvaluate));
        evres = Index(1);
        sprintf('�ۺ�ˮƽ��һ�Ĺ켣Ϊ��%d��',Index(1))
end
        
end

%%�Ӻ����� �ϰ���㵽�켣�ľ�����㣺
function minLength = minLengthPoint2Line(Obstacle,arc_success)
l = size(arc_success,1);
for i = 1:l
    distancePoint2Line(i) = norm(arc_success(i,:)-Obstacle(1:3));
end
minLength = min(distancePoint2Line);
end