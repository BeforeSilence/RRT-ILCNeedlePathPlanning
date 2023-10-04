%% DSL: This function is used for random generation of insertion scene
% Start at 20230828:16:35
% End at 20230828:16:55
% Inspired by GPT
function world = randomInsertScene(corner, num_spheres, radius)
% 定义长方体的两个角点坐标
% corner1 = [0, 0, -100]; % 第一个角点坐标
% corner2 = [50, 50, 0]; % 第二个角点坐标

corner1 = corner(1,:);
corner2 = corner(2,:);

% 随机生成球体的坐标和半径
% num_spheres = 6; % 生成球体的数量
% min_radius = 5;   % 最小半径
% max_radius = 10;   % 最大半径

min_radius = radius(1);
max_radius = radius(2);

% 生成随机球体
% spheres = struct('x', {}, 'y', {}, 'z', {}, 'radius', {});
spheres = [];
for i = 1:num_spheres
    % 随机生成球体的坐标和半径
    x = rand() * (corner2(1) - corner1(1)) + corner1(1);
    y = rand() * (corner2(2) - corner1(2)) + corner1(2);
    z = rand() * (corner2(3) - corner1(3)) + corner1(3);
    radius = rand() * (max_radius - min_radius) + min_radius;
    
    % 碰撞检测和位置调整
    collision = true;
    while collision
        collision = false;
        for j = 1:size(spheres,1)
            dist = sqrt((x - spheres(j,1))^2 + (y - spheres(j,2))^2 + (z - spheres(j,3))^2);
            if dist < radius + spheres(j,4)
                collision = true;
                x = rand() * (corner2(1) - corner1(1)) + corner1(1);
                y = rand() * (corner2(2) - corner1(2)) + corner1(2);
                z = rand() * (corner2(3) - corner1(3)) + corner1(3);
                break;
            end
        end
        
        % 检查球体是否超出边界
        if x - radius < corner1(1) || x + radius > corner2(1) || ...
           y - radius < corner1(2) || y + radius > corner2(2) || ...
           z - radius < corner1(3) || z + radius > corner2(3)
            collision = true;
            x = rand() * (corner2(1) - corner1(1)) + corner1(1);
            y = rand() * (corner2(2) - corner1(2)) + corner1(2);
            z = rand() * (corner2(3) - corner1(3)) + corner1(3);
        end
    end
    
    spheres(i,1) = x;
    spheres(i,2) = y;
    spheres(i,3) = z;
    spheres(i,4) = radius;
end
spheres = sortrows(spheres,3);
spheres(1,4) = 3;% 目标点始终为3的半径
% figure()
world.obstacles = spheres;
% plotObstacle(world)
% hold on
% axis equal;
end