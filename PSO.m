%% 无人机协同任务分配标准PSO算法实现
% 作者: Back0077

% 清空工作空间和命令窗
%clear; clc;

%% 参数设置
Nu = 3;   % 无人机数量
Nt = 10;  % 目标数量
MaxIter = 2000;   % 最大迭代次数
PopSize = 50;   % 种群规模
b1 = 0.5;   % 提前到达惩罚系数
b2 = 0.5;   % 延迟惩罚系数

% PSO参数
w=1;
c1 = 2; c2 = 2;   

% 无人机参数矩阵 [编号, 基地X坐标, 基地Y坐标, 攻击成功率, 最大任务数, 飞行速度]
UAVs = [
    1, 15, 70, 0.92, 6, 5;
    2, 85, 20, 0.88, 5, 6;
    3, 45, 90, 0.85, 7, 4;
];

% 目标参数矩阵 [编号, X坐标, Y坐标, 价值, 时间窗开始, 时间窗结束, 执行耗时, 所需无人机数]
Targets = [
    1, 30, 80, 35,  5, 40, 1, 1;
    2, 70, 35, 45, 15, 50, 2, 2;
    3, 10, 50, 25, 10, 60, 1, 1;
    4, 95, 60, 50, 20, 80, 2, 1;
    5, 50, 15, 30,  5, 30, 1, 2;
    6, 25, 25, 40, 25, 55, 2, 1;
    7, 80, 75, 28, 10, 45, 1, 1;
    8, 40, 40, 55, 15, 65, 2, 2;
    9, 60, 50, 32, 30, 70, 1, 1;
    10, 90, 90, 48, 40, 90, 2, 1;
];

%% 初始化粒子群
D = sum(Targets(:,8)) + Nu - 1;  % 粒子维度
PosRange = [1, D];    % 位置范围
VelRange = [-5, 5];   % 速度范围

% 全局最优解
GlobalBest = struct('Position', [], 'Cost', inf);
GlobalBest.Position = rand(1,D)*(PosRange(2)-PosRange(1)) + PosRange(1);

% 初始化粒子
Particles = struct();
for i = 1:PopSize
    Particles(i).Position = rand(1,D)*(PosRange(2)-PosRange(1)) + PosRange(1);
    Particles(i).Velocity = rand(1,D)*(VelRange(2)-VelRange(1)) + VelRange(1);
    Particles(i).Best = struct('Position', Particles(i).Position, 'Cost', inf);
    Particles(i).Cost = inf;
end

%% PSO主循环
PSO_Cost = zeros(1, MaxIter);  % 记录最优成本

for iter = 1:MaxIter
    for k = 1:PopSize
        % 解码粒子并验证有效性
        [UAVPaths, isValid] = DecodeParticle(Particles(k).Position, UAVs, Targets);
        if ~isValid
            Particles(k).Cost = inf;
            continue;
        end
        
        % 计算适应度
        f_k = FitnessFunction(UAVPaths, UAVs, Targets, b1, b2);
        Particles(k).Cost = f_k;
        
        % 更新个体最优
        if f_k < Particles(k).Best.Cost
            Particles(k).Best.Position = Particles(k).Position;
            Particles(k).Best.Cost = f_k;
        end
        
        % 更新全局最优
        if f_k < GlobalBest.Cost
            GlobalBest.Position = Particles(k).Position;
            GlobalBest.Cost = f_k;
        end
    end
    
    % 更新粒子状态
    for k = 1:PopSize
        % 生成随机数
        r1 = rand(1,D);
        r2 = rand(1,D);
        
        % 更新速度和位置
        Particles(k).Velocity = w * Particles(k).Velocity + ...
            c1 * r1 .* (Particles(k).Best.Position - Particles(k).Position) + ...
            c2 * r2 .* (GlobalBest.Position - Particles(k).Position);
        
        % 限制速度范围
        Particles(k).Velocity = max(min(Particles(k).Velocity, VelRange(2)), VelRange(1));
        
        % 更新位置
        Particles(k).Position = Particles(k).Position + Particles(k).Velocity;
        Particles(k).Position = max(min(Particles(k).Position, PosRange(2)), PosRange(1));
    end
    
    % 记录最优解
    PSO_Cost(iter) = GlobalBest.Cost;
    
    % 输出进度
        fprintf('迭代 %d, 最优成本: %.2f\n', iter, GlobalBest.Cost);
end
%% 输出结果
[finalPaths, ~] = DecodeParticle(GlobalBest.Position, UAVs, Targets);
fprintf('\n最终分配方案:\n');
for i = 1:Nu
    fprintf('无人机 %d 的任务序列: ', i);
    disp(finalPaths{i});
end
fprintf('总成本: %.2f\n', GlobalBest.Cost);

%% 绘制收敛曲线



%% 辅助函数
% DecodeParticle - 解码粒子位置为任务分配方案
function [UAVPaths, isValid] = DecodeParticle(Position, UAVs, Targets)
    Nu = size(UAVs,1);    % 无人机数量
    Nt = size(Targets,1); % 目标数量
    
    % 1. 创建映射表
    mapping = {};
    map_idx = 1;
    
    % 遍历每个目标，根据所需无人机数量添加到映射表
    for i = 1:Nt
        required_uavs = Targets(i,8);  % 获取目标i所需的无人机数量
        for j = 1:required_uavs
            mapping{map_idx} = ['T', num2str(i)];
            map_idx = map_idx + 1;
        end
    end
    
    % 添加无人机标记到映射表
    for i = 1:(Nu-1)
        mapping{map_idx} = ['U', num2str(i)];
        map_idx = map_idx + 1;
    end
    
    % 2. 对位置向量排序获得序号
    [~, indices] = sort(Position);
    sequence_numbers = zeros(size(Position));
    for i = 1:length(indices)
        sequence_numbers(indices(i)) = i;
    end
    
    % 3. 将序号映射为对应的任务或无人机标记
    mapped_sequence = cell(size(Position));
    for i = 1:length(Position)
        mapped_sequence{i} = mapping{sequence_numbers(i)};
    end
    
    % 4. 找到所有无人机标记的位置
    uav_positions = [];
    for i = 1:length(mapped_sequence)
        if strncmp(mapped_sequence{i}, 'U', 1)
            uav_id = str2double(mapped_sequence{i}(2:end));
            uav_positions = [uav_positions; i, uav_id];
        end
    end
    uav_positions = sortrows(uav_positions, 1);
    
    % 5. 初始化无人机任务路径
    UAVPaths = cell(Nu, 1);
    
    % 6. 根据无人机标记位置分配任务
    if ~isempty(uav_positions)
        % 处理第一个无人机的任务
        first_uav = uav_positions(1,2);
        UAVPaths{first_uav} = ExtractTasks(mapped_sequence(1:uav_positions(1,1)-1));
        
        % 处理中间无人机的任务
        for i = 1:size(uav_positions,1)-1
            current_uav = uav_positions(i+1,2);
            start_pos = uav_positions(i,1) + 1;
            end_pos = uav_positions(i+1,1) - 1;
            UAVPaths{current_uav} = ExtractTasks(mapped_sequence(start_pos:end_pos));
        end
        
        % 处理最后一个无人机的任务
        last_uav = Nu;
        UAVPaths{last_uav} = ExtractTasks(mapped_sequence(uav_positions(end,1)+1:end));
    else
        % 如果没有无人机标记，均匀分配任务
        tasksPerUAV = floor(length(mapped_sequence)/Nu);
        extra = mod(length(mapped_sequence), Nu);
        start_idx = 1;
        
        for i = 1:Nu
            num_tasks = tasksPerUAV + (i <= extra);
            end_idx = start_idx + num_tasks - 1;
            UAVPaths{i} = ExtractTasks(mapped_sequence(start_idx:end_idx));
            start_idx = end_idx + 1;
        end
    end
    
    % 7. 修复并验证解决方案
    [UAVPaths, isValid] = RepairSolution(UAVPaths, UAVs, Targets);
end

% 提取任务序号
function tasks = ExtractTasks(sequence)
    tasks = [];
    for i = 1:length(sequence)
        if sequence{i}(1) == 'T'
            tasks = [tasks, str2double(sequence{i}(2:end))];
        end
    end
end

% 修复解决方案使其满足约束条件
function [UAVPaths, isValid] = RepairSolution(UAVPaths, UAVs, Targets)
    Nu = size(UAVs, 1);
    Nt = size(Targets, 1);
    maxAttempts = 5;
    isValid = false;
    
    for attempt = 1:maxAttempts
        % 1. 移除重复任务
        for i = 1:Nu
            UAVPaths{i} = unique(UAVPaths{i}, 'stable');
        end
        
        % 2. 确保不超过最大任务数限制
        for i = 1:Nu
            if length(UAVPaths{i}) > UAVs(i,5)
                UAVPaths{i} = UAVPaths{i}(1:UAVs(i,5));
            end
        end
        
        % 3. 统计任务分配情况
        taskAssignments = zeros(Nt, Nu);
        for i = 1:Nu
            for task = UAVPaths{i}
                if task <= Nt
                    taskAssignments(task, i) = 1;
                end
            end
        end
        
        % 4. 修复任务分配数量
        modified = false;
        for t = 1:Nt
            required = Targets(t,8);
            current = sum(taskAssignments(t,:));
            
            if current ~= required
                modified = true;
                assigned = find(taskAssignments(t,:));
                unassigned = setdiff(1:Nu, assigned);
                
                if current < required
                    % 需要增加分配
                    needed = required - current;
                    available = [];
                    for uav = unassigned
                        if length(UAVPaths{uav}) < UAVs(uav,5)
                            available = [available, uav];
                        end
                    end
                    
                    if ~isempty(available)
                        toAdd = available(randperm(length(available), ...
                            min(needed, length(available))));
                        for uav = toAdd
                            UAVPaths{uav} = [UAVPaths{uav}, t];
                            taskAssignments(t,uav) = 1;
                        end
                    end
                else
                    % 需要减少分配
                    excess = current - required;
                    if ~isempty(assigned)
                        toRemove = assigned(randperm(length(assigned), ...
                            min(excess, length(assigned))));
                        for uav = toRemove
                            UAVPaths{uav}(UAVPaths{uav}==t) = [];
                            taskAssignments(t,uav) = 0;
                        end
                    end
                end
            end
        end

        % 检测并处理死锁情况
        [hasDeadlock, strongComponents] = DetectDeadlock(UAVPaths, Targets);
        if hasDeadlock
            UAVPaths = ResolveDeadlock(UAVPaths, strongComponents);
            continue;
        end

        % 验证约束条件
        if CheckConstraints(UAVPaths, UAVs, Targets)
            isValid = true;
            break;
        end
        
        % 如果没有修改且方案无效，随机重新分配
        if ~modified
            for i = 1:Nu
                UAVPaths{i} = [];
            end
        end
    end
end

% 验证约束条件
function valid = CheckConstraints(UAVPaths, UAVs, Targets)
    Nu = size(UAVs, 1);
    Nt = size(Targets, 1);
    valid = true;
    
    % 1. 检查任务数约束和重复任务
    for i = 1:Nu
        if length(UAVPaths{i}) > UAVs(i,5)
            valid = false;
            return;
        end
        if length(unique(UAVPaths{i})) < length(UAVPaths{i})
            valid = false;
            return;
        end
    end
    
    % 2. 检查任务分配数量
    taskCounts = zeros(1, Nt);
    for i = 1:Nu
        for task = UAVPaths{i}
            if task <= Nt
                taskCounts(task) = taskCounts(task) + 1;
            else
                valid = false;
                return;
            end
        end
    end
    
    % 3. 验证每个任务的分配数量是否正确
    valid = all(taskCounts == [Targets(:,8)]');
end

% 局部随机搜索
function [best_position, best_cost] = LocalRandomSearch(particle, particles, UAVs, Targets, b1, b2)
    % 初始化搜索参数
    N = 0;
    N_max = 5;
    K = 1;
    K_max = 4;
    
    best_position = particle.Position;
    best_cost = particle.Cost;
    current_position = particle.Position;
    current_cost = particle.Cost;
    
    % 执行局部搜索
    while N <= N_max
        K = 1;
        while K <= K_max
            if rand < 1/K_max
                try
                    % 根据不同策略生成新解
                    switch K
                        case 1
                            temp_position = SinglePointSwapOperator(current_position);
                        case 2
                            temp_position = SlideOperator(current_position);
                        case 3
                            temp_position = ReverseOperator(current_position);
                        case 4
                            m = randi(length(particles));
                            other_position = particles(m).Position;
                            temp_position = ExchangeOperator(current_position, other_position);
                    end
                    
                    % 评估新解
                    [UAVPaths, isValid] = DecodeParticle(temp_position, UAVs, Targets);
                    
                    if isValid
                        new_cost = FitnessFunction(UAVPaths, UAVs, Targets, b1, b2);
                        
                        if new_cost < current_cost
                            current_position = temp_position;
                            current_cost = new_cost;
                            
                            if new_cost < best_cost
                                best_position = temp_position;
                                best_cost = new_cost;
                            end
                            
                            K = 1;
                            continue;
                        end
                    end
                catch
                    continue;
                end
            end
            K = K + 1;
        end
        N = N + 1;
    end
end

% 单点交换算子
function new_position = SinglePointSwapOperator(position)
    L = length(position);
    if L <= 1
        new_position = position;
        return;
    end
    
    % 随机选择两个不同位置交换
    i = randi(L);
    j = randi(L);
    while i == j && L > 1
        j = randi(L);
    end
    
    new_position = position;
    new_position([i,j]) = position([j,i]);
end

% 滑动算子
function new_position = SlideOperator(position)
    L = length(position);
    if L <= 2
        new_position = position;
        return;
    end
    
    % 生成滑动范围和方向
    i = randi(L-1);
    j = min(i + 1 + randi(L-i), L);
    Dir = randi([0,1]);
    n = randi(max(1, min(3, abs(i-j)-1)));
    
    new_position = position;
    segment = position(i:j);
    
    % 执行滑动操作
    if Dir == 0  % 向左滑动
        move_start = max(1, i-n);
        move_end = min(L, j-n);
        new_position(move_start:move_end) = segment(1:(move_end-move_start+1));
    else  % 向右滑动
        move_start = min(L, i+n);
        move_end = min(L, j+n);
        new_position(move_start:move_end) = segment(1:(move_end-move_start+1));
    end
end

% 翻转算子
function new_position = ReverseOperator(position)
    L = length(position);
    if L <= 1
        new_position = position;
        return;
    end
    
    % 生成翻转区间
    i = randi(L-1);
    j = i + randi(L-i);
    
    i = max(1, min(i, L-1));
    j = max(i+1, min(j, L));
    
    % 执行翻转操作
    new_position = position;
    segment = position(i:j);
    new_position(i:j) = fliplr(segment);
end

% 交换算子
function new_position = ExchangeOperator(position, other_position)
    L = length(position);
    if L <= 1
        new_position = position;
        return;
    end
    % 在随机位置交换两个解的后半部分
    i = randi(max(1, L-1));
    
    new_position = position;
    new_position(i:end) = other_position(i:end);
end

% 适应度函数
function J = FitnessFunction(UAVPaths, UAVs, Targets, b1, b2)
    Nu = length(UAVPaths);
    a = [1, 1, 1, 1];  % 目标函数权重系数
    D_total = 0; t_max = 0; TW_total = 0; I_total = 0;
    
    for uav = 1:Nu
        path = UAVPaths{uav};
        if isempty(path), continue; end
        
        base = UAVs(uav,2:3);
        speed = UAVs(uav,6);
        P = UAVs(uav,4);
        
        current_pos = base;
        arrival_time = 0;
        total_dist = 0;
        TW_uav = 0;
        I_uav = 0;
        
        % 计算每个任务的执行情况
        for i = 1:length(path)
            tgt = Targets(path(i),:);
            dist = norm(current_pos - tgt(2:3));
            
            % 计算到达时间
            if i == 1
                arrival_time = arrival_time + dist/speed;
            else
                arrival_time = arrival_time + tgt(7) + dist/speed;
            end
            
            % 处理协同任务
            if tgt(8) > 1
                collaborators = arrayfun(@(u) GetArrivalTime(UAVPaths{u}, path(i), UAVs(u,:), Targets), ...
                    find(cellfun(@(x) ismember(path(i), x), UAVPaths)));
                exec_time = max(collaborators);
            else
                exec_time = arrival_time;
            end
            
            % 计算时间窗口惩罚
            TW_uav = TW_uav + max(tgt(5)-exec_time,0)*b1 + max(exec_time-tgt(6),0)*b2;
            
            % 计算任务收益
            I_uav = I_uav + P * tgt(4);
            
            % 更新总距离和当前位置
            total_dist = total_dist + dist;
            current_pos = tgt(2:3);
        end
        
        % 计算返回基地的成本
        return_dist = norm(current_pos - base);
        D_total = D_total + total_dist + return_dist;
        t_max = max(t_max, arrival_time + return_dist/speed);
        TW_total = TW_total + TW_uav;
        I_total = I_total + I_uav;
    end
    
    % 计算总目标函数值
    J = a(1)*D_total + a(2)*t_max + a(3)*TW_total - a(4)*I_total;
end

% 获取到达时间
function arrival = GetArrivalTime(path, target, uav, Targets)
    base = uav(2:3);
    speed = uav(6);
    current_pos = base;
    arrival = 0;
    
    % 计算到达指定目标的时间
    for i = 1:length(path)
        tgt = Targets(path(i),:);
        dist = norm(current_pos - tgt(2:3));
        if i == 1
            arrival = arrival + dist/speed;
        else
            arrival = arrival + tgt(7) + dist/speed;
        end
        if path(i) == target
            return;
        end
        current_pos = tgt(2:3);
    end
end

% 死锁检测
function [hasCycle, strongComponents] = DetectDeadlock(UAVPaths, Targets)
    % 构建任务依赖图
    Nt = size(Targets, 1);
    adjacencyMatrix = zeros(Nt, Nt);
    
    % 建立任务间的依赖关系
    for path = UAVPaths
        if ~isempty(path{1})
            for i = 1:(length(path{1})-1)
                currentTask = path{1}(i);
                nextTask = path{1}(i+1);
                adjacencyMatrix(currentTask, nextTask) = 1;
            end
        end
    end
    
    % 使用Kosaraju算法检测强连通分量
    [hasCycle, strongComponents] = FindStrongComponents(adjacencyMatrix);
end

% 查找强连通分量（续）
function [hasCycle, strongComponents] = FindStrongComponents(adjacencyMatrix)
    N = size(adjacencyMatrix, 1);
    visited = false(1, N);
    stack = [];
    strongComponents = {};
    
    % 第一次DFS，获取结点的完成顺序
    for i = 1:N
        if ~visited(i)
            [visited, stack] = DFS(adjacencyMatrix, i, visited, stack);
        end
    end
    
    % 转置图并重置访问标记
    transposedMatrix = adjacencyMatrix';
    visited = false(1, N);
    
    % 第二次DFS，找出强连通分量
    while ~isempty(stack)
        v = stack(end);
        stack(end) = [];
        if ~visited(v)
            component = [];
            [visited, component] = DFS(transposedMatrix, v, visited, component);
            if length(component) > 1
                strongComponents{end+1} = component;
            end
        end
    end
    
    % 判断是否存在环（强连通分量）
    hasCycle = ~isempty(strongComponents);
end

% DFS深度优先搜索辅助函数
function [visited, stack] = DFS(graph, v, visited, stack)
    visited(v) = true;  % 标记当前节点为已访问
    neighbors = find(graph(v,:));  % 找出所有邻接节点
    
    % 递归访问未访问的邻接节点
    for u = neighbors
        if ~visited(u)
            [visited, stack] = DFS(graph, u, visited, stack);
        end
    end
    
    % 将当前节点添加到栈中（完成顺序）
    stack = [stack, v];
end

% 解决死锁函数
function UAVPaths = ResolveDeadlock(UAVPaths, strongComponents)
    for comp = strongComponents
        component = comp{1};
        % 遍历每个无人机的路径
        for i = 1:length(UAVPaths)
            path = UAVPaths{i};
            if ~isempty(path)
                % 找到属于当前强连通分量的任务
                indices = find(ismember(path, component));
                if length(indices) > 1
                    % 通过反转任务顺序来打破循环
                    path(indices) = fliplr(path(indices));
                    UAVPaths{i} = path;
                end
            end
        end
    end
end
