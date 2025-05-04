% 测试粒子编解码验证程序
% 作者：@back001001
% 时间：2025-05-04

clear;
clc;

% 1. 设置测试数据
testPosition = [0.31, 0.78, 0.45, 0.92, 0.23, 0.67];  % 测试位置向量
fprintf('测试位置向量:\n');
disp(testPosition);

% 2. 创建映射表
mapping = {'T1', 'T2', 'T2', 'T3', 'U1', 'U2'};  % 示例映射表
fprintf('\n映射表:\n');
disp(mapping);

% 3. 对位置向量排序并获取编号
[~, indices] = sort(testPosition);
sequence_numbers = zeros(size(testPosition));
for i = 1:length(indices)
    sequence_numbers(indices(i)) = i;
end

% 显示排序过程
fprintf('\n排序过程:\n');
fprintf('原始位置值\t排序后索引\t获得的编号\n');
for i = 1:length(testPosition)
    fprintf('%.2f\t\t%d\t\t%d\n', testPosition(i), indices(i), sequence_numbers(i));
end

% 4. 映射到任务序列
mapped_sequence = cell(size(testPosition));
for i = 1:length(testPosition)
    mapped_sequence{i} = mapping{sequence_numbers(i)};
end

% 显示最终映射结果
fprintf('\n最终映射结果:\n');
fprintf('位置\t位置值\t编号\t映射元素\n');
for i = 1:length(testPosition)
    fprintf('%d\t%.2f\t%d\t%s\n', i, testPosition(i), sequence_numbers(i), mapped_sequence{i});
end

% 5. 解析无人机任务分配
fprintf('\n解析无人机任务分配:\n');

% 找到所有无人机标记的位置
uav_positions = [];
for i = 1:length(mapped_sequence)
    if strncmp(mapped_sequence{i}, 'U', 1)
        uav_id = str2double(mapped_sequence{i}(2:end));
        uav_positions = [uav_positions; i, uav_id];
    end
end
uav_positions = sortrows(uav_positions, 1);

% 初始化无人机任务路径
UAVPaths = cell(3, 1);  % 3个无人机

% 提取任务的辅助函数
function tasks = ExtractTasks(sequence)
    tasks = [];
    for i = 1:length(sequence)
        if sequence{i}(1) == 'T'
            tasks = [tasks, str2double(sequence{i}(2:end))];
        end
    end
end

% 分配任务
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
    last_uav = 3;  % 总共3个无人机
    UAVPaths{last_uav} = ExtractTasks(mapped_sequence(uav_positions(end,1)+1:end));
end

% 显示最终任务分配结果
fprintf('\n最终任务分配结果:\n');
for i = 1:length(UAVPaths)
    fprintf('无人机 %d 的任务序列: ', i);
    disp(UAVPaths{i});
end
