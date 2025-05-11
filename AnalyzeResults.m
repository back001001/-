% 协同任务分配结果分析脚本（完整功能版）
% 作者: Back0077
% 功能: 自动分析实验数据、生成路径图、创建文本报告、备份脚本
clear; clc;

%% 参数配置
% ========== 用户可修改区域 ==========
targetGroup = 'Group1_Original';   % 分析的参数组名称
lineColors = {'#FF0000', '#00FF00', '#0000FF'}; % 红/绿/蓝路径颜色
reportEncoding = 'UTF-8';          % 文本报告编码格式
% ===================================

%% 自动创建带时间戳的分析文件夹
saveRootDir = fullfile(pwd, 'Analysis');
analysisTime = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
saveDir_min = fullfile(saveRootDir, [targetGroup, '_Min_', analysisTime]);
saveDir_max = fullfile(saveRootDir, [targetGroup, '_Max_', analysisTime]);

% 创建文件夹并验证
createDir(saveDir_min);
createDir(saveDir_max);
fprintf('分析文件夹创建成功:\n- %s\n- %s\n\n', saveDir_min, saveDir_max);

%% 加载实验数据
srcDir = fullfile('Results', targetGroup);
files = dir(fullfile(srcDir, 'Run*.mat'));

if isempty(files)
    error('实验数据未找到，请先运行RunExperiments.m');
end

%% 搜索最优/最差解
[minCost, maxCost] = deal(inf, -inf);
[bestRunIdx, worstRunIdx] = deal(1, 1);

for i = 1:length(files)
    data = load(fullfile(srcDir, files(i).name));
    
    % 数据完整性校验
    validateDataStructure(data);
    
    % 更新极值记录
    currentCost = data.GlobalBest.Cost;
    if currentCost < minCost
        minCost = currentCost;
        bestRunIdx = i;
    end
    if currentCost > maxCost
        maxCost = currentCost;
        worstRunIdx = i;
    end
end

%% 保存关键结果
bestData = load(fullfile(srcDir, files(bestRunIdx).name));
worstData = load(fullfile(srcDir, files(worstRunIdx).name));

saveResults(saveDir_min, bestData, 'BestResult.mat');
saveResults(saveDir_max, worstData, 'WorstResult.mat');

%% 可视化与报告生成
% 绘制路径图
PlotPath(saveDir_min, bestData, '最优任务分配路径图', lineColors);
PlotPath(saveDir_max, worstData, '最差任务分配路径表', lineColors);

% 生成文本报告
generateTextReport(saveDir_min, bestData, reportEncoding);
generateTextReport(saveDir_max, worstData, reportEncoding);

%% 系统维护
backupScript(saveDir_min);
backupScript(saveDir_max);
fprintf('\n===== 分析流程完成 =====\n');

%------------------------ 子函数定义 ------------------------%
%% 目录创建函数
function createDir(dirPath)
    if ~exist(dirPath, 'dir')
        try
            mkdir(dirPath);
        catch
            error('目录创建失败: %s', dirPath);
        end
    end
end

%% 数据校验函数
function validateDataStructure(data)
    requiredFields = {'GlobalBest', 'Targets', 'UAVs'};
    for i = 1:length(requiredFields)
        if ~isfield(data, requiredFields{i})
            error('数据字段缺失: %s', requiredFields{i});
        end
    end
    if ~isfield(data.GlobalBest, 'Allocation') || ~isfield(data.GlobalBest, 'Cost')
        error('GlobalBest结构体字段不完整');
    end
end

%% 结果保存函数
function saveResults(saveDir, data, fileName)
    try
        save(fullfile(saveDir, fileName), '-struct', 'data');
    catch
        error('结果保存失败: %s', fullfile(saveDir, fileName));
    end
end


%% 绘图函数（图例优化）
function PlotPath(saveDir, data, titleText, colors)
    figure;
    set(gcf, 'Color', 'white', 'Position', [100, 100, 800, 600]);
    hold on;
    
    % === 1. 绘制目标点：黑色五角星 ===
    scatter(data.Targets(:,2), data.Targets(:,3), 120, 'k', 'p', 'filled', 'MarkerEdgeColor', 'k');
    
    % === 2. 绘制无人机基地：颜色区分三角形 ===
    h_base = gobjects(3,1);  % 图形句柄数组
    for i = 1:size(data.UAVs,1)
        h_base(i) = scatter(data.UAVs(i,2), data.UAVs(i,3), 200, '^', ...
            'MarkerFaceColor', colors{i}, 'MarkerEdgeColor', 'k');
    end
    
    % === 3. 绘制任务路径：颜色区分实线 ===
    h_path = gobjects(3,1);  % 图形句柄数组
    for uav = 1:length(data.GlobalBest.Allocation)
        path = data.GlobalBest.Allocation{uav};
        base = data.UAVs(uav,2:3);
        color = colors{uav};
        
        if ~isempty(path)
            % 基地到第一个目标
            h_path(uav) = plot([base(1), data.Targets(path(1),2)], ...
                 [base(2), data.Targets(path(1),3)], ...
                '-', 'Color', color, 'LineWidth', 2.5);
            
            % 目标间路径
            for i = 1:length(path)-1
                plot([data.Targets(path(i),2), data.Targets(path(i+1),2)], ...
                     [data.Targets(path(i),3), data.Targets(path(i+1),3)], ...
                    '-', 'Color', color, 'LineWidth', 2.5);
            end
            
            % 返回基地
            plot([data.Targets(path(end),2), base(1)], ...
                 [data.Targets(path(end),3), base(2)], ...
                '-', 'Color', color, 'LineWidth', 2.5);
        end
    end
    
    % === 4. 坐标轴与标签设置 ===
    set(gca, 'Box', 'on', 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.5);
    xlabel('X 坐标 (km)', 'FontName', 'Microsoft YaHei', 'FontSize', 12);
    ylabel('Y 坐标 (km)', 'FontName', 'Microsoft YaHei', 'FontSize', 12);
    
    % === 5. 标题包含适应度值 ===
    fitnessValue = data.GlobalBest.Cost;
    title(sprintf('%s\n适应度值: %.2f', titleText, fitnessValue), ...
        'FontName', 'Microsoft YaHei', 'FontSize', 14);
    
    % === 6. 固定坐标轴范围 ===
    xlim([0 100]);
    ylim([0 100]);
    
    % === 7. 图例优化 ===
    % 提取有效图形对象（过滤空句柄）
    valid_path = h_path(ishandle(h_path));
    valid_base = h_base(ishandle(h_base));
    
    % 生成标签
    legendLabels = [...
        arrayfun(@(i) sprintf('UAV%d 任务路径', i), 1:length(valid_path), 'UniformOutput', false), ...
        arrayfun(@(i) sprintf('UAV%d 基地', i), 1:length(valid_base), 'UniformOutput', false)];
    
    % 绘制图例
    legend([valid_path; valid_base], legendLabels, ...
        'Location', 'eastoutside', ...
        'FontSize', 10, ...
        'Box', 'off');
    
    % 调整布局避免图例遮挡
    set(gcf, 'Position', [100, 100, 900, 600]);  % 增加画布宽度
    
    % 保存高清图片
    exportgraphics(gcf, fullfile(saveDir, 'TaskAllocationMap.png'), 'Resolution', 300);
    fprintf('路径图已保存: %s\n', fullfile(saveDir, 'TaskAllocationMap.png'));
    close(gcf);
end
%% 文本报告生成函数
function generateTextReport(saveDir, data, encoding)
    reportPath = fullfile(saveDir, 'TaskAllocationReport.txt');
    
    try
        fid = fopen(reportPath, 'w', 'n', encoding);
    catch
        error('文件创建失败: %s', reportPath);
    end
    
    % 报告头信息
    fprintf(fid, '协同任务分配分析报告\n');
    fprintf(fid, '生成时间: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf(fid, '适应度值: %.2f\n\n', data.GlobalBest.Cost);
    
    % 无人机任务详情
    fprintf(fid, '===== 任务分配详情 =====\n');
    for uavID = 1:length(data.GlobalBest.Allocation)
        taskPath = data.GlobalBest.Allocation{uavID};
        
        if isempty(taskPath)
            fprintf(fid, 'UAV%d: 未分配任务\n\n', uavID);
        else
            targets = arrayfun(@(x) sprintf('T%02d', x), taskPath, 'UniformOutput', false);
            fprintf(fid, 'UAV%d任务序列:\n%s\n\n', uavID, strjoin(targets, ' → '));
        end
    end
    
    % 统计摘要
    fprintf(fid, '===== 系统统计 =====\n');
    fprintf(fid, '总任务点数: %d\n', size(data.Targets, 1));
    fprintf(fid, '参与无人机数: %d\n', size(data.UAVs, 1));
    fprintf(fid, '最优路径成本: %.2f\n', data.GlobalBest.Cost);
    
    fclose(fid);
    fprintf('文本报告已生成: %s\n', reportPath);
end

%% 脚本备份函数
function backupScript(saveDir)
    try
        copyfile([mfilename('fullpath'), '.m'], saveDir);
    catch
        warning('脚本备份失败: %s', saveDir);
    end
end