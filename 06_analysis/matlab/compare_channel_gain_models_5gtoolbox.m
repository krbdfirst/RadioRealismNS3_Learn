%% compare_channel_gain_models_5gtoolbox.m
% Benchmark alternative regressors for toolbox_channel_gain_db on the same
% held-out split used by the generalized PHY pipeline.
%
% Required MATLAB products:
%   - MATLAB (base)
%   - Statistics and Machine Learning Toolbox

clear;
clc;

rrp = rr_paths();
resultsRoot = rrp.phyOutDir;
trainingCsv = fullfile(resultsRoot, "generalized_phy_training_dataset_5gtoolbox.csv");
outputDir = fullfile(resultsRoot, "channel_gain_model_benchmark");

if ~isfile(trainingCsv)
    error("Training dataset not found: %s", trainingCsv);
end

if ~isfolder(outputDir)
    mkdir(outputDir);
end

summaryCsv = fullfile(outputDir, "channel_gain_model_comparison.csv");
plotPng = fullfile(outputDir, "channel_gain_model_comparison.png");

T = readtable(trainingCsv, 'TextType', 'string');
if isempty(T)
    error("Training dataset is empty: %s", trainingCsv);
end

requiredVars = { ...
    'run_dir', ...
    'adoption_pct', ...
    'log10_distance_m', ...
    'relative_speed_mps', ...
    'radial_velocity_mps', ...
    'abs_radial_velocity_mps', ...
    'heading_alignment_cos', ...
    'los_probability', ...
    'los_flag', ...
    'toolbox_channel_gain_db'};

missingVars = setdiff(requiredVars, string(T.Properties.VariableNames));
if ~isempty(missingVars)
    error("Training dataset is missing required variables: %s", strjoin(missingVars, ", "));
end

if ~ismember("run_index", string(T.Properties.VariableNames))
    T.run_index = parseRunIndexFromRunDir(T.run_dir);
end

predictorNames = { ...
    'log10_distance_m', ...
    'relative_speed_mps', ...
    'radial_velocity_mps', ...
    'abs_radial_velocity_mps', ...
    'heading_alignment_cos', ...
    'los_probability'};

T.los_flag = double(T.los_flag);

holdoutRunDirs = strings(0,1);
adoptionLevels = unique(T.adoption_pct)';
for adoptionPct = adoptionLevels
    adoptionRows = unique(T(T.adoption_pct == adoptionPct, {'run_dir','run_index'}), 'rows');
    if height(adoptionRows) < 2
        continue;
    end
    [~, idx] = max(adoptionRows.run_index);
    holdoutRunDirs(end + 1, 1) = adoptionRows.run_dir(idx); %#ok<AGROW>
end

holdoutMask = ismember(T.run_dir, holdoutRunDirs);
if ~any(holdoutMask) || ~any(~holdoutMask)
    error("Could not construct a non-empty train/holdout split from %s", trainingCsv);
end

trainRows = T(~holdoutMask, :);
testRows = T(holdoutMask, :);

modelNames = [ ...
    "linear_baseline"
    "linear_interactions"
    "linear_quadratic"
    "bagged_trees"
    "lsboost_trees"
    "split_los_linear"];

models = repmat(struct('name', "", 'fit', [], 'predict', []), numel(modelNames), 1);
for i = 1:numel(modelNames)
    models(i).name = modelNames(i);
end

models(1).fit = @() fitlm(trainRows(:, [predictorNames, {'toolbox_channel_gain_db'}]), ...
    'toolbox_channel_gain_db ~ log10_distance_m + relative_speed_mps + radial_velocity_mps + abs_radial_velocity_mps + heading_alignment_cos + los_probability');
models(1).predict = @(model, tbl) predict(model, tbl(:, predictorNames));

models(2).fit = @() fitlm(trainRows(:, [predictorNames, {'toolbox_channel_gain_db'}]), ...
    'interactions', 'ResponseVar', 'toolbox_channel_gain_db');
models(2).predict = @(model, tbl) predict(model, tbl(:, predictorNames));

models(3).fit = @() fitlm(trainRows(:, [predictorNames, {'toolbox_channel_gain_db'}]), ...
    'quadratic', 'ResponseVar', 'toolbox_channel_gain_db');
models(3).predict = @(model, tbl) predict(model, tbl(:, predictorNames));

models(4).fit = @() fitrensemble(trainRows(:, [predictorNames, {'toolbox_channel_gain_db'}]), ...
    'toolbox_channel_gain_db', ...
    'Method', 'Bag', ...
    'NumLearningCycles', 200, ...
    'Learners', templateTree('MinLeafSize', 8));
models(4).predict = @(model, tbl) predict(model, tbl(:, predictorNames));

models(5).fit = @() fitrensemble(trainRows(:, [predictorNames, {'toolbox_channel_gain_db'}]), ...
    'toolbox_channel_gain_db', ...
    'Method', 'LSBoost', ...
    'NumLearningCycles', 300, ...
    'LearnRate', 0.05, ...
    'Learners', templateTree('MinLeafSize', 10));
models(5).predict = @(model, tbl) predict(model, tbl(:, predictorNames));

models(6).fit = @() fitSplitLosLinearModels(trainRows, predictorNames);
models(6).predict = @(model, tbl) predictSplitLosLinearModels(model, tbl, predictorNames);

summary = table('Size', [0 5], ...
    'VariableTypes', {'string','double','double','double','double'}, ...
    'VariableNames', {'model','train_r2','train_rmse','holdout_r2','holdout_rmse'});

holdoutPredictions = struct();

for i = 1:numel(models)
    fitted = models(i).fit();
    trainPred = models(i).predict(fitted, trainRows);
    holdoutPred = models(i).predict(fitted, testRows);

    summary = [summary; { ...
        models(i).name, ...
        rsquaredValue(trainRows.toolbox_channel_gain_db, trainPred), ...
        rmseValue(trainRows.toolbox_channel_gain_db, trainPred), ...
        rsquaredValue(testRows.toolbox_channel_gain_db, holdoutPred), ...
        rmseValue(testRows.toolbox_channel_gain_db, holdoutPred)}]; %#ok<AGROW>

    holdoutPredictions.(matlab.lang.makeValidName(models(i).name)) = holdoutPred;
end

summary = sortrows(summary, 'holdout_r2', 'descend');
writetable(summary, summaryCsv);
disp(summary);

bestModelName = summary.model(1);
bestField = matlab.lang.makeValidName(bestModelName);
bestPred = holdoutPredictions.(bestField);

fig = figure('Visible', 'off', 'Position', [100 100 1400 500]);
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
bar(categorical(summary.model), summary.holdout_r2);
ylabel('Holdout R^2');
title('Channel-Gain Model Comparison');
grid on;

nexttile;
scatter(testRows.toolbox_channel_gain_db, bestPred, 6, testRows.adoption_pct, 'filled');
xlabel('Target Channel Gain (dB)');
ylabel('Predicted Channel Gain (dB)');
title(sprintf('Best Holdout Model: %s (R^2=%.3f)', bestModelName, summary.holdout_r2(1)));
grid on;
refline(1,0);
colorbar;

exportgraphics(fig, plotPng, 'Resolution', 180);
close(fig);

fprintf("\nChannel-gain model benchmark complete.\n");
fprintf("Summary    : %s\n", summaryCsv);
fprintf("Diagnostic : %s\n\n", plotPng);

function model = fitSplitLosLinearModels(trainRows, predictorNames)
    losPredictors = [predictorNames, {'toolbox_channel_gain_db'}];
    losRows = trainRows(trainRows.los_flag > 0.5, :);
    nlosRows = trainRows(trainRows.los_flag <= 0.5, :);

    model = struct();
    model.los = fitlm(losRows(:, losPredictors), ...
        'toolbox_channel_gain_db ~ log10_distance_m + relative_speed_mps + radial_velocity_mps + abs_radial_velocity_mps + heading_alignment_cos + los_probability');
    model.nlos = fitlm(nlosRows(:, losPredictors), ...
        'toolbox_channel_gain_db ~ log10_distance_m + relative_speed_mps + radial_velocity_mps + abs_radial_velocity_mps + heading_alignment_cos + los_probability');
end

function pred = predictSplitLosLinearModels(model, rows, predictorNames)
    pred = zeros(height(rows), 1);
    losMask = rows.los_flag > 0.5;
    if any(losMask)
        pred(losMask) = predict(model.los, rows(losMask, predictorNames));
    end
    if any(~losMask)
        pred(~losMask) = predict(model.nlos, rows(~losMask, predictorNames));
    end
end

function value = rmseValue(actual, predicted)
    value = sqrt(mean((actual - predicted).^2));
end

function value = rsquaredValue(actual, predicted)
    sse = sum((actual - predicted).^2);
    sst = sum((actual - mean(actual)).^2);
    if sst <= eps
        value = 1.0;
    else
        value = 1.0 - (sse / sst);
    end
end

function runIndex = parseRunIndexFromRunDir(runDir)
    runIndex = zeros(numel(runDir), 1);
    for i = 1:numel(runDir)
        tokens = regexp(char(runDir(i)), 'Ideal_PHYModel_CAV\d+-(\d+)', 'tokens', 'once');
        if isempty(tokens)
            error("Could not parse run index from run_dir value: %s", string(runDir(i)));
        end
        runIndex(i) = str2double(tokens{1});
    end
end
