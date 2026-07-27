%% refresh_generalized_phy_diagnostics_5gtoolbox.m
% Refresh the generalized PHY diagnostics figure so the channel-gain panel
% reflects the deployed runtime model rather than the older baseline linear
% fit embedded in build_generalized_phy_model_5gtoolbox.m.
%
% Default behavior:
%   - uses the split LOS/NLOS linear channel-gain regressor
%   - reuses the existing training dataset and large-scale model outputs
%   - writes a new PNG and CSV beside the original diagnostics files
%
% Required MATLAB products:
%   - MATLAB (base)
%   - Statistics and Machine Learning Toolbox

clear;
clc;

%% Configuration
rrp = rr_paths();
resultsRoot = rrp.phyOutDir;
trainingCsv = fullfile(resultsRoot, "generalized_phy_training_dataset_5gtoolbox.csv");
metricsCsv = fullfile(resultsRoot, "generalized_phy_fit_metrics_5gtoolbox.csv");
modelMatFile = fullfile(resultsRoot, "generalized_phy_model_5gtoolbox.mat");

% Runtime-deployed channel-gain model.
% Options:
%   "split_los_linear"   -> deployed LOS/NLOS coefficient split (recommended)
%   "linear_quadratic"   -> compact single-model nonlinear alternative
%   "linear_interactions"
%   "linear_baseline"
%   "bagged_trees"
%   "lsboost_trees"
selectedChannelGainModel = "split_los_linear";

% "holdout" keeps the scatter honest to the benchmark score.
% "all" gives a denser cloud but mixes train and holdout rows.
channelGainScatterMode = "holdout";

overwriteOriginal = false;
updatedDiagPng = fullfile(resultsRoot, "generalized_phy_diagnostics_5gtoolbox_updated.png");
updatedMetricsCsv = fullfile(resultsRoot, "generalized_phy_fit_metrics_5gtoolbox_updated.csv");
if overwriteOriginal
    updatedDiagPng = fullfile(resultsRoot, "generalized_phy_diagnostics_5gtoolbox.png");
    updatedMetricsCsv = fullfile(resultsRoot, "generalized_phy_fit_metrics_5gtoolbox.csv");
end

if ~isfile(trainingCsv)
    error("Training dataset not found: %s", trainingCsv);
end
if ~isfile(metricsCsv)
    error("Base metrics CSV not found: %s", metricsCsv);
end
if ~isfile(modelMatFile)
    error("Base model MAT file not found: %s", modelMatFile);
end

%% Load inputs
T = readtable(trainingCsv, 'TextType', 'string');
if isempty(T)
    error("Training dataset is empty: %s", trainingCsv);
end

baseMetrics = readtable(metricsCsv, 'TextType', 'string');
S = load(modelMatFile, "pathLossModel", "rssiModel", "delayModel", "kFactorModel");

predictorNames = { ...
    'log10_distance_m', ...
    'relative_speed_mps', ...
    'radial_velocity_mps', ...
    'abs_radial_velocity_mps', ...
    'heading_alignment_cos', ...
    'los_probability'};

requiredVars = [ ...
    "run_dir", "adoption_pct", "toolbox_path_loss_db", "toolbox_rssi_dbm", ...
    "toolbox_delay_spread_ns", "toolbox_k_factor_db", "toolbox_channel_gain_db", ...
    "cdl_profile", "los_flag", predictorNames];
missingVars = setdiff(requiredVars, string(T.Properties.VariableNames));
if ~isempty(missingVars)
    error("Training dataset is missing required variables: %s", strjoin(missingVars, ", "));
end

if ~ismember("run_index", string(T.Properties.VariableNames))
    T.run_index = parseRunIndexFromRunDir(T.run_dir);
end
T.los_flag = double(T.los_flag);
profileOrder = unique(T.cdl_profile, 'stable');
profileOrder = profileOrder(~ismissing(profileOrder) & strlength(profileOrder) > 0);

%% Recreate the holdout split used by the benchmark
holdoutRunDirs = strings(0, 1);
adoptionLevels = unique(T.adoption_pct)';
for adoptionPct = adoptionLevels
    adoptionRows = unique(T(T.adoption_pct == adoptionPct, {'run_dir', 'run_index'}), 'rows');
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

%% Fit the selected channel-gain model
[channelGainModel, channelGainPredictor, channelGainLabel] = fitChannelGainModel( ...
    selectedChannelGainModel, trainRows, predictorNames);

trainPredGain = channelGainPredictor(channelGainModel, trainRows);
holdoutPredGain = channelGainPredictor(channelGainModel, testRows);
allPredGain = channelGainPredictor(channelGainModel, T);

trainR2Gain = rsquaredValue(trainRows.toolbox_channel_gain_db, trainPredGain);
trainRmseGain = rmseValue(trainRows.toolbox_channel_gain_db, trainPredGain);
holdoutR2Gain = rsquaredValue(testRows.toolbox_channel_gain_db, holdoutPredGain);
holdoutRmseGain = rmseValue(testRows.toolbox_channel_gain_db, holdoutPredGain);

%% Predict the unchanged metrics from the original generalized model
predPathLoss = predict(S.pathLossModel, T(:, predictorNames));
predRssi = predict(S.rssiModel, T(:, predictorNames));
predDelayNs = 10 .^ predict(S.delayModel, T(:, predictorNames));
predKFactor = predict(S.kFactorModel, T(:, predictorNames));

%% Build updated metrics table
metricsMap = containers.Map(baseMetrics.metric, 1:height(baseMetrics));
channelKey = "channel_gain_db";
if isKey(metricsMap, channelKey)
    baseMetrics{metricsMap(channelKey), "train_r2"} = trainR2Gain;
    baseMetrics{metricsMap(channelKey), "train_rmse"} = trainRmseGain;
    baseMetrics{metricsMap(channelKey), "holdout_r2"} = holdoutR2Gain;
    baseMetrics{metricsMap(channelKey), "holdout_rmse"} = holdoutRmseGain;
else
    baseMetrics = [baseMetrics; {channelKey, trainR2Gain, trainRmseGain, holdoutR2Gain, holdoutRmseGain}]; %#ok<AGROW>
end
writetable(baseMetrics, updatedMetricsCsv);

%% Diagnostics figure
fig = figure('Visible', 'off', 'Position', [100 100 1500 1000]);
tiledlayout(2, 4, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
if ismember("toolbox_rssi_shadowed_dbm", string(T.Properties.VariableNames))
    scatter(T.distance_m, T.toolbox_rssi_shadowed_dbm, 4, T.adoption_pct, 'filled');
else
    scatter(T.distance_m, T.toolbox_rssi_dbm, 4, T.adoption_pct, 'filled');
end
xlabel('Distance (m)');
ylabel('Toolbox RSSI (dBm)');
title('5G Toolbox Training Targets');
grid on;
colorbar;

nexttile;
scatter(T.toolbox_path_loss_db, predPathLoss, 4, T.adoption_pct, 'filled');
xlabel('Target Path Loss (dB)');
ylabel('Predicted Path Loss (dB)');
title(sprintf('Path-Loss Fit (R^2=%.3f)', metricValue(baseMetrics, "path_loss_db", "train_r2")));
grid on;
refline(1, 0);

nexttile;
scatter(T.toolbox_rssi_dbm, predRssi, 4, T.adoption_pct, 'filled');
xlabel('Target RSSI (dBm)');
ylabel('Predicted RSSI (dBm)');
title(sprintf('RSSI Fit (R^2=%.3f)', metricValue(baseMetrics, "rssi_dbm", "train_r2")));
grid on;
refline(1, 0);

nexttile;
scatter(T.toolbox_delay_spread_ns, predDelayNs, 4, T.adoption_pct, 'filled');
xlabel('Target Delay Spread (ns)');
ylabel('Predicted Delay Spread (ns)');
title(sprintf('Delay-Spread Fit (R^2=%.3f)', metricValue(baseMetrics, "delay_spread_ns", "train_r2")));
grid on;
refline(1, 0);

nexttile;
plotProfileDistributionComparison(T.cdl_profile, T.toolbox_k_factor_db, predKFactor, ...
    profileOrder, ...
    [0.20 0.45 0.80], [0.90 0.45 0.05], ...
    'K-Factor by CDL Profile (target vs predicted)', ...
    'K-Factor (dB)');

nexttile;
switch lower(channelGainScatterMode)
    case "holdout"
        plotProfileDistributionComparison(testRows.cdl_profile, testRows.toolbox_channel_gain_db, holdoutPredGain, ...
            profileOrder, ...
            [0.20 0.45 0.80], [0.90 0.45 0.05], ...
            sprintf('Channel Gain by CDL Profile (%s, Holdout R^2=%.3f)', channelGainLabel, holdoutR2Gain), ...
            'Channel Gain (dB)');
    case "all"
        plotProfileDistributionComparison(T.cdl_profile, T.toolbox_channel_gain_db, allPredGain, ...
            profileOrder, ...
            [0.20 0.45 0.80], [0.90 0.45 0.05], ...
            sprintf('Channel Gain by CDL Profile (%s, Train R^2=%.3f)', channelGainLabel, trainR2Gain), ...
            'Channel Gain (dB)');
    otherwise
        error("Unsupported channelGainScatterMode: %s", channelGainScatterMode);
end

nexttile;
boxchart(categorical(T.cdl_profile), T.toolbox_rssi_dbm);
xlabel('CDL Profile');
ylabel('Toolbox RSSI (dBm)');
title('RSSI by Selected CDL Profile');
grid on;

nexttile;
metricOrder = ["channel_gain_db"; "delay_spread_ns"; "k_factor_db"; "path_loss_db"; "rssi_dbm"];
trainR2 = zeros(numel(metricOrder), 1);
holdoutR2 = zeros(numel(metricOrder), 1);
for i = 1:numel(metricOrder)
    trainR2(i) = metricValue(baseMetrics, metricOrder(i), "train_r2");
    holdoutR2(i) = metricValue(baseMetrics, metricOrder(i), "holdout_r2");
end
bar(categorical(metricOrder, metricOrder), [trainR2 holdoutR2]);
ylabel('R^2');
title(sprintf('Train vs Holdout R^2 (%s for channel gain)', channelGainLabel));
legend('Train', 'Holdout', 'Location', 'southoutside');
grid on;

sgtitle(sprintf('Updated generalized PHY diagnostics (%s channel-gain model)', channelGainLabel), ...
    'FontWeight', 'bold');

exportgraphics(fig, updatedDiagPng, 'Resolution', 180);
close(fig);

fprintf("\nUpdated generalized PHY diagnostics complete.\n");
fprintf("Selected channel-gain model : %s\n", selectedChannelGainModel);
fprintf("Updated metrics CSV         : %s\n", updatedMetricsCsv);
fprintf("Updated diagnostics PNG     : %s\n\n", updatedDiagPng);
disp(baseMetrics);

%% Local functions
function [model, predictor, label] = fitChannelGainModel(modelName, trainRows, predictorNames)
    switch string(modelName)
        case "linear_baseline"
            model = fitlm(trainRows(:, [predictorNames, {'toolbox_channel_gain_db'}]), ...
                'toolbox_channel_gain_db ~ log10_distance_m + relative_speed_mps + radial_velocity_mps + abs_radial_velocity_mps + heading_alignment_cos + los_probability');
            predictor = @(m, rows) predict(m, rows(:, predictorNames));
            label = "linear baseline";

        case "linear_interactions"
            model = fitlm(trainRows(:, [predictorNames, {'toolbox_channel_gain_db'}]), ...
                'interactions', 'ResponseVar', 'toolbox_channel_gain_db');
            predictor = @(m, rows) predict(m, rows(:, predictorNames));
            label = "linear interactions";

        case "linear_quadratic"
            model = fitlm(trainRows(:, [predictorNames, {'toolbox_channel_gain_db'}]), ...
                'quadratic', 'ResponseVar', 'toolbox_channel_gain_db');
            predictor = @(m, rows) predict(m, rows(:, predictorNames));
            label = "linear quadratic";

        case "bagged_trees"
            model = fitrensemble(trainRows(:, [predictorNames, {'toolbox_channel_gain_db'}]), ...
                'toolbox_channel_gain_db', ...
                'Method', 'Bag', ...
                'NumLearningCycles', 200, ...
                'Learners', templateTree('MinLeafSize', 8));
            predictor = @(m, rows) predict(m, rows(:, predictorNames));
            label = "bagged trees";

        case "lsboost_trees"
            model = fitrensemble(trainRows(:, [predictorNames, {'toolbox_channel_gain_db'}]), ...
                'toolbox_channel_gain_db', ...
                'Method', 'LSBoost', ...
                'NumLearningCycles', 300, ...
                'LearnRate', 0.05, ...
                'Learners', templateTree('MinLeafSize', 10));
            predictor = @(m, rows) predict(m, rows(:, predictorNames));
            label = "LSBoost trees";

        case "split_los_linear"
            model = fitSplitLosLinearModels(trainRows, predictorNames);
            predictor = @(m, rows) predictSplitLosLinearModels(m, rows, predictorNames);
            label = "split LOS/NLOS linear";

        otherwise
            error("Unsupported channel-gain model: %s", string(modelName));
    end
end

function model = fitSplitLosLinearModels(trainRows, predictorNames)
    predictorBlock = [predictorNames, {'toolbox_channel_gain_db'}];
    losRows = trainRows(trainRows.los_flag > 0.5, :);
    nlosRows = trainRows(trainRows.los_flag <= 0.5, :);

    if isempty(losRows) || isempty(nlosRows)
        error("split_los_linear requires both LOS and NLOS rows in the training split.");
    end

    model = struct();
    model.los = fitlm(losRows(:, predictorBlock), ...
        'toolbox_channel_gain_db ~ log10_distance_m + relative_speed_mps + radial_velocity_mps + abs_radial_velocity_mps + heading_alignment_cos + los_probability');
    model.nlos = fitlm(nlosRows(:, predictorBlock), ...
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

function value = metricValue(metricsTable, metricName, columnName)
    idx = strcmp(string(metricsTable.metric), string(metricName));
    if ~any(idx)
        error("Metric %s not found in metrics table.", string(metricName));
    end
    value = metricsTable{find(idx, 1, 'first'), string(columnName)};
end

function plotProfileDistributionComparison(profileValues, targetValues, predictedValues, profileOrder, targetColor, predictedColor, plotTitle, yLabelText)
    hold on;

    profileValues = string(profileValues);
    targetHandle = gobjects(1, 1);
    predictedHandle = gobjects(1, 1);

    for i = 1:numel(profileOrder)
        mask = profileValues == profileOrder(i);
        if ~any(mask)
            continue;
        end

        xTarget = repmat(i - 0.18, sum(mask), 1);
        xPred = repmat(i + 0.18, sum(mask), 1);

        h1 = boxchart(xTarget, targetValues(mask), ...
            'BoxFaceColor', targetColor, ...
            'MarkerStyle', '.', ...
            'MarkerColor', targetColor);
        h2 = boxchart(xPred, predictedValues(mask), ...
            'BoxFaceColor', predictedColor, ...
            'MarkerStyle', '.', ...
            'MarkerColor', predictedColor);

        if i == 1
            targetHandle = h1;
            predictedHandle = h2;
        end
    end

    hold off;
    xlim([0.5, numel(profileOrder) + 0.5]);
    xticks(1:numel(profileOrder));
    xticklabels(profileOrder);
    ylabel(yLabelText);
    title(plotTitle);
    legend([targetHandle, predictedHandle], {'Target', 'Predicted'}, 'Location', 'southoutside');
    grid on;
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
