function outputs = build_generalized_phy_model_5gtoolbox_core(resultsRoot, outputDirName, maxFitRowsPerRun, maxRunsPerAdoption, reuseExistingTrainingCsv)
% Build the generalized NR-PC5 PHY training set and fitted surrogate models.
%
% This core function is shared by the standard and larger-sample wrappers.

rrp = rr_paths();
if nargin < 1 || strlength(resultsRoot) == 0
    resultsRoot = rrp.resultsRoot;
end
if nargin < 2 || strlength(outputDirName) == 0
    outputDirName = "phy_model_outputs_5gtoolbox";
end
if nargin < 3 || isempty(maxFitRowsPerRun)
    maxFitRowsPerRun = 2500;
end
if nargin < 4 || isempty(maxRunsPerAdoption)
    maxRunsPerAdoption = inf;
end
if nargin < 5 || isempty(reuseExistingTrainingCsv)
    reuseExistingTrainingCsv = false;
end

if exist('nrCDLChannel', 'class') ~= 8 && exist('nrCDLChannel', 'file') ~= 2
    error(['5G Toolbox does not appear to be available. This function needs ', ...
           'nrCDLChannel, nrCarrierConfig, nrOFDMInfo, nrPerfectTimingEstimate, ', ...
           'and nrPerfectChannelEstimate.']);
end

cfg = defaultConfig(resultsRoot, outputDirName, maxFitRowsPerRun, maxRunsPerAdoption, reuseExistingTrainingCsv);
prepareOutputDirectory(cfg);
if cfg.reuseExistingTrainingCsv && isfile(cfg.trainingCsv)
    fitTable = readtable(cfg.trainingCsv, 'TextType', 'string');
    manifest = inferManifestFromTrainingTable(fitTable);
    writetable(manifest, cfg.manifestCsv);
else
    runDirs = discoverRunDirs(cfg);
    if isempty(runDirs)
        error("No Ideal_PHYModel_CAV* folders found under %s", cfg.resultsRoot);
    end

    fitTable = table();
    manifest = table('Size',[0 6], ...
        'VariableTypes', {'string','double','double','string','double','double'}, ...
        'VariableNames', {'run_dir','adoption_pct','run_index','trace_file','source_row_count','sampled_row_count'});

    firstWrite = true;
    for k = 1:numel(runDirs)
        runDir = fullfile(runDirs(k).folder, runDirs(k).name);
        traceFiles = dir(fullfile(runDir, "cav_pair_trace_*.csv"));
        if isempty(traceFiles)
            warning("Skipping %s: no cav_pair_trace_*.csv found", runDir);
            continue;
        end

        tracePath = fullfile(traceFiles(1).folder, traceFiles(1).name);
        meta = parseRunMetadata(runDirs(k).name);

        T = readtable(tracePath, 'TextType', 'string');
        if isempty(T)
            warning("Skipping %s: empty trace file", tracePath);
            continue;
        end

        requiredVars = {'sender_vehicle_id','receiver_vehicle_id','sender_x_m','sender_y_m', ...
            'receiver_x_m','receiver_y_m','sender_speed_mps','receiver_speed_mps', ...
            'sender_heading_deg','receiver_heading_deg','time_s'};
        T = rmmissing(T, 'DataVariables', requiredVars);
        if isempty(T)
            warning("Skipping %s: only incomplete rows after filtering", tracePath);
            continue;
        end

        baseF = deriveGeometryRows(T, meta.adoptionPct, cfg.fcHz, cfg.fcGHz, cfg.c, cfg.txPowerDbm, cfg.hTx, cfg.hRx);
        sampleIdx = stratifiedSampleIndices(baseF, cfg.maxFitRowsPerRun, ...
            cfg.distanceEdgesM, cfg.relativeSpeedEdges, cfg.radialSpeedEdges);
        sampledF = baseF(sampleIdx, :);

        enrichedF = enrichWith5GToolbox(sampledF, cfg);
        enrichedF.run_dir = repmat(string(runDirs(k).name), height(enrichedF), 1);
        enrichedF = movevars(enrichedF, "run_dir", "Before", 1);

        if firstWrite
            writetable(enrichedF, cfg.trainingCsv);
            firstWrite = false;
        else
            writetable(enrichedF, cfg.trainingCsv, 'WriteMode', 'append', 'WriteVariableNames', false);
        end

        manifest = [manifest; {string(runDirs(k).name), meta.adoptionPct, meta.runIndex, string(tracePath), height(baseF), height(enrichedF)}]; %#ok<AGROW>
        fitTable = [fitTable; enrichedF]; %#ok<AGROW>
    end

    if isempty(fitTable)
        error("No usable training rows were built from the available PHY-model runs.");
    end

    writetable(manifest, cfg.manifestCsv);
end

predictorNames = { ...
    'log10_distance_m', ...
    'relative_speed_mps', ...
    'radial_velocity_mps', ...
    'abs_radial_velocity_mps', ...
    'heading_alignment_cos', ...
    'los_probability'};

pathLossModel = fitlm(fitTable(:, predictorNames), fitTable.toolbox_path_loss_db);
rssiModel     = fitlm(fitTable(:, predictorNames), fitTable.toolbox_rssi_dbm);
delayModel    = fitlm(fitTable(:, predictorNames), log10(fitTable.toolbox_delay_spread_ns));
kFactorModel  = fitlm(fitTable(:, predictorNames), fitTable.toolbox_k_factor_db);
channelGainModel = fitlm(fitTable(:, predictorNames), fitTable.toolbox_channel_gain_db);
splitChannelGainModels = fitSplitLosChannelGainModels(fitTable, predictorNames);

GeneralizedPhyModel = struct();
GeneralizedPhyModel.metadata = struct( ...
    'created_at', string(datetime("now")), ...
    'results_root', string(cfg.resultsRoot), ...
    'run_count', height(manifest), ...
    'fit_rows', height(fitTable), ...
    'available_adoption_pct', unique(manifest.adoption_pct)', ...
    'builder', "5G Toolbox backed generalized PHY model");

GeneralizedPhyModel.constants = struct( ...
    'fc_hz', cfg.fcHz, ...
    'fc_ghz', cfg.fcGHz, ...
    'tx_power_dbm', cfg.txPowerDbm, ...
    'tx_height_m', cfg.hTx, ...
    'rx_height_m', cfg.hRx, ...
    'sample_rate_hz', cfg.sampleRate, ...
    'num_time_samples', cfg.numTimeSamples, ...
    'scs_khz', cfg.carrier.SubcarrierSpacing, ...
    'n_size_grid', cfg.carrier.NSizeGrid, ...
    'noise_figure_db', cfg.noiseFigureDb, ...
    'bandwidth_hz', cfg.bandwidthHz, ...
    'shadow_sigma_los_db', cfg.shadowSigmaLosDb, ...
    'shadow_sigma_nlos_db', cfg.shadowSigmaNlosDb, ...
    'heading_convention', "math_from_logger: vx=speed*cosd(theta), vy=speed*sind(theta)", ...
    'large_scale_model', "3GPP_TR_38_901_UMi", ...
    'small_scale_model', "5G_Toolbox_nrCDLChannel", ...
    'runtime_shadow_model', "lognormal_per_link_epoch", ...
    'runtime_success_model', "sinr_bler_lookup");

GeneralizedPhyModel.predictors = predictorNames;
GeneralizedPhyModel.pathLossModel = compact(pathLossModel);
GeneralizedPhyModel.rssiModel = compact(rssiModel);
GeneralizedPhyModel.delaySpreadLog10Model = compact(delayModel);
GeneralizedPhyModel.kFactorModel = compact(kFactorModel);
GeneralizedPhyModel.channelGainModel = compact(channelGainModel);
GeneralizedPhyModel.channelGainLosModel = compact(splitChannelGainModels.los);
GeneralizedPhyModel.channelGainNlosModel = compact(splitChannelGainModels.nlos);

save(cfg.modelMatFile, ...
    "GeneralizedPhyModel", ...
    "pathLossModel", "rssiModel", "delayModel", "kFactorModel", "channelGainModel", ...
    "splitChannelGainModels", ...
    "-v7.3");

fitMetrics = buildFitMetrics(fitTable, predictorNames, manifest, ...
    pathLossModel, rssiModel, delayModel, kFactorModel, splitChannelGainModels);
writetable(fitMetrics, cfg.metricsCsv);

writeDiagnosticsFigure(fitTable, predictorNames, fitMetrics, ...
    pathLossModel, rssiModel, delayModel, kFactorModel, splitChannelGainModels, cfg.diagPngFile);

fprintf("\n5G Toolbox generalized PHY model build complete.\n");
fprintf("Training dataset : %s\n", cfg.trainingCsv);
fprintf("Manifest         : %s\n", cfg.manifestCsv);
fprintf("Model            : %s\n", cfg.modelMatFile);
fprintf("Fit metrics      : %s\n", cfg.metricsCsv);
fprintf("Diagnostics      : %s\n\n", cfg.diagPngFile);
disp(fitMetrics);

outputs = struct();
outputs.trainingCsv = cfg.trainingCsv;
outputs.manifestCsv = cfg.manifestCsv;
outputs.modelMatFile = cfg.modelMatFile;
outputs.metricsCsv = cfg.metricsCsv;
outputs.diagPngFile = cfg.diagPngFile;
outputs.outputDir = cfg.outputDir;
end

function cfg = defaultConfig(resultsRoot, outputDirName, maxFitRowsPerRun, maxRunsPerAdoption, reuseExistingTrainingCsv)
cfg = struct();
cfg.resultsRoot = string(resultsRoot);
cfg.outputDir = fullfile(resultsRoot, outputDirName);
cfg.trainingCsv = fullfile(cfg.outputDir, "generalized_phy_training_dataset_5gtoolbox.csv");
cfg.manifestCsv = fullfile(cfg.outputDir, "generalized_phy_manifest_5gtoolbox.csv");
cfg.modelMatFile = fullfile(cfg.outputDir, "generalized_phy_model_5gtoolbox.mat");
cfg.metricsCsv = fullfile(cfg.outputDir, "generalized_phy_fit_metrics_5gtoolbox.csv");
cfg.diagPngFile = fullfile(cfg.outputDir, "generalized_phy_diagnostics_5gtoolbox.png");

cfg.fcHz = 5.9e9;
cfg.fcGHz = cfg.fcHz / 1e9;
cfg.c = physconst("LightSpeed");
cfg.txPowerDbm = 23;
cfg.hTx = 1.5;
cfg.hRx = 1.5;
cfg.noiseFigureDb = 7.0;
cfg.bandwidthHz = 52 * 12 * 30e3;
cfg.shadowSigmaLosDb = 4.0;
cfg.shadowSigmaNlosDb = 7.82;
cfg.shadowSeed = 42017;
cfg.channelSeedBase = 73000;

cfg.carrier = nrCarrierConfig('SubcarrierSpacing', 30, 'NSizeGrid', 52);
ofdmInfo = nrOFDMInfo(cfg.carrier);
cfg.sampleRate = ofdmInfo.SampleRate;
cfg.numTimeSamples = max(256, round(0.5e-3 * cfg.sampleRate));

cfg.maxFitRowsPerRun = maxFitRowsPerRun;
cfg.maxRunsPerAdoption = maxRunsPerAdoption;
cfg.reuseExistingTrainingCsv = logical(reuseExistingTrainingCsv);
cfg.distanceEdgesM = [0 25 50 75 100 150 200 300 500 inf];
cfg.relativeSpeedEdges = [0 2 5 10 15 20 30 inf];
cfg.radialSpeedEdges = [0 1 3 6 10 15 25 inf];
cfg.targetProfiles = ["CDL-A", "CDL-C", "CDL-D", "CDL-E"];
end

function runDirs = discoverRunDirs(cfg)
runDirs = dir(fullfile(cfg.resultsRoot, "**", "Ideal_PHYModel_CAV*"));
runDirs = runDirs([runDirs.isdir]);
if isempty(runDirs)
    return;
end

meta = table('Size', [numel(runDirs) 4], ...
    'VariableTypes', {'double','double','string','double'}, ...
    'VariableNames', {'adoption_pct','run_index','folder','original_idx'});
for i = 1:numel(runDirs)
    parsed = parseRunMetadata(runDirs(i).name);
    meta.adoption_pct(i) = parsed.adoptionPct;
    meta.run_index(i) = parsed.runIndex;
    meta.folder(i) = string(fullfile(runDirs(i).folder, runDirs(i).name));
    meta.original_idx(i) = i;
end

[~, order] = sortrows(meta(:, {'run_index','adoption_pct'}), {'run_index','adoption_pct'});
meta = meta(order, :);

if isfinite(cfg.maxRunsPerAdoption)
    keep = false(height(meta), 1);
    adoptionLevels = unique(meta.adoption_pct)';
    for adoptionPct = adoptionLevels
        idx = find(meta.adoption_pct == adoptionPct);
        idx = idx(1:min(numel(idx), cfg.maxRunsPerAdoption));
        keep(idx) = true;
    end
    meta = meta(keep, :);
end

runDirs = runDirs(meta.original_idx);
end

function manifest = inferManifestFromTrainingTable(fitTable)
runNames = unique(string(fitTable.run_dir), 'stable');
manifest = table('Size',[0 6], ...
    'VariableTypes', {'string','double','double','string','double','double'}, ...
    'VariableNames', {'run_dir','adoption_pct','run_index','trace_file','source_row_count','sampled_row_count'});
for i = 1:numel(runNames)
    runName = runNames(i);
    rows = fitTable(string(fitTable.run_dir) == runName, :);
    meta = parseRunMetadata(runName);
    manifest = [manifest; {runName, meta.adoptionPct, meta.runIndex, "", height(rows), height(rows)}]; %#ok<AGROW>
end
end

function prepareOutputDirectory(cfg)
if ~isfolder(cfg.outputDir)
    mkdir(cfg.outputDir);
end

targets = [
    cfg.modelMatFile
    cfg.metricsCsv
    cfg.diagPngFile];
if ~cfg.reuseExistingTrainingCsv
    targets = [
        cfg.trainingCsv
        cfg.manifestCsv
        targets];
end
for i = 1:numel(targets)
    if isfile(targets(i))
        delete(targets(i));
    end
end
end

function meta = parseRunMetadata(runName)
tokens = regexp(runName, 'Ideal_PHYModel_CAV(\d+)-(\d+)', 'tokens', 'once');
if isempty(tokens)
    error("Could not parse adoption/run metadata from %s", runName);
end
meta.adoptionPct = str2double(tokens{1});
meta.runIndex = str2double(tokens{2});
end

function F = deriveGeometryRows(T, adoptionPct, fcHz, fcGHz, c, txPowerDbm, hTx, hRx)
sender_vx = T.sender_speed_mps .* cosd(T.sender_heading_deg);
sender_vy = T.sender_speed_mps .* sind(T.sender_heading_deg);
receiver_vx = T.receiver_speed_mps .* cosd(T.receiver_heading_deg);
receiver_vy = T.receiver_speed_mps .* sind(T.receiver_heading_deg);

dx = T.receiver_x_m - T.sender_x_m;
dy = T.receiver_y_m - T.sender_y_m;
distance_m = sqrt(dx.^2 + dy.^2);
distance_m(distance_m < 1.0) = 1.0;

d3d_m = sqrt(distance_m.^2 + (hTx - hRx).^2);
ux = dx ./ distance_m;
uy = dy ./ distance_m;

rel_vx = sender_vx - receiver_vx;
rel_vy = sender_vy - receiver_vy;
relative_speed_mps = sqrt(rel_vx.^2 + rel_vy.^2);
radial_velocity_mps = rel_vx .* ux + rel_vy .* uy;
abs_radial_velocity_mps = abs(radial_velocity_mps);
doppler_hz = (radial_velocity_mps ./ c) .* fcHz;

heading_alignment_cos = cosd(T.sender_heading_deg - T.receiver_heading_deg);
los_probability = umiLosProbability(distance_m);

path_loss_los_db = 32.4 + 21 .* log10(d3d_m) + 20 .* log10(fcGHz);
path_loss_nlos_prime_db = 35.3 .* log10(d3d_m) + 22.4 + 21.3 .* log10(fcGHz) - 0.3 .* (hRx - 1.5);
path_loss_nlos_db = max(path_loss_los_db, path_loss_nlos_prime_db);
expected_path_loss_db = (los_probability .* path_loss_los_db) + ((1 - los_probability) .* path_loss_nlos_db);
expected_rssi_dbm = txPowerDbm - expected_path_loss_db;

F = table();
F.time_s = T.time_s;
F.adoption_pct = repmat(adoptionPct, height(T), 1);
F.sender_vehicle_id = T.sender_vehicle_id;
F.receiver_vehicle_id = T.receiver_vehicle_id;
F.distance_m = distance_m;
F.log10_distance_m = log10(distance_m);
F.sender_speed_mps = T.sender_speed_mps;
F.receiver_speed_mps = T.receiver_speed_mps;
F.relative_speed_mps = relative_speed_mps;
F.radial_velocity_mps = radial_velocity_mps;
F.abs_radial_velocity_mps = abs_radial_velocity_mps;
F.sender_heading_deg = T.sender_heading_deg;
F.receiver_heading_deg = T.receiver_heading_deg;
F.heading_alignment_cos = heading_alignment_cos;
F.doppler_hz = doppler_hz;
F.los_probability = los_probability;
F.path_loss_los_db = path_loss_los_db;
F.path_loss_nlos_db = path_loss_nlos_db;
F.expected_path_loss_db = expected_path_loss_db;
F.expected_rssi_dbm = expected_rssi_dbm;
end

function F = enrichWith5GToolbox(F, cfg)
n = height(F);

F.los_flag = false(n, 1);
F.cdl_profile = strings(n, 1);
F.seed_delay_spread_ns = zeros(n, 1);
F.toolbox_delay_spread_s = zeros(n, 1);
F.toolbox_delay_spread_ns = zeros(n, 1);
F.toolbox_k_factor_db = zeros(n, 1);
F.toolbox_channel_gain_db = zeros(n, 1);
F.shadow_sigma_db = zeros(n, 1);
F.shadow_fading_db = zeros(n, 1);
F.toolbox_path_loss_db = zeros(n, 1);
F.toolbox_rssi_dbm = zeros(n, 1);
F.toolbox_path_loss_shadowed_db = zeros(n, 1);
F.toolbox_rssi_shadowed_dbm = zeros(n, 1);
F.channel_seed = zeros(n, 1, 'uint32');

shadowStream = RandStream('mt19937ar', 'Seed', cfg.shadowSeed);

for i = 1:n
    losDraw = F.los_probability(i) >= 0.5;
    F.los_flag(i) = losDraw;

    [profileName, seedDelaySpreadNs] = selectCdlProfile(F.distance_m(i), losDraw);
    F.cdl_profile(i) = profileName;
    F.seed_delay_spread_ns(i) = seedDelaySpreadNs;

    channelSeed = deriveChannelSeed(F.sender_vehicle_id(i), F.receiver_vehicle_id(i), F.time_s(i), F.adoption_pct(i), cfg.channelSeedBase);
    F.channel_seed(i) = channelSeed;

    cdl = nrCDLChannel;
    cdl.DelayProfile = char(profileName);
    cdl.DelaySpread = max(seedDelaySpreadNs, 1.0) * 1e-9;
    cdl.MaximumDopplerShift = max(1.0, abs(F.doppler_hz(i)));
    cdl.CarrierFrequency = cfg.fcHz;
    cdl.SampleRate = cfg.sampleRate;
    cdl.ChannelFiltering = false;
    cdl.NumTimeSamples = cfg.numTimeSamples;
    cdl.NormalizePathGains = false;
    cdl.NormalizeChannelOutputs = true;
    cdl.RandomStream = 'mt19937ar with seed';
    cdl.Seed = double(channelSeed);
    cdl.InitialTime = max(0.0, F.time_s(i));

    [pathGains, sampleTimes] = cdl();
    infoStruct = info(cdl);
    pathFilters = getPathFilters(cdl);

    F.toolbox_delay_spread_ns(i) = estimateDelaySpreadNs(pathGains, infoStruct.PathDelays);
    F.toolbox_delay_spread_s(i) = F.toolbox_delay_spread_ns(i) * 1e-9;
    F.toolbox_k_factor_db(i) = estimateKFactorDb(pathGains, infoStruct.ClusterTypes);
    F.toolbox_channel_gain_db(i) = estimateEffectiveChannelGainDb(cfg.carrier, pathGains, pathFilters, sampleTimes);

    if losDraw
        largeScalePathLossDb = F.path_loss_los_db(i);
        F.shadow_sigma_db(i) = cfg.shadowSigmaLosDb;
    else
        largeScalePathLossDb = F.path_loss_nlos_db(i);
        F.shadow_sigma_db(i) = cfg.shadowSigmaNlosDb;
    end
    F.shadow_fading_db(i) = F.shadow_sigma_db(i) * randn(shadowStream, 1);
    F.toolbox_path_loss_db(i) = largeScalePathLossDb;
    F.toolbox_rssi_dbm(i) = cfg.txPowerDbm - largeScalePathLossDb + F.toolbox_channel_gain_db(i);
    F.toolbox_path_loss_shadowed_db(i) = largeScalePathLossDb + F.shadow_fading_db(i);
    F.toolbox_rssi_shadowed_dbm(i) = F.toolbox_rssi_dbm(i) - F.shadow_fading_db(i);
end
end

function [profileName, delaySpreadNs] = selectCdlProfile(distanceM, isLos)
if isLos
    if distanceM < 120
        profileName = "CDL-D";
        delaySpreadNs = min(140, 30 + 0.30 * distanceM);
    else
        profileName = "CDL-E";
        delaySpreadNs = min(220, 55 + 0.40 * distanceM);
    end
else
    if distanceM < 150
        profileName = "CDL-C";
        delaySpreadNs = min(260, 90 + 0.50 * distanceM);
    else
        profileName = "CDL-A";
        delaySpreadNs = min(450, 140 + 0.75 * distanceM);
    end
end
end

function channelSeed = deriveChannelSeed(senderId, receiverId, timeS, adoptionPct, seedBase)
key = char(sprintf('%s|%s|%d|%d', senderId, receiverId, round(timeS * 1e3), round(adoptionPct)));
hashValue = fnv1aHash(key);
channelSeed = uint32(mod(double(hashValue) + double(seedBase), double(intmax('uint32') - 1)) + 1);
end

function hashValue = fnv1aHash(text)
bytes = uint8(text);
hashValue = uint64(1469598103934665603);
prime = uint64(1099511628211);
for idx = 1:numel(bytes)
    hashValue = bitxor(hashValue, uint64(bytes(idx)));
    hashValue = hashValue * prime;
end
end

function delaySpreadNs = estimateDelaySpreadNs(pathGains, pathDelaysS)
pathGainsSiso = squeeze(pathGains(:, :, 1, 1));
if isempty(pathGainsSiso)
    delaySpreadNs = 0.0;
    return;
end
pdpPower = mean(abs(pathGainsSiso).^2, 1);
pdpPower = reshape(pdpPower, 1, []);
pathDelaysS = reshape(pathDelaysS, 1, []);
normalizer = max(sum(pdpPower), eps);
pdpPower = pdpPower ./ normalizer;
meanDelayS = sum(pdpPower .* pathDelaysS);
rmsDelaySpreadS = sqrt(sum(pdpPower .* (pathDelaysS - meanDelayS).^2));
delaySpreadNs = max(0.0, rmsDelaySpreadS * 1e9);
end

function kFactorDb = estimateKFactorDb(pathGains, clusterTypes)
pathGainsSiso = squeeze(pathGains(:, :, 1, 1));
if isempty(pathGainsSiso)
    kFactorDb = -30.0;
    return;
end
clusterTypes = string(clusterTypes(:));
losIdx = find(clusterTypes == "LOS");
if isempty(losIdx)
    kFactorDb = -30.0;
    return;
end

scatterIdx = setdiff(1:size(pathGainsSiso, 2), losIdx);
losPower = mean(sum(abs(pathGainsSiso(:, losIdx)).^2, 2));
if isempty(scatterIdx)
    scatterPower = 0.0;
else
    scatterPower = mean(sum(abs(pathGainsSiso(:, scatterIdx)).^2, 2));
end

if scatterPower <= eps
    kFactorDb = 30.0;
else
    kFactorDb = 10 * log10(losPower / scatterPower + eps);
end
kFactorDb = min(30.0, max(-30.0, kFactorDb));
end

function channelGainDb = estimateEffectiveChannelGainDb(carrier, pathGains, pathFilters, sampleTimes)
timingOffset = 0;
try
    timingOffset = nrPerfectTimingEstimate(pathGains, pathFilters);
catch
end
hPerfect = nrPerfectChannelEstimate(carrier, pathGains, pathFilters, timingOffset, sampleTimes);
hSiso = squeeze(hPerfect(:, :, 1, 1));
channelGainDb = 10 * log10(median(abs(hSiso(:)).^2) + eps);
end

function models = fitSplitLosChannelGainModels(rows, predictorNames)
losRows = rows(rows.los_flag > 0.5, :);
nlosRows = rows(rows.los_flag <= 0.5, :);
if isempty(losRows) || isempty(nlosRows)
    error("Split LOS channel-gain fitting requires both LOS and NLOS rows.");
end

formula = 'toolbox_channel_gain_db ~ log10_distance_m + relative_speed_mps + radial_velocity_mps + abs_radial_velocity_mps + heading_alignment_cos + los_probability';

models = struct();
models.los = fitlm(losRows(:, [predictorNames, {'toolbox_channel_gain_db'}]), formula);
models.nlos = fitlm(nlosRows(:, [predictorNames, {'toolbox_channel_gain_db'}]), formula);
end

function pred = predictSplitLosChannelGainModels(models, rows, predictorNames)
pred = zeros(height(rows), 1);
losMask = rows.los_flag > 0.5;
if any(losMask)
    pred(losMask) = predict(models.los, rows(losMask, predictorNames));
end
if any(~losMask)
    pred(~losMask) = predict(models.nlos, rows(~losMask, predictorNames));
end
end

function metrics = buildFitMetrics(fitTable, predictorNames, manifest, ...
    pathLossModel, rssiModel, delayModel, kFactorModel, splitChannelGainModels)
metricNames = ["path_loss_db"; "rssi_dbm"; "delay_spread_ns"; "k_factor_db"; "channel_gain_db"];

predPathLoss = predict(pathLossModel, fitTable(:, predictorNames));
predRssi = predict(rssiModel, fitTable(:, predictorNames));
predDelayNs = 10 .^ predict(delayModel, fitTable(:, predictorNames));
predKFactor = predict(kFactorModel, fitTable(:, predictorNames));
predFading = predictSplitLosChannelGainModels(splitChannelGainModels, fitTable, predictorNames);

trainR2 = [
    rsquaredValue(fitTable.toolbox_path_loss_db, predPathLoss)
    rsquaredValue(fitTable.toolbox_rssi_dbm, predRssi)
    rsquaredValue(fitTable.toolbox_delay_spread_ns, predDelayNs)
    rsquaredValue(fitTable.toolbox_k_factor_db, predKFactor)
    rsquaredValue(fitTable.toolbox_channel_gain_db, predFading)];

trainRmse = [
    rmseValue(fitTable.toolbox_path_loss_db, predPathLoss)
    rmseValue(fitTable.toolbox_rssi_dbm, predRssi)
    rmseValue(fitTable.toolbox_delay_spread_ns, predDelayNs)
    rmseValue(fitTable.toolbox_k_factor_db, predKFactor)
    rmseValue(fitTable.toolbox_channel_gain_db, predFading)];

holdoutRunDirs = strings(0,1);
adoptionLevels = unique(manifest.adoption_pct)';
for adoptionPct = adoptionLevels
    adoptionRows = manifest(manifest.adoption_pct == adoptionPct, :);
    if height(adoptionRows) < 2
        continue;
    end
    [~, idx] = max(adoptionRows.run_index);
    holdoutRunDirs(end + 1, 1) = adoptionRows.run_dir(idx); %#ok<AGROW>
end

holdoutMask = ismember(fitTable.run_dir, holdoutRunDirs);
if any(holdoutMask) && any(~holdoutMask)
    trainRows = fitTable(~holdoutMask, :);
    testRows = fitTable(holdoutMask, :);

    holdoutPathLossModel = fitlm(trainRows(:, predictorNames), trainRows.toolbox_path_loss_db);
    holdoutRssiModel = fitlm(trainRows(:, predictorNames), trainRows.toolbox_rssi_dbm);
    holdoutDelayModel = fitlm(trainRows(:, predictorNames), log10(trainRows.toolbox_delay_spread_ns));
    holdoutKFactorModel = fitlm(trainRows(:, predictorNames), trainRows.toolbox_k_factor_db);
    holdoutSplitGainModels = fitSplitLosChannelGainModels(trainRows, predictorNames);

    holdoutPredPathLoss = predict(holdoutPathLossModel, testRows(:, predictorNames));
    holdoutPredRssi = predict(holdoutRssiModel, testRows(:, predictorNames));
    holdoutPredDelayNs = 10 .^ predict(holdoutDelayModel, testRows(:, predictorNames));
    holdoutPredKFactor = predict(holdoutKFactorModel, testRows(:, predictorNames));
    holdoutPredFading = predictSplitLosChannelGainModels(holdoutSplitGainModels, testRows, predictorNames);

    holdoutR2 = [
        rsquaredValue(testRows.toolbox_path_loss_db, holdoutPredPathLoss)
        rsquaredValue(testRows.toolbox_rssi_dbm, holdoutPredRssi)
        rsquaredValue(testRows.toolbox_delay_spread_ns, holdoutPredDelayNs)
        rsquaredValue(testRows.toolbox_k_factor_db, holdoutPredKFactor)
        rsquaredValue(testRows.toolbox_channel_gain_db, holdoutPredFading)];

    holdoutRmse = [
        rmseValue(testRows.toolbox_path_loss_db, holdoutPredPathLoss)
        rmseValue(testRows.toolbox_rssi_dbm, holdoutPredRssi)
        rmseValue(testRows.toolbox_delay_spread_ns, holdoutPredDelayNs)
        rmseValue(testRows.toolbox_k_factor_db, holdoutPredKFactor)
        rmseValue(testRows.toolbox_channel_gain_db, holdoutPredFading)];
else
    holdoutR2 = nan(numel(metricNames), 1);
    holdoutRmse = nan(numel(metricNames), 1);
end

metrics = table(metricNames, trainR2, trainRmse, holdoutR2, holdoutRmse, ...
    'VariableNames', {'metric','train_r2','train_rmse','holdout_r2','holdout_rmse'});
end

function writeDiagnosticsFigure(fitTable, predictorNames, fitMetrics, ...
    pathLossModel, rssiModel, delayModel, kFactorModel, splitChannelGainModels, outputPng)
predPathLoss = predict(pathLossModel, fitTable(:, predictorNames));
predRssi = predict(rssiModel, fitTable(:, predictorNames));
predDelayNs = 10 .^ predict(delayModel, fitTable(:, predictorNames));
predKFactor = predict(kFactorModel, fitTable(:, predictorNames));
predFading = predictSplitLosChannelGainModels(splitChannelGainModels, fitTable, predictorNames);

profileCats = categorical(fitTable.cdl_profile);
profileOrder = string(categories(profileCats));

fig = figure('Visible','off', 'Position', [100 100 1550 1000]);
tiledlayout(2,4, 'Padding','compact', 'TileSpacing','compact');

nexttile;
scatter(fitTable.distance_m, fitTable.toolbox_rssi_shadowed_dbm, 4, fitTable.adoption_pct, 'filled');
xlabel('Distance (m)');
ylabel('Received Power (dBm)');
title('Toolbox-derived received power targets');
grid on;
colorbar;

nexttile;
scatter(fitTable.toolbox_path_loss_db, predPathLoss, 4, fitTable.adoption_pct, 'filled');
xlabel('Target path loss (dB)');
ylabel('Predicted path loss (dB)');
title(sprintf('Path-loss fit (R^2=%.3f)', metricValue(fitMetrics, "path_loss_db", "train_r2")));
grid on;
refline(1,0);

nexttile;
scatter(fitTable.toolbox_rssi_dbm, predRssi, 4, fitTable.adoption_pct, 'filled');
xlabel('Target mean RSSI (dBm)');
ylabel('Predicted mean RSSI (dBm)');
title(sprintf('RSSI fit (R^2=%.3f)', metricValue(fitMetrics, "rssi_dbm", "train_r2")));
grid on;
refline(1,0);

nexttile;
scatter(fitTable.toolbox_delay_spread_ns, predDelayNs, 4, fitTable.adoption_pct, 'filled');
xlabel('Target delay spread (ns)');
ylabel('Predicted delay spread (ns)');
title(sprintf('Delay-spread fit (R^2=%.3f)', metricValue(fitMetrics, "delay_spread_ns", "train_r2")));
grid on;
refline(1,0);

nexttile;
boxchart(profileCats, fitTable.toolbox_k_factor_db);
hold on;
plotPredictedProfileMeans(profileOrder, profileCats, predKFactor, [0.90 0.45 0.05]);
hold off;
xlabel('CDL profile');
ylabel('K-factor (dB)');
title(sprintf('K-factor by profile (R^2=%.3f)', metricValue(fitMetrics, "k_factor_db", "train_r2")));
grid on;

nexttile;
boxchart(profileCats, fitTable.toolbox_channel_gain_db);
hold on;
plotPredictedProfileMeans(profileOrder, profileCats, predFading, [0.13 0.45 0.85]);
hold off;
xlabel('CDL profile');
ylabel('Channel gain (dB)');
title(sprintf('Channel gain by profile (R^2=%.3f)', metricValue(fitMetrics, "channel_gain_db", "holdout_r2")));
grid on;

nexttile;
boxchart(categorical(fitTable.los_flag, [0 1], ["NLOS","LOS"]), fitTable.shadow_fading_db);
xlabel('Link state');
ylabel('Shadow fading (dB)');
title('Shadow-fading samples by link state');
grid on;

nexttile;
bar(categorical(fitMetrics.metric), [fitMetrics.train_r2 fitMetrics.holdout_r2]);
ylabel('R^2');
title('Train vs holdout R^2');
legend('Train', 'Holdout', 'Location', 'southoutside');
grid on;

sgtitle('Generalized PHY diagnostics', 'FontWeight', 'bold');
applyLightTheme(fig);
exportgraphics(fig, outputPng, 'Resolution', 180);
close(fig);
end

function plotPredictedProfileMeans(profileOrder, profileValues, predictedValues, colorValue)
profileValues = categorical(string(profileValues), profileOrder);
for i = 1:numel(profileOrder)
    mask = profileValues == profileOrder(i);
    if ~any(mask)
        continue;
    end
    meanValue = mean(predictedValues(mask));
    xCenter = i;
    plot([xCenter - 0.20, xCenter + 0.20], [meanValue, meanValue], '-', 'Color', colorValue, 'LineWidth', 2.0);
end
end

function sampleIdx = stratifiedSampleIndices(F, maxRows, distanceEdgesM, relativeSpeedEdges, radialSpeedEdges)
if height(F) <= maxRows
    sampleIdx = (1:height(F))';
    return;
end

distBin = discretize(F.distance_m, distanceEdgesM);
relBin = discretize(F.relative_speed_mps, relativeSpeedEdges);
radBin = discretize(F.abs_radial_velocity_mps, radialSpeedEdges);

key = strcat(string(distBin), "_", string(relBin), "_", string(radBin));
[groups, ~] = findgroups(key);
counts = splitapply(@numel, groups, groups);

nGroups = numel(counts);
baseQuota = max(1, floor(maxRows / nGroups));
picks = false(height(F), 1);

for g = 1:nGroups
    idx = find(groups == g);
    quota = min(numel(idx), baseQuota);
    if quota > 0
        picked = idx(randperm(numel(idx), quota));
        picks(picked) = true;
    end
end

sampleIdx = find(picks);

if numel(sampleIdx) < maxRows
    remaining = setdiff((1:height(F))', sampleIdx);
    needed = min(maxRows - numel(sampleIdx), numel(remaining));
    if needed > 0
        addIdx = remaining(randperm(numel(remaining), needed));
        sampleIdx = [sampleIdx; addIdx];
    end
elseif numel(sampleIdx) > maxRows
    sampleIdx = sampleIdx(randperm(numel(sampleIdx), maxRows));
end

sampleIdx = sort(sampleIdx);
end

function pLos = umiLosProbability(distanceMeters)
d = max(distanceMeters, 1.0);
base = min(18 ./ d, 1.0) .* (1 - exp(-d ./ 36)) + exp(-d ./ 36);
pLos = max(0, min(1, base));
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
idx = find(metricsTable.metric == metricName, 1, 'first');
if isempty(idx)
    value = NaN;
else
    value = metricsTable{idx, columnName};
end
end

function applyLightTheme(fig)
    set(fig, 'Color', 'white');
    axList = findall(fig, 'type', 'axes');
    for k = 1:numel(axList)
        ax = axList(k);
        set(ax, 'Color',      'white', ...
                'XColor',     [0.15 0.15 0.15], ...
                'YColor',     [0.15 0.15 0.15], ...
                'GridColor',  [0.80 0.80 0.80], ...
                'GridAlpha',  0.9, ...
                'FontSize',   9, ...
                'FontName',   'Helvetica');
        if ~isempty(ax.Title),  ax.Title.Color  = [0 0 0]; end
        if ~isempty(ax.XLabel), ax.XLabel.Color = [0 0 0]; end
        if ~isempty(ax.YLabel), ax.YLabel.Color = [0 0 0]; end
    end
    legList = findall(fig, 'type', 'legend');
    for k = 1:numel(legList)
        set(legList(k), 'Color', 'white', 'TextColor', [0 0 0], ...
            'EdgeColor', [0.7 0.7 0.7]);
    end
    cbList = findall(fig, 'type', 'colorbar');
    for k = 1:numel(cbList)
        set(cbList(k), 'Color', [0.15 0.15 0.15]);
    end
    txtList = findall(fig, 'type', 'text');
    for k = 1:numel(txtList)
        if isequal(get(txtList(k), 'Color'), [1 1 1])
            set(txtList(k), 'Color', [0 0 0]);
        end
    end
end
