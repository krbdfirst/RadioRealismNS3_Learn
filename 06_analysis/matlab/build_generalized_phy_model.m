%% build_generalized_phy_model.m
% Generalized NR-PC5 PHY model builder for CAV adoption sweeps.
%
% Required MATLAB products:
%   - MATLAB (base)
%   - Statistics and Machine Learning Toolbox
%
% Optional:
%   - Parallel Computing Toolbox (not required here)
%   - 5G Toolbox (not required for this generalized model fit; useful later
%     if you want explicit CDL waveform/channel realizations)
%
% Inputs:
%   - results/inet/Ideal_PHYModel_CAV*/cav_pair_trace_*.csv
%
% Outputs:
%   - results/inet/phy_model_outputs/generalized_phy_training_dataset.csv
%   - results/inet/phy_model_outputs/generalized_phy_model.mat
%   - results/inet/phy_model_outputs/generalized_phy_manifest.csv
%   - results/inet/phy_model_outputs/generalized_phy_diagnostics.png
%
% Purpose:
%   Build a seed-transferable geometry-to-PHY model from multiple adoption
%   levels and seeds. This does NOT replay exact pair traces. Instead, it
%   learns compact regression surfaces for expected PHY quantities from the
%   geometry that appears across many runs.

clear;
clc;

%% Configuration
rrp = rr_paths();
resultsRoot = rrp.resultsRoot;
outputDir   = fullfile(resultsRoot, "phy_model_outputs");
if ~isfolder(outputDir)
    mkdir(outputDir);
end

trainingCsv   = fullfile(outputDir, "generalized_phy_training_dataset.csv");
manifestCsv   = fullfile(outputDir, "generalized_phy_manifest.csv");
modelMatFile  = fullfile(outputDir, "generalized_phy_model.mat");
diagPngFile   = fullfile(outputDir, "generalized_phy_diagnostics.png");

if isfile(trainingCsv), delete(trainingCsv); end
if isfile(manifestCsv), delete(manifestCsv); end
if isfile(modelMatFile), delete(modelMatFile); end
if isfile(diagPngFile), delete(diagPngFile); end

% Carrier / PHY assumptions
fcHz       = 5.9e9;
fcGHz      = fcHz / 1e9;
c          = physconst("LightSpeed");
txPowerDbm = 23;
hTx        = 1.5;
hRx        = 1.5;

% Fitting controls
maxFitRowsPerRun = 15000;
rng(42);

%% Discover available PHY-model runs
runDirs = dir(fullfile(resultsRoot, "Ideal_PHYModel_CAV*"));
runDirs = runDirs([runDirs.isdir]);

if isempty(runDirs)
    error("No Ideal_PHYModel_CAV* folders found under %s", resultsRoot);
end

fitTable = table();
manifest = table('Size',[0 5], ...
    'VariableTypes', {'string','double','double','string','double'}, ...
    'VariableNames', {'run_dir','adoption_pct','run_index','trace_file','row_count'});

firstWrite = true;

%% Ingest all available runs
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

    F = deriveGeneralizedPhyRows(T, meta.adoptionPct, fcHz, fcGHz, c, txPowerDbm, hTx, hRx);
    F.run_dir = repmat(string(runDirs(k).name), height(F), 1);
    F = movevars(F, "run_dir", "Before", 1);

    if firstWrite
        writetable(F, trainingCsv);
        firstWrite = false;
    else
        writetable(F, trainingCsv, 'WriteMode', 'append', 'WriteVariableNames', false);
    end

    manifest = [manifest; {string(runDirs(k).name), meta.adoptionPct, meta.runIndex, string(tracePath), height(F)}]; %#ok<AGROW>

    sampleCount = min(maxFitRowsPerRun, height(F));
    sampleIdx = randperm(height(F), sampleCount);
    fitTable = [fitTable; F(sampleIdx,:)]; %#ok<AGROW>
end

if isempty(fitTable)
    error("No usable training rows were built from the available PHY-model runs.");
end

writetable(manifest, manifestCsv);

%% Fit compact generalized models
predictorNames = { ...
    'log10_distance_m', ...
    'relative_speed_mps', ...
    'radial_velocity_mps', ...
    'abs_radial_velocity_mps', ...
    'heading_alignment_cos', ...
    'adoption_pct', ...
    'los_probability'};

pathLossModel = fitlm(fitTable(:, predictorNames), fitTable.expected_path_loss_db);
rssiModel     = fitlm(fitTable(:, predictorNames), fitTable.expected_rssi_dbm);
delayModel    = fitlm(fitTable(:, predictorNames), log10(fitTable.expected_delay_spread_ns));
kFactorModel  = fitlm(fitTable(:, predictorNames), fitTable.expected_k_factor_db);

GeneralizedPhyModel = struct();
GeneralizedPhyModel.metadata = struct( ...
    'created_at', string(datetime("now")), ...
    'results_root', string(resultsRoot), ...
    'run_count', height(manifest), ...
    'fit_rows', height(fitTable), ...
    'available_adoption_pct', unique(manifest.adoption_pct)');

GeneralizedPhyModel.constants = struct( ...
    'fc_hz', fcHz, ...
    'fc_ghz', fcGHz, ...
    'tx_power_dbm', txPowerDbm, ...
    'tx_height_m', hTx, ...
    'rx_height_m', hRx, ...
    'heading_convention', "math_from_logger: vx=speed*cosd(theta), vy=speed*sind(theta)", ...
    'los_model', "3GPP_TR_38_901_UMi", ...
    'notes', "Expected PHY quantities are derived from multi-run geometry, then compressed into regression models.");

GeneralizedPhyModel.predictors = predictorNames;
GeneralizedPhyModel.pathLossModel = compact(pathLossModel);
GeneralizedPhyModel.rssiModel = compact(rssiModel);
GeneralizedPhyModel.delaySpreadLog10Model = compact(delayModel);
GeneralizedPhyModel.kFactorModel = compact(kFactorModel);

save(modelMatFile, "GeneralizedPhyModel", "pathLossModel", "rssiModel", "delayModel", "kFactorModel", "-v7.3");

%% Diagnostics
predPathLoss = predict(pathLossModel, fitTable(:, predictorNames));
predRssi     = predict(rssiModel, fitTable(:, predictorNames));
predDelayNs  = 10 .^ predict(delayModel, fitTable(:, predictorNames));

fig = figure('Visible','off', 'Position', [100 100 1400 900]);
tiledlayout(2,2, 'Padding','compact', 'TileSpacing','compact');

nexttile;
scatter(fitTable.distance_m, fitTable.expected_rssi_dbm, 4, fitTable.adoption_pct, 'filled');
xlabel('Distance (m)');
ylabel('Expected RSSI (dBm)');
title('Training Targets Across Adoption Levels');
grid on;
colorbar;

nexttile;
scatter(fitTable.expected_path_loss_db, predPathLoss, 4, fitTable.adoption_pct, 'filled');
xlabel('Target Path Loss (dB)');
ylabel('Predicted Path Loss (dB)');
title(sprintf('Path-Loss Fit (R^2=%.3f)', pathLossModel.Rsquared.Ordinary));
grid on;
refline(1,0);

nexttile;
scatter(fitTable.expected_rssi_dbm, predRssi, 4, fitTable.adoption_pct, 'filled');
xlabel('Target RSSI (dBm)');
ylabel('Predicted RSSI (dBm)');
title(sprintf('RSSI Fit (R^2=%.3f)', rssiModel.Rsquared.Ordinary));
grid on;
refline(1,0);

nexttile;
scatter(fitTable.expected_delay_spread_ns, predDelayNs, 4, fitTable.adoption_pct, 'filled');
xlabel('Target Delay Spread (ns)');
ylabel('Predicted Delay Spread (ns)');
title(sprintf('Delay-Spread Fit (R^2=%.3f)', delayModel.Rsquared.Ordinary));
grid on;
refline(1,0);

exportgraphics(fig, diagPngFile, 'Resolution', 180);
close(fig);

fprintf("\nGeneralized PHY model build complete.\n");
fprintf("Training dataset : %s\n", trainingCsv);
fprintf("Manifest         : %s\n", manifestCsv);
fprintf("Model            : %s\n", modelMatFile);
fprintf("Diagnostics      : %s\n\n", diagPngFile);

%% Local functions
function meta = parseRunMetadata(runName)
    tokens = regexp(runName, 'Ideal_PHYModel_CAV(\d+)-(\d+)', 'tokens', 'once');
    if isempty(tokens)
        error("Could not parse adoption/run metadata from %s", runName);
    end
    meta.adoptionPct = str2double(tokens{1});
    meta.runIndex = str2double(tokens{2});
end

function F = deriveGeneralizedPhyRows(T, adoptionPct, fcHz, fcGHz, c, txPowerDbm, hTx, hRx)
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

    expected_k_factor_db = 9 .* los_probability;

    delay_los_ns = 45 + 0.18 .* distance_m;
    delay_nlos_ns = 120 + 0.65 .* distance_m;
    expected_delay_spread_ns = (los_probability .* delay_los_ns) + ((1 - los_probability) .* delay_nlos_ns);
    expected_delay_spread_ns = min(max(expected_delay_spread_ns, 35), 600);
    expected_delay_spread_s = expected_delay_spread_ns .* 1e-9;

    % Geometry-only expected RSSI mean. We intentionally do not inject
    % realization-specific fading here because the goal is a seed-transferable model.
    expected_rssi_dbm = txPowerDbm - expected_path_loss_db;

    F = table();
    F.time_s = T.time_s;
    F.adoption_pct = repmat(adoptionPct, height(T), 1);
    F.sender_vehicle_id = T.sender_vehicle_id;
    F.receiver_vehicle_id = T.receiver_vehicle_id;
    F.sender_x_m = T.sender_x_m;
    F.sender_y_m = T.sender_y_m;
    F.receiver_x_m = T.receiver_x_m;
    F.receiver_y_m = T.receiver_y_m;
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
    F.expected_delay_spread_s = expected_delay_spread_s;
    F.expected_delay_spread_ns = expected_delay_spread_ns;
    F.expected_k_factor_db = expected_k_factor_db;
end

function p = umiLosProbability(d)
    d = max(d, 1.0);
    p = ones(size(d));
    idx = d > 18;
    p(idx) = (18 ./ d(idx)) + exp(-d(idx) ./ 36) .* (1 - 18 ./ d(idx));
    p = max(0.0, min(1.0, p));
end
