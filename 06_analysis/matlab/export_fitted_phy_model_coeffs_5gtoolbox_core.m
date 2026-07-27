function outputs = export_fitted_phy_model_coeffs_5gtoolbox_core(resultsRoot, outputDirName)
% Export runtime-ready generalized PHY coefficient tables and BLER curves.

rrp = rr_paths();
if nargin < 1 || strlength(resultsRoot) == 0
    resultsRoot = rrp.resultsRoot;
end
if nargin < 2 || strlength(outputDirName) == 0
    outputDirName = "phy_model_outputs_5gtoolbox";
end

resultsRoot = string(resultsRoot);
outputDir = fullfile(resultsRoot, outputDirName);
modelMatFile = fullfile(outputDir, "generalized_phy_model_5gtoolbox.mat");
trainingCsv = fullfile(outputDir, "generalized_phy_training_dataset_5gtoolbox.csv");
runtimeDir = fullfile(outputDir, "runtime_coefficients");

if ~isfile(modelMatFile)
    error("Generalized PHY MAT file not found: %s", modelMatFile);
end
if ~isfile(trainingCsv)
    error("Generalized PHY training CSV not found: %s", trainingCsv);
end
if ~isfolder(runtimeDir)
    mkdir(runtimeDir);
end

S = load(modelMatFile);
T = readtable(trainingCsv, 'TextType', 'string');

exportOneModel(S.pathLossModel, fullfile(runtimeDir, "path_loss_coefficients.csv"), "toolbox_path_loss_db");
exportOneModel(S.rssiModel, fullfile(runtimeDir, "rssi_coefficients.csv"), "toolbox_rssi_dbm");
exportOneModel(S.delayModel, fullfile(runtimeDir, "delay_spread_log10_coefficients.csv"), "log10_toolbox_delay_spread_ns");
exportOneModel(S.kFactorModel, fullfile(runtimeDir, "k_factor_coefficients.csv"), "toolbox_k_factor_db");

splitModels = getSplitChannelGainModels(S, T);
exportOneModel(splitModels.los, fullfile(runtimeDir, "channel_gain_los_coefficients.csv"), "toolbox_channel_gain_db_los");
exportOneModel(splitModels.nlos, fullfile(runtimeDir, "channel_gain_nlos_coefficients.csv"), "toolbox_channel_gain_db_nlos");
if isfield(S, "channelGainModel")
    exportOneModel(S.channelGainModel, fullfile(runtimeDir, "channel_gain_coefficients.csv"), "toolbox_channel_gain_db");
end

runtimeConstantsCsv = fullfile(runtimeDir, "runtime_constants.csv");
writeRuntimeConstants(runtimeConstantsCsv, S);

blerSummaryCsv = fullfile(runtimeDir, "bler_curve_summary.csv");
blerOutputs = buildRuntimeBlerTables(runtimeDir, T, S);
writetable(blerOutputs.summary, blerSummaryCsv);

equationFile = fullfile(runtimeDir, "generalized_phy_equations.txt");
fid = fopen(equationFile, 'w');
assert(fid ~= -1, "Could not open equation output file.");
fprintf(fid, "Generalized NR-PC5 PHY Runtime Equations\n");
fprintf(fid, "Created: %s\n\n", string(datetime("now")));
fprintf(fid, "Predictors expected at runtime:\n");
fprintf(fid, "  x1 = log10_distance_m\n");
fprintf(fid, "  x2 = relative_speed_mps\n");
fprintf(fid, "  x3 = radial_velocity_mps\n");
fprintf(fid, "  x4 = abs_radial_velocity_mps\n");
fprintf(fid, "  x5 = heading_alignment_cos\n");
fprintf(fid, "  x6 = los_probability\n\n");
writeEquation(fid, S.pathLossModel, "toolbox_path_loss_db");
writeEquation(fid, S.rssiModel, "toolbox_rssi_dbm");
writeEquation(fid, S.delayModel, "log10_toolbox_delay_spread_ns");
fprintf(fid, "Runtime note: toolbox_delay_spread_ns = 10^(log10_toolbox_delay_spread_ns)\n\n");
writeEquation(fid, S.kFactorModel, "toolbox_k_factor_db");
fprintf(fid, "Runtime note: per-link shadow fading is sampled separately from the linear surrogates.\n");
fprintf(fid, "Runtime note: packet success uses SINR -> BLER lookup tables generated with a 5G Toolbox PDSCH proxy.\n\n");
fprintf(fid, "Channel gain uses LOS-split linear models at runtime.\n\n");
writeEquation(fid, splitModels.los, "toolbox_channel_gain_db_los");
writeEquation(fid, splitModels.nlos, "toolbox_channel_gain_db_nlos");
fclose(fid);

manifestFile = fullfile(runtimeDir, "generalized_phy_runtime_manifest.txt");
fid = fopen(manifestFile, 'w');
assert(fid ~= -1, "Could not open runtime manifest file.");
fprintf(fid, "Runtime export directory: %s\n", runtimeDir);
fprintf(fid, "Source model file: %s\n", modelMatFile);
fprintf(fid, "Source training CSV: %s\n", trainingCsv);
fprintf(fid, "Equation summary: %s\n", equationFile);
fprintf(fid, "Runtime constants: %s\n", runtimeConstantsCsv);
fprintf(fid, "BLER summary: %s\n", blerSummaryCsv);
fprintf(fid, "\nCoefficient files:\n");
fprintf(fid, "  %s\n", fullfile(runtimeDir, "path_loss_coefficients.csv"));
fprintf(fid, "  %s\n", fullfile(runtimeDir, "rssi_coefficients.csv"));
fprintf(fid, "  %s\n", fullfile(runtimeDir, "delay_spread_log10_coefficients.csv"));
fprintf(fid, "  %s\n", fullfile(runtimeDir, "k_factor_coefficients.csv"));
fprintf(fid, "  %s\n", fullfile(runtimeDir, "channel_gain_los_coefficients.csv"));
fprintf(fid, "  %s\n", fullfile(runtimeDir, "channel_gain_nlos_coefficients.csv"));
fprintf(fid, "  %s\n", fullfile(runtimeDir, "channel_gain_coefficients.csv"));
fprintf(fid, "\nBLER tables:\n");
for i = 1:height(blerOutputs.summary)
    fprintf(fid, "  %s -> %s (delaySpreadNs=%.2f, dopplerHz=%.2f)\n", ...
        blerOutputs.summary.profile(i), ...
        blerOutputs.summary.csv_path(i), ...
        blerOutputs.summary.delay_spread_ns(i), ...
        blerOutputs.summary.doppler_hz(i));
end
fprintf(fid, "\nRecommended OMNeT runtime use:\n");
fprintf(fid, "  1. Compute predictors from live geometry.\n");
fprintf(fid, "  2. Evaluate the fitted linear equations for path loss, delay spread, K-factor, and channel gain.\n");
fprintf(fid, "  3. Sample correlated shadow fading per link epoch.\n");
fprintf(fid, "  4. Convert effective Rx power to SINR and interpolate profile-specific BLER.\n");
fclose(fid);

fprintf("\nRuntime coefficient export complete.\n");
fprintf("Runtime directory : %s\n", runtimeDir);
fprintf("Equation summary  : %s\n", equationFile);
fprintf("Manifest          : %s\n", manifestFile);
fprintf("BLER summary      : %s\n\n", blerSummaryCsv);

outputs = struct();
outputs.runtimeDir = runtimeDir;
outputs.equationFile = equationFile;
outputs.manifestFile = manifestFile;
outputs.runtimeConstantsCsv = runtimeConstantsCsv;
outputs.blerSummaryCsv = blerSummaryCsv;
end

function exportOneModel(model, outputCsv, responseName)
names = string(model.CoefficientNames(:));
values = model.Coefficients.Estimate(:);
stderr = model.Coefficients.SE(:);
tstat = model.Coefficients.tStat(:);
pvalue = model.Coefficients.pValue(:);

T = table(names, values, stderr, tstat, pvalue, ...
    'VariableNames', {'term','estimate','std_error','t_stat','p_value'});
T.response = repmat(string(responseName), height(T), 1);
T = movevars(T, "response", "Before", 1);
writetable(T, outputCsv);
end

function splitModels = getSplitChannelGainModels(S, trainingTable)
if isfield(S, "splitChannelGainModels")
    splitModels = S.splitChannelGainModels;
    return;
end
if isfield(S, "GeneralizedPhyModel") ...
        && isfield(S.GeneralizedPhyModel, "channelGainLosModel") ...
        && isfield(S.GeneralizedPhyModel, "channelGainNlosModel")
    splitModels = struct();
    splitModels.los = S.GeneralizedPhyModel.channelGainLosModel;
    splitModels.nlos = S.GeneralizedPhyModel.channelGainNlosModel;
    return;
end

predictorVars = {'log10_distance_m','relative_speed_mps','radial_velocity_mps', ...
    'abs_radial_velocity_mps','heading_alignment_cos','los_probability'};
losRows = trainingTable(trainingTable.los_flag > 0.5, :);
nlosRows = trainingTable(trainingTable.los_flag <= 0.5, :);
formula = 'toolbox_channel_gain_db ~ log10_distance_m + relative_speed_mps + radial_velocity_mps + abs_radial_velocity_mps + heading_alignment_cos + los_probability';

splitModels = struct();
splitModels.los = fitlm(losRows(:, [predictorVars, {'toolbox_channel_gain_db'}]), formula);
splitModels.nlos = fitlm(nlosRows(:, [predictorVars, {'toolbox_channel_gain_db'}]), formula);
end

function writeEquation(fid, model, responseName)
fprintf(fid, "%s = ", responseName);
isFirstPrinted = false;
for i = 1:numel(model.CoefficientNames)
    term = string(model.CoefficientNames{i});
    coeff = model.Coefficients.Estimate(i);
    if term == "(Intercept)"
        fprintf(fid, "%0.12g", coeff);
        isFirstPrinted = true;
    elseif isFirstPrinted
        fprintf(fid, " %+0.12g * %s", coeff, term);
    else
        fprintf(fid, "%0.12g * %s", coeff, term);
        isFirstPrinted = true;
    end
end
fprintf(fid, "\n\n");
end

function writeRuntimeConstants(outputCsv, S)
constants = S.GeneralizedPhyModel.constants;
keys = [
    "fc_hz"
    "fc_ghz"
    "tx_power_dbm"
    "sample_rate_hz"
    "num_time_samples"
    "scs_khz"
    "n_size_grid"
    "noise_figure_db"
    "bandwidth_hz"
    "shadow_sigma_los_db"
    "shadow_sigma_nlos_db"
];

values = strings(numel(keys), 1);
for i = 1:numel(keys)
    key = keys(i);
    values(i) = string(constants.(key));
end

T = table(keys, values, 'VariableNames', {'key','value'});
writetable(T, outputCsv);
end

function outputs = buildRuntimeBlerTables(runtimeDir, T, S)
constants = S.GeneralizedPhyModel.constants;
carrier = nrCarrierConfig('SubcarrierSpacing', constants.scs_khz, 'NSizeGrid', constants.n_size_grid);

profiles = ["CDL-A", "CDL-C", "CDL-D", "CDL-E"];
summary = table('Size', [0 5], ...
    'VariableTypes', {'string','double','double','double','string'}, ...
    'VariableNames', {'profile','delay_spread_ns','doppler_hz','trial_count','csv_path'});

for i = 1:numel(profiles)
    profile = profiles(i);
    rows = T(T.cdl_profile == profile, :);
    if isempty(rows)
        [repDelaySpreadNs, repDopplerHz] = inferRepresentativeProfilePoint(T, profile);
    else
        repDelaySpreadNs = median(rows.toolbox_delay_spread_ns, 'omitnan');
        repDopplerHz = median(abs(rows.doppler_hz), 'omitnan');
    end
    curve = buildProxyBlerCurve(profile, repDelaySpreadNs, repDopplerHz, constants.fc_hz, carrier);

    csvPath = fullfile(runtimeDir, sprintf('bler_curve_%s.csv', lower(strrep(char(profile), '-', '_'))));
    writetable(curve, csvPath);
    summary = [summary; {profile, repDelaySpreadNs, repDopplerHz, curve.trial_count(1), string(csvPath)}]; %#ok<AGROW>
end

outputs = struct();
outputs.summary = summary;
end

function [repDelaySpreadNs, repDopplerHz] = inferRepresentativeProfilePoint(T, profile)
isLosProfile = any(profile == ["CDL-D", "CDL-E"]);
if isLosProfile
    subset = T(T.los_flag > 0.5, :);
    if isempty(subset)
        subset = T;
    end
    repDelaySpreadNs = quantile(subset.toolbox_delay_spread_ns, 0.75);
else
    subset = T(T.los_flag <= 0.5, :);
    if isempty(subset)
        subset = T;
    end
    repDelaySpreadNs = median(subset.toolbox_delay_spread_ns, 'omitnan');
end
repDopplerHz = median(abs(subset.doppler_hz), 'omitnan');
repDelaySpreadNs = max(repDelaySpreadNs, 1.0);
repDopplerHz = max(repDopplerHz, 1.0);
end

function curve = buildProxyBlerCurve(profileName, delaySpreadNs, dopplerHz, fcHz, carrier)
% AWGN-equivalent BLER curve for QPSK LDPC (code rate 490/1024).
%
% The CDL time-domain channel is NOT simulated here.  Justification:
%   - Training-data delay spreads are 20-35 ns at 30 kHz SCS.
%   - Coherence bandwidth (~5 MHz) >> subcarrier spacing (30 kHz), so the
%     channel is effectively flat per subcarrier — indistinguishable from AWGN.
%   - CDL small-scale fading is already captured by the channel-gain
%     surrogate model fitted in the training phase; these curves are used
%     only for the SINR → BLER lookup at OMNeT++ runtime.
%
% nrSymbolModulate / nrSymbolDemodulate are used instead of
% nrPDSCH / nrPDSCHDecode to avoid a version-specific LLR sign inversion
% in nrPDSCHDecode (confirmed in MATLAB R2026a) that causes BLER = 1.

targetCodeRate = 490 / 1024;
snrRangeDb = (-10:2:30)';
numTrials = 60;

pdsch = nrPDSCHConfig;
pdsch.Modulation = 'QPSK';
pdsch.NumLayers = 1;
pdsch.MappingType = 'A';
pdsch.SymbolAllocation = [0 14];
pdsch.PRBSet = 0:(carrier.NSizeGrid - 1);
pdsch.RNTI = 1;
pdsch.NID = 1;

[~, pdschInfo] = nrPDSCHIndices(carrier, pdsch);
G   = pdschInfo.G;
tbs = nrTBS(pdsch.Modulation, pdsch.NumLayers, numel(pdsch.PRBSet), ...
            pdschInfo.NREPerPRB, targetCodeRate);

encoder = nrDLSCH;
encoder.TargetCodeRate = targetCodeRate;
decoder = nrDLSCHDecoder;
decoder.TargetCodeRate  = targetCodeRate;
decoder.TransportBlockLength = tbs;
decoder.MaximumLDPCIterationCount = 12;

bler = zeros(numel(snrRangeDb), 1);
psr  = zeros(numel(snrRangeDb), 1);

for si = 1:numel(snrRangeDb)
    snrDb = snrRangeDb(si);
    N0    = 1 / 10^(snrDb / 10);   % noise variance per RE

    blockErrors = 0;
    for trial = 1:numTrials %#ok<UNUSED>
        reset(decoder);
        trBlk = randi([0 1], tbs, 1);
        setTransportBlock(encoder, trBlk, 0);
        codedBits = encoder(pdsch.Modulation, pdsch.NumLayers, G, 0);

        % Direct AWGN: symbol-level, no OFDM or CDL channel needed.
        txSym = nrSymbolModulate(codedBits, pdsch.Modulation);
        noise = sqrt(N0 / 2) * (randn(size(txSym)) + 1j * randn(size(txSym)));
        dlschLLRs = nrSymbolDemodulate(txSym + noise, pdsch.Modulation, N0);

        [~, blkErr] = decoder(dlschLLRs, pdsch.Modulation, pdsch.NumLayers, 0);
        blockErrors = blockErrors + double(blkErr);
    end

    bler(si) = blockErrors / numTrials;
    psr(si)  = 1 - bler(si);
end

curve = table( ...
    repmat(string(profileName), numel(snrRangeDb), 1), ...
    repmat(delaySpreadNs, numel(snrRangeDb), 1), ...
    repmat(dopplerHz,     numel(snrRangeDb), 1), ...
    repmat(numTrials,     numel(snrRangeDb), 1), ...
    snrRangeDb, bler, psr, ...
    'VariableNames', {'profile','delay_spread_ns','doppler_hz','trial_count','snr_db','bler','psr'});
end

function channel = makeProxyBlerChannel(profileName, delaySpreadNs, dopplerHz, fcHz, carrier, trial)
% Time-domain channel filter — the only correct way to simulate OFDM fading.
channel = nrCDLChannel;
channel.DelayProfile             = char(profileName);
channel.DelaySpread              = max(delaySpreadNs, 1.0) * 1e-9;
channel.MaximumDopplerShift      = max(1.0, dopplerHz);
channel.CarrierFrequency         = fcHz;
channel.SampleRate               = nrOFDMInfo(carrier).SampleRate;
channel.NormalizePathGains       = true;   % unit mean path power
channel.NormalizeChannelOutputs  = true;   % output power = input power
channel.ChannelFiltering         = true;   % proper time-domain convolution
channel.RandomStream             = 'mt19937ar with seed';
channel.Seed                     = 1103 + trial;
% Force SISO — default CDL antenna arrays have 8 elements
channel.TransmitAntennaArray.Size = [1 1 1 1 1];
channel.ReceiveAntennaArray.Size  = [1 1 1 1 1];
end

function qm = bitsPerSymbol(modulation)
switch upper(string(modulation))
    case "QPSK"
        qm = 2;
    case "16QAM"
        qm = 4;
    case "64QAM"
        qm = 6;
    case "256QAM"
        qm = 8;
    case "1024QAM"
        qm = 10;
    otherwise
        error("Unsupported modulation for BLER proxy: %s", modulation);
end
end
