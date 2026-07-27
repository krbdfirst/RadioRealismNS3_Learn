%% diagnose_and_refit_channel_gain.m
%
% Does three things:
%   1. Plots actual vs predicted for channel gain, K-factor, and shadow fading
%      so you can see exactly where the OLS fit breaks down.
%   2. Fits per-profile Rician/Rayleigh distribution parameters for channel
%      gain — replacing the OLS with a physically correct stochastic model.
%   3. Exports the distribution parameters to runtime_coefficients/ so the
%      C++ runtime can draw from the correct distribution instead of
%      evaluating a bad OLS equation.

clear; clc;

rrp = rr_paths();
trainingCsv  = fullfile(rrp.phyOutDir, "generalized_phy_training_dataset_5gtoolbox.csv");
runtimeDir   = rrp.runtimeDir;
outPng       = fullfile(rrp.phyOutDir, "channel_gain_diagnostics.png");
distParamCsv = fullfile(runtimeDir, "channel_gain_distribution_params.csv");

assert(isfile(trainingCsv), "Training CSV not found: %s", trainingCsv);

T = readtable(trainingCsv, 'TextType', 'string');
fprintf("Loaded %d training rows.\n", height(T));

profiles = ["CDL-A", "CDL-C", "CDL-D", "CDL-E"];
profileColors = lines(numel(profiles));

%% ── Reload OLS models for overlay ──────────────────────────────────────────
modelMat = fullfile(rrp.phyOutDir, "generalized_phy_model_5gtoolbox.mat");
assert(isfile(modelMat), "Model .mat not found: run the pipeline first.");
S = load(modelMat);

predictorNames = {'log10_distance_m','relative_speed_mps','radial_velocity_mps', ...
    'abs_radial_velocity_mps','heading_alignment_cos','los_probability'};

% Per-profile OLS channel gain predictions (split model)
splitGainPred = zeros(height(T), 1);
losMask = T.los_flag > 0.5;
if isfield(S, 'splitChannelGainModels') && ~isempty(S.splitChannelGainModels)
    if any(losMask)
        splitGainPred(losMask)  = predict(S.splitChannelGainModels.los,  T(losMask,  predictorNames));
    end
    if any(~losMask)
        splitGainPred(~losMask) = predict(S.splitChannelGainModels.nlos, T(~losMask, predictorNames));
    end
end

kPred     = predict(S.kFactorModel,  T(:, predictorNames));
delayPred = 10 .^ predict(S.delayModel, T(:, predictorNames));

%% ── Figure 1: Channel Gain ──────────────────────────────────────────────────
fig = figure('Visible','off','Position',[50 50 1600 1200]);
tiledlayout(3, 4, 'Padding','compact','TileSpacing','compact');

% Row 1: actual channel gain vs distance, one tile per profile
for pi = 1:numel(profiles)
    pName = profiles(pi);
    mask  = string(T.cdl_profile) == pName;
    if ~any(mask), continue; end

    subDist = T.distance_m(mask);
    subAct  = T.toolbox_channel_gain_db(mask);
    subPred = splitGainPred(mask);

    % Sort by distance for the prediction line
    [sortDist, sIdx] = sort(subDist);
    sortPred = subPred(sIdx);

    nexttile;
    scatter(subDist, subAct, 6, profileColors(pi,:), 'filled', 'MarkerFaceAlpha', 0.35);
    hold on;
    plot(sortDist, sortPred, 'k-', 'LineWidth', 1.5);
    xlabel('Distance (m)');
    ylabel('Channel gain (dB)');
    title(sprintf('%s  — OLS R²=%.2f', pName, rsq(subAct, subPred)));
    legend('Actual','OLS fit','Location','best','FontSize',7);
    grid on;
end

% Row 2: K-factor actual vs predicted (all profiles combined, colored by profile)
nexttile([1 2]);
hold on;
for pi = 1:numel(profiles)
    pName = profiles(pi);
    mask  = string(T.cdl_profile) == pName;
    if ~any(mask), continue; end
    scatter(T.distance_m(mask), T.toolbox_k_factor_db(mask), 6, profileColors(pi,:), 'filled', 'MarkerFaceAlpha',0.35, 'DisplayName', char(pName) + " actual");
end
% OLS prediction line (median other features, sweep distance)
dGrid = linspace(1, max(T.distance_m), 200)';
medRow = table(log10(dGrid), ...
    repmat(median(T.relative_speed_mps),     200,1), ...
    repmat(median(T.radial_velocity_mps),    200,1), ...
    repmat(median(T.abs_radial_velocity_mps),200,1), ...
    repmat(median(T.heading_alignment_cos),  200,1), ...
    repmat(median(T.los_probability),        200,1), ...
    'VariableNames', predictorNames);
kLine = predict(S.kFactorModel, medRow);
plot(dGrid, kLine, 'k-', 'LineWidth',2, 'DisplayName','OLS (median features)');
xlabel('Distance (m)');  ylabel('K-factor (dB)');
title(sprintf('K-factor — OLS R²=%.2f  RMSE=%.1f dB  (clamped to ±0.75 dB at runtime)', ...
    S.kFactorModel.Rsquared.Ordinary, sqrt(mean((T.toolbox_k_factor_db - kPred).^2))));
legend('Location','best','FontSize',7);
grid on;

% K-factor residuals
nexttile([1 2]);
histogram(T.toolbox_k_factor_db - kPred, 50, 'FaceColor',[0.3 0.6 0.9]);
xlabel('Residual (dB)');  ylabel('Count');
title('K-factor OLS residuals  (should be ≈ Normal)');
grid on;

% Row 3: Shadow fading distribution check
nexttile([1 2]);
histogram(T.shadow_fading_db(losMask),  40, 'FaceColor',[0.2 0.7 0.3], 'FaceAlpha',0.6, ...
    'DisplayName', sprintf('LOS (σ=%.1f dB)', std(T.shadow_fading_db(losMask))));
hold on;
histogram(T.shadow_fading_db(~losMask), 40, 'FaceColor',[0.8 0.3 0.3], 'FaceAlpha',0.6, ...
    'DisplayName', sprintf('NLOS (σ=%.1f dB)', std(T.shadow_fading_db(~losMask))));
legend;  xlabel('Shadow fading draw (dB)');  ylabel('Count');
title('Shadow fading distribution — this is a random draw, not predicted by OLS');
grid on;

% Delay spread actual vs predicted
nexttile([1 2]);
scatter(T.distance_m, T.toolbox_delay_spread_ns, 6, T.los_flag, 'filled', 'MarkerFaceAlpha',0.4);
hold on;
[sortD, sIdx2] = sort(T.distance_m);
plot(sortD, delayPred(sIdx2), 'k.', 'MarkerSize', 1);
xlabel('Distance (m)');  ylabel('Delay spread (ns)');
colorbar('Ticks',[0 1],'TickLabels',{'NLOS','LOS'});
title(sprintf('Delay spread — OLS R²=%.2f  (minor impact on PDR)', ...
    rsquaredVal(T.toolbox_delay_spread_ns, delayPred)));
grid on;

applyLightTheme(fig);
exportgraphics(fig, outPng, 'Resolution', 180);
close(fig);
fprintf("Diagnostic figure saved: %s\n", outPng);

%% ── Fit per-profile channel gain distribution parameters ───────────────────
%
% Model: distance-conditioned location-scale distribution
%
%   Step 1 — fit mean power vs distance (log-log linear, i.e. power law):
%       log10( Ω(d) ) = a_p + b_p · log10(d)
%     This is OLS in log-log space; it captures the geometry-dependent part.
%
%   Step 2 — the SHAPE of the distribution stays per-profile:
%     LOS  (CDL-D/E): Rician  — K fixed from global moment estimate
%     NLOS (CDL-A/C): Rayleigh — exponential in linear power
%
%   At runtime: compute Ω(d) from the power law, then draw from
%   Rician(K, Ω(d))  or  Exponential(Ω(d)).
%
%   Why not use a fancier regressor (XGBoost, neural net)?
%     No model can beat R²≈0.35 for instantaneous channel gain because
%     small-scale fading is stochastic at the wavelength scale (λ=5 cm).
%     The stochastic floor is ~65–70% of the total variance — irreducible
%     from geometry. Fitting Ω(d) captures the distance trend correctly;
%     drawing from the distribution captures the stochastic component
%     correctly.  That is the physically correct decomposition.

fprintf("\nFitting per-profile channel gain distribution parameters ...\n");

distRows = table();
for pi = 1:numel(profiles)
    pName = profiles(pi);
    mask  = string(T.cdl_profile) == pName;
    gainDb   = T.toolbox_channel_gain_db(mask);
    distM    = T.distance_m(mask);

    if isempty(gainDb)
        warning("No rows for profile %s — skipping.", pName);
        continue;
    end

    gainLin  = 10 .^ (gainDb / 10);   % instantaneous linear power
    muLin    = mean(gainLin);          % global mean (fallback)

    % ── Step 1: distance-conditioned mean power Ω(d) ───────────────────
    % Fit log10(omega_i) = a + b·log10(d_i)  via ordinary least squares.
    % Clamp distance to ≥1 m to avoid log(0).
    log10D    = log10(max(distM, 1));
    log10Gain = log10(gainLin + eps);
    X         = [ones(numel(log10D), 1), log10D];
    beta      = X \ log10Gain;          % [intercept; slope]
    omegaLog10Intercept = beta(1);
    omegaLog10Slope     = beta(2);

    % Predicted mean power at each distance
    omegaPred = 10 .^ (omegaLog10Intercept + omegaLog10Slope * log10D);

    % R² of the log10-domain mean-power fit
    r2Omega = rsq(log10Gain, omegaLog10Intercept + omegaLog10Slope * log10D);

    % ── Step 2: fit distribution shape from distance-normalised residuals ──
    % Divide each sample by its predicted mean power so the residuals are
    % dimensionless (mean ≈ 1).  K is a property of the CDL profile, not
    % distance, so fitting it on residuals is cleaner.
    normLin = gainLin ./ max(omegaPred, eps);

    isLosProf = any(pName == ["CDL-D", "CDL-E"]);
    if isLosProf
        % Rician moment estimators on normalised power
        muN   = mean(normLin);
        varN  = var(normLin);
        sig2  = (muN - sqrt(max(muN^2 - varN, 0))) / 2;
        sig2  = max(sig2, eps);
        v2    = max(muN - 2*sig2, 0);
        kLin  = v2 / (2 * sig2);
        kDb   = 10 * log10(max(kLin, eps));

        meanGainDb = 10 * log10(muLin + eps);
        stdGainDb  = std(gainDb);

        fprintf("  %-6s  Rician  K=%.1f dB  Om(d): 10^(%.3f + %.3f*log10(d))  R2_Om=%.3f  n=%d\n", ...
            pName, kDb, omegaLog10Intercept, omegaLog10Slope, r2Omega, numel(gainDb));
        distType = "rician";
    else
        kDb      = -Inf;
        meanGainDb = 10 * log10(muLin + eps);
        stdGainDb  = std(gainDb);
        distType = "rayleigh";

        fprintf("  %-6s  Rayleigh  Om(d): 10^(%.3f + %.3f*log10(d))  R2_Om=%.3f  n=%d\n", ...
            pName, omegaLog10Intercept, omegaLog10Slope, r2Omega, numel(gainDb));
    end

    distRows = [distRows; table( ...
        string(pName), distType, ...
        kDb, meanGainDb, stdGainDb, muLin, sqrt(var(gainLin)), numel(gainDb), ...
        omegaLog10Intercept, omegaLog10Slope, r2Omega, ...
        'VariableNames', { ...
            'profile','dist_type','rician_k_db','mean_gain_db','std_gain_db', ...
            'mean_lin','std_lin','n_samples', ...
            'omega_log10_intercept','omega_log10_slope','r2_omega_fit'})]; %#ok<AGROW>
end

writetable(distRows, distParamCsv);
fprintf("\nDistribution parameters saved: %s\n", distParamCsv);
disp(distRows(:, {'profile','dist_type','rician_k_db','mean_gain_db', ...
                  'omega_log10_intercept','omega_log10_slope','r2_omega_fit','n_samples'}));

%% ── Figure 2: distance-conditioned Ω(d) fit + histogram overlay ────────────
fig2 = figure('Visible','off','Position',[50 50 1600 700]);
tiledlayout(2, numel(profiles), 'Padding','compact','TileSpacing','compact');

validProfiles = distRows.profile;

% ── Row 1: log10(Ω) vs log10(d) scatter + fitted power law ──────────────
for pi = 1:numel(profiles)
    pName = profiles(pi);
    mask  = string(T.cdl_profile) == pName;
    gainDb = T.toolbox_channel_gain_db(mask);
    if isempty(gainDb), continue; end

    profRow = distRows(string(distRows.profile) == pName, :);
    if isempty(profRow), continue; end

    distM    = T.distance_m(mask);
    gainLin  = 10.^(gainDb/10);
    log10D   = log10(max(distM, 1));
    log10G   = log10(gainLin + eps);

    dGrid    = linspace(min(log10D), max(log10D), 200);
    omegaLine = profRow.omega_log10_intercept + profRow.omega_log10_slope * dGrid;

    nexttile;
    scatter(log10D, log10G, 4, profileColors(pi,:), 'filled', 'MarkerFaceAlpha', 0.25);
    hold on;
    plot(dGrid, omegaLine, 'k-', 'LineWidth', 2);
    xlabel('log_{10}(d)  [m]');  ylabel('log_{10}(Ω)  [lin power]');
    title(sprintf('%s — Ω(d) fit  R²=%.3f', pName, profRow.r2_omega_fit));
    legend('Toolbox samples', sprintf('10^{%.2f%+.2f·log_{10}(d)}', ...
        profRow.omega_log10_intercept, profRow.omega_log10_slope), ...
        'Location','best','FontSize',7);
    grid on;
end

% ── Row 2: per-profile gain histogram with Rician / Rayleigh overlay ─────
for pi = 1:numel(profiles)
    pName = profiles(pi);
    mask  = string(T.cdl_profile) == pName;
    gainDb = T.toolbox_channel_gain_db(mask);
    if isempty(gainDb), continue; end

    profRow = distRows(string(distRows.profile) == pName, :);
    isLosProf = any(pName == ["CDL-D","CDL-E"]);

    nexttile;
    histogram(gainDb, 40, 'Normalization','pdf', 'FaceColor', profileColors(pi,:), 'FaceAlpha',0.6);
    hold on;

    xr   = linspace(min(gainDb)-2, max(gainDb)+2, 300);
    xLin = 10.^(xr/10);
    dxDgain = xLin .* log(10)/10;   % dB→linear Jacobian

    % Overlay using the global mean power (mean of Ω over the dataset)
    omega = profRow.mean_lin;

    if isLosProf && ~isempty(profRow)
        kLin = 10^(profRow.rician_k_db/10);
        ricPdf = (1+kLin)/omega * exp(-kLin - (1+kLin)*xLin/omega) .* ...
            besseli(0, 2*sqrt(kLin*(1+kLin)*xLin/omega)) .* dxDgain;
        ricPdf(~isfinite(ricPdf)) = 0;
        plot(xr, ricPdf, 'r-', 'LineWidth', 2, 'DisplayName', ...
            sprintf('Rician K=%.1f dB', profRow.rician_k_db));
    else
        % Rayleigh / exponential power in dB
        rayPdf = (1/omega) * exp(-xLin/omega) .* dxDgain;
        rayPdf(~isfinite(rayPdf)) = 0;
        plot(xr, rayPdf, 'r-', 'LineWidth', 2, 'DisplayName', 'Rayleigh (Exp power)');
    end

    pd = fitdist(gainDb, 'Normal');
    plot(xr, pdf(pd, xr), 'k--', 'LineWidth', 1.5, 'DisplayName', ...
        sprintf('Normal fit μ=%.1f', pd.mu));

    xlabel('Channel gain (dB)');  ylabel('PDF');
    title(sprintf('%s  (n=%d)  mean=%.1f dB', pName, sum(mask), mean(gainDb)));
    legend('Location','best','FontSize',7);
    grid on;
end

histPng = strrep(outPng, 'channel_gain_diagnostics', 'channel_gain_distributions');
applyLightTheme(fig2);
exportgraphics(fig2, histPng, 'Resolution',180);
close(fig2);
fprintf("Distribution figure saved: %s\n", histPng);

fprintf("\n── Summary ──────────────────────────────────────────────────────\n");
fprintf("Channel gain OLS R²        = %.3f  (point prediction — fundamentally limited by stochastic floor)\n", ...
    rsq(T.toolbox_channel_gain_db, splitGainPred));
fprintf("  → REPLACED by distance-conditioned distribution sampler:\n");
fprintf("     Ω(d) = 10^(a + b·log₁₀(d))  fitted per profile\n");
for pi = 1:height(distRows)
    fprintf("     %-6s  Ω(d) fit R²=%.3f  (slope=%.3f)\n", ...
        distRows.profile(pi), distRows.r2_omega_fit(pi), distRows.omega_log10_slope(pi));
end
fprintf("     Remaining variance drawn from Rician/Rayleigh — physically irreducible.\n");
fprintf("Shadow fading       → already a random draw (N(0,σ²)) — correct, nothing to change\n");
fprintf("K-factor OLS R²     = %.3f  RMSE=%.1f dB → KEEP (clamped to ±0.75 dB at runtime)\n", ...
    S.kFactorModel.Rsquared.Ordinary, sqrt(mean((T.toolbox_k_factor_db - kPred).^2)));
fprintf("Delay spread OLS R² = %.3f              → KEEP (only affects air delay, minor)\n", ...
    rsquaredVal(T.toolbox_delay_spread_ns, delayPred));

%% ── Local helpers ───────────────────────────────────────────────────────────
function applyLightTheme(fig)
    % Set figure and all axes to a clean white presentation theme.
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
    % Legends
    legList = findall(fig, 'type', 'legend');
    for k = 1:numel(legList)
        set(legList(k), 'Color', 'white', 'TextColor', [0 0 0], ...
            'EdgeColor', [0.7 0.7 0.7]);
    end
    % Colorbars
    cbList = findall(fig, 'type', 'colorbar');
    for k = 1:numel(cbList)
        set(cbList(k), 'Color', [0.15 0.15 0.15]);
    end
    % Any remaining text (sgtitle etc.)
    txtList = findall(fig, 'type', 'text');
    for k = 1:numel(txtList)
        if isequal(get(txtList(k), 'Color'), [1 1 1])
            set(txtList(k), 'Color', [0 0 0]);
        end
    end
end

function v = rsq(actual, predicted)
    sse = sum((actual(:) - predicted(:)).^2);
    sst = sum((actual(:) - mean(actual(:))).^2);
    v = max(0, 1 - sse/max(sst, eps));
end

function v = rsquaredVal(actual, predicted)
    sse = sum((actual(:) - predicted(:)).^2);
    sst = sum((actual(:) - mean(actual(:))).^2);
    v = max(0, 1 - sse/max(sst, eps));
end
