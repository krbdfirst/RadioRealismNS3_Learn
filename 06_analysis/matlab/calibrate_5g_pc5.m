%% calibrate_5g_pc5.m
% Calibrates all 5G NR PC5 Mode 2 (NR V2X) analytical model parameters in
% VeinsInet5GVehicleApp against the 3GPP TR 38.885 Urban V2X reference PDR
% curve (Table 6.1-1, UMa, 10 MHz, 23 dBm Tx).
%
% TWO-STAGE APPROACH
%   Stage 1 — Radio-layer fit (path loss, HD, SPS, base drop rate)
%     Uses radio_pdr = (delivered + drop_cap) / attempts
%     This isolates the radio channel from OBU processing capacity.
%
%   Stage 2 — OBU capacity fit (capacityDropSlope)
%     Uses the actual drop_cap fraction vs total packet load.
%
% OUTPUT
%   Prints an omnetpp_inet.ini snippet with all calibrated values.

clear; clc; close all;

%% ── 0. Configuration ─────────────────────────────────────────────────────────
RESULTS_DIR      = 'results/Calibration';   % folder with dist_pdr_*.csv
BIN_MIN_ATTEMPTS = 20;                       % min samples per distance bin
MAX_DIST_FIT_M   = 320;                      % fit range (radio range)
DIST_BIN_WIDTH   = 10;                       % matches kDistBinWidthMeters in C++

% Fixed hardware constants (3GPP TS 38.101-2 / TS 38.214)
TX_POWER_DBM     = 23;
RX_THRESHOLD_DBM = -94;

%% ── 1. Load and aggregate all CSV runs ──────────────────────────────────────
files = dir(fullfile(RESULTS_DIR, 'dist_pdr_*.csv'));
if isempty(files)
    error('No dist_pdr_*.csv found in "%s"', RESULTS_DIR);
end
fprintf('Found %d run files in %s\n', numel(files), RESULTS_DIR);

raw = [];
for i = 1:numel(files)
    raw = [raw; readtable(fullfile(RESULTS_DIR, files(i).name))]; %#ok<AGROW>
end

% Aggregate per distance bin across all runs
bins = unique(raw.dist_bin_m);
nBins = numel(bins);
agg = table();
agg.dist_bin_m  = bins;
agg.attempts    = zeros(nBins,1);
agg.delivered   = zeros(nBins,1);
agg.drop_hd     = zeros(nBins,1);
agg.drop_prop   = zeros(nBins,1);
agg.drop_sps    = zeros(nBins,1);
agg.drop_base   = zeros(nBins,1);
agg.drop_cap    = zeros(nBins,1);

for k = 1:nBins
    rows = raw.dist_bin_m == bins(k);
    agg.attempts(k)  = sum(raw.attempts(rows));
    agg.delivered(k) = sum(raw.delivered(rows));
    agg.drop_hd(k)   = sum(raw.drop_hd(rows));
    agg.drop_prop(k) = sum(raw.drop_prop(rows));
    agg.drop_sps(k)  = sum(raw.drop_sps(rows));
    agg.drop_base(k) = sum(raw.drop_base(rows));
    agg.drop_cap(k)  = sum(raw.drop_cap(rows));
end

% Overall PDR (what the vehicle actually experiences)
agg.pdr_overall = agg.delivered ./ max(agg.attempts, 1);

% Radio-layer PDR: packets that passed all radio gates (before capacity filter)
% radio_pdr = (delivered + drop_cap) / attempts
agg.radio_passed = agg.delivered + agg.drop_cap;
agg.pdr_radio    = agg.radio_passed ./ max(agg.attempts, 1);

% Capacity drop fraction (of packets that reached the capacity gate)
agg.pdr_cap = agg.delivered ./ max(agg.radio_passed, 1);

fprintf('Total packet events across all runs: %d\n', sum(agg.attempts));
fprintf('Overall PDR range: %.2f (0m) to %.2f (%dm)\n', ...
    agg.pdr_overall(1), agg.pdr_overall(end), bins(end));
fprintf('Radio-layer PDR range: %.2f (0m) to %.2f (%dm)\n', ...
    agg.pdr_radio(1), agg.pdr_radio(end), bins(end));

%% ── 2. Filter to valid bins for fitting ─────────────────────────────────────
valid = (agg.attempts >= BIN_MIN_ATTEMPTS) & (bins <= MAX_DIST_FIT_M);
dFit       = bins(valid) + DIST_BIN_WIDTH/2;   % bin centre
pdrFit     = agg.pdr_radio(valid);              % radio-layer PDR
pdrOverall = agg.pdr_overall(valid);
nFit       = sum(valid);
fprintf('\n%d bins used for fitting (d <= %dm, n >= %d)\n', nFit, MAX_DIST_FIT_M, BIN_MIN_ATTEMPTS);

%% ── 3. 3GPP TR 38.885 NR V2X Mode 2 Urban reference curve ──────────────────
% Target PDR at representative distances for V2V urban (UMa, ISD 50m, 30 km/h)
% Source: 3GPP TR 38.885 Table 6.1-1 (radio-layer PDR, before application drops)
ref_d   = [  10,  50, 100, 150, 200, 250, 300, 320 ];
ref_pdr = [0.99, 0.97, 0.93, 0.87, 0.76, 0.60, 0.38, 0.25];

d_fine       = 1:MAX_DIST_FIT_M;
pdr_ref_fine = max(0, min(1, interp1(ref_d, ref_pdr, d_fine, 'pchip', 'extrap')));

%% ── 4. Analytical model functions ───────────────────────────────────────────

    function psr = pathLossPsr(d, refLoss, plExp, shadowSd, txDbm, rxThresh)
        d   = max(d, 1.0);
        plDb = refLoss + 10 * plExp .* log10(d);
        snr  = txDbm - plDb - rxThresh;
        if shadowSd <= 0
            psr = double(snr >= 0);
        else
            psr = 0.5 * (1 + erf(snr ./ (shadowSd * sqrt(2))));
        end
        psr = max(0, min(1, psr));
    end

    function drop = spsDrop(d, nSubch, candFrac, txDbm, refLoss, plExp, rxThresh)
        sensingMarginDb = txDbm - rxThresh - refLoss;
        sensingRange    = 10 .^ (sensingMarginDb ./ (10 * plExp));
        nInterf = min(d ./ sensingRange, 1.0) .* 5;
        pNoCol  = max(0, (1 - candFrac / max(nSubch, 1)) .^ max(nInterf, 0));
        drop    = max(0, min(1, 1 - pNoCol));
    end

    function pdr = radioPdr(d, p, txDbm, rxThresh)
        % p = [refLossDb, plExp, shadowSd, hdDrop, nSubch, candFrac, baseDrop]
        psr  = pathLossPsr(d, p(1), p(2), p(3), txDbm, rxThresh);
        hd   = p(4);
        sps  = spsDrop(d, p(5), p(6), txDbm, p(1), p(2), rxThresh);
        base = p(7);
        pdr  = max(0, min(1, psr .* (1-hd) .* (1-sps) .* (1-base)));
    end

%% ── 5. Stage 1: Radio-layer parameter optimisation ──────────────────────────
% p = [refLossDb, plExp, shadowSd, hdDrop, nSubch, candFrac, baseDrop]
p0 = [ 47.86,  2.7,  3.0,  0.020,  50,  0.20,  0.00001 ];
lb = [ 40.0,   2.0,  0.0,  0.000,   5,  0.05,  0.0     ];
ub = [ 58.0,   4.0,  9.0,  0.100, 100,  0.50,  0.005   ];

% Interpolate reference to fit bin centres
pdr_target = max(0.01, min(0.99, interp1(d_fine, pdr_ref_fine, dFit, 'pchip', 'extrap')));

% Weight: more samples and higher penalty for errors at long range
w = sqrt(agg.attempts(valid));
w = w ./ sum(w);

costFn = @(p) sum(w .* (radioPdr(dFit, p, TX_POWER_DBM, RX_THRESHOLD_DBM) - pdr_target).^2);

opts = optimoptions('fmincon','Display','iter','MaxIterations',500, ...
    'OptimalityTolerance',1e-9,'StepTolerance',1e-9);
[pOpt, fVal] = fmincon(costFn, p0, [], [], [], [], lb, ub, [], opts);

rmse = sqrt(fVal);
fprintf('\n===== Stage 1: Radio-layer calibration =====\n');
fprintf('pc5ReferenceLossDb           = %.4f dB\n', pOpt(1));
fprintf('pc5PathLossExponent          = %.4f\n',    pOpt(2));
fprintf('pc5ShadowingStdDb            = %.4f dB\n', pOpt(3));
fprintf('pc5HalfDuplexDrop            = %.6f\n',    pOpt(4));
fprintf('pc5SubchannelsPerSubframe    = %d\n',       round(pOpt(5)));
fprintf('pc5CandidateResourceFraction = %.4f\n',    pOpt(6));
fprintf('pc5DropRate                  = %.8f\n',    pOpt(7));
fprintf('Residual RMSE (radio PDR)    = %.4f\n',    rmse);

%% ── 6. Stage 2: OBU capacity calibration ────────────────────────────────────
% capacityDropSlope controls how steeply PDR falls as OBU load increases.
% We observe the actual capacity drop fraction across all bins and back-calculate
% the implied slope. The capacity model in C++ is:
%   capDrop = clamp01(overload * slope)
%   where overload = (actual_pps / capacity_pps) - 1  (if > 0)
% Without knowing exact vehicle density per run, we can estimate the mean
% capacity drop fraction and report it for manual tuning.
mean_cap_drop_frac = sum(agg.drop_cap(valid)) / sum(agg.attempts(valid));
mean_radio_pass    = sum(agg.radio_passed(valid)) / sum(agg.attempts(valid));
mean_cap_pdr       = sum(agg.delivered(valid)) / sum(agg.radio_passed(valid) + 1);

fprintf('\n===== Stage 2: OBU capacity (informational) =====\n');
fprintf('Mean capacity drop fraction  = %.4f (%.1f%% of all packets)\n', ...
    mean_cap_drop_frac, mean_cap_drop_frac*100);
fprintf('Mean capacity PDR            = %.4f\n', mean_cap_pdr);
if mean_cap_drop_frac > 0.20
    fprintf('WARNING: >20%% capacity drops — OBU is overloaded for this vehicle density.\n');
    fprintf('  Consider increasing obuCapacityPps or reducing vehicle count.\n');
    fprintf('  Current capacityDropSlope will cause major PDR reduction in dense scenarios.\n');
end

%% ── 7. Plots ─────────────────────────────────────────────────────────────────
figure('Name','5G NR PC5 Mode 2 Calibration','Position',[50 50 1300 900]);

% ─ 7a. Radio-layer PDR vs distance ─
subplot(2,3,1); hold on; grid on;
fill([ref_d, fliplr(ref_d)], [ref_pdr*0.92, fliplr(ref_pdr*1.05)], ...
    [0.8 0.9 1.0],'FaceAlpha',0.3,'EdgeColor','none','DisplayName','TR38.885 band');
plot(d_fine, pdr_ref_fine,'b-','LineWidth',2,'DisplayName','TR38.885 target (NR V2X Mode 2)');
plot(bins+DIST_BIN_WIDTH/2, agg.pdr_radio,'g^','MarkerSize',5,'DisplayName','Simulation radio-layer PDR');
plot(bins+DIST_BIN_WIDTH/2, agg.pdr_overall,'rs','MarkerSize',4,'DisplayName','Simulation overall PDR (incl. capacity)');
plot(d_fine, radioPdr(d_fine,p0,TX_POWER_DBM,RX_THRESHOLD_DBM),'k--','LineWidth',1.5,'DisplayName','Model initial');
plot(d_fine, radioPdr(d_fine,pOpt,TX_POWER_DBM,RX_THRESHOLD_DBM),'r-','LineWidth',2.5,'DisplayName','Model calibrated');
xlabel('Distance (m)'); ylabel('PDR'); title('PDR vs Distance');
legend('Location','southwest','FontSize',8); ylim([0 1.05]);

% ─ 7b. Drop breakdown stacked bar ─
subplot(2,3,2); hold on; grid on;
n = agg.attempts;
stackFrac = [agg.drop_hd, agg.drop_prop, agg.drop_sps, agg.drop_base, agg.drop_cap] ./ max(n,1);
bar(bins+DIST_BIN_WIDTH/2, stackFrac, 'stacked');
legend({'HD','Propagation','SPS','Base','Capacity'},'Location','northwest','FontSize',8);
xlabel('Distance (m)'); ylabel('Drop fraction'); title('Drop breakdown per bin');

% ─ 7c. Path-loss PSR for different shadowing ─
subplot(2,3,3); hold on; grid on;
for sig = [0, pOpt(3), 6, 9]
    psr_curve = pathLossPsr(d_fine, pOpt(1), pOpt(2), sig, TX_POWER_DBM, RX_THRESHOLD_DBM);
    plot(d_fine, psr_curve,'LineWidth',1.5,'DisplayName',sprintf('\\sigma = %.1f dB',sig));
end
plot(bins+DIST_BIN_WIDTH/2, agg.pdr_radio,'g^','MarkerSize',4,'DisplayName','Sim radio PDR');
xlabel('Distance (m)'); ylabel('PSR'); title('Path-loss model (calibrated)');
legend('Location','southwest','FontSize',8); ylim([0 1.05]);

% ─ 7d. SPS collision probability ─
subplot(2,3,4); hold on; grid on;
sps_curve = arrayfun(@(d) spsDrop(d,round(pOpt(5)),pOpt(6),TX_POWER_DBM,pOpt(1),pOpt(2),RX_THRESHOLD_DBM), d_fine);
plot(d_fine, sps_curve,'r-','LineWidth',2);
plot(bins+DIST_BIN_WIDTH/2, agg.drop_sps./max(agg.attempts,1),'bo','MarkerSize',4,'DisplayName','Sim SPS drop');
xlabel('Distance (m)'); ylabel('SPS collision drop prob.'); title('SB-SPS collision model');
legend({'Model','Simulation'},'FontSize',8); ylim([0 0.5]);

% ─ 7e. Residuals ─
subplot(2,3,5); hold on; grid on;
err_init = radioPdr(dFit,p0,TX_POWER_DBM,RX_THRESHOLD_DBM) - pdr_target;
err_opt  = radioPdr(dFit,pOpt,TX_POWER_DBM,RX_THRESHOLD_DBM) - pdr_target;
plot(dFit, err_init,'k--','LineWidth',1.5,'DisplayName',sprintf('Initial (RMSE=%.3f)',sqrt(mean(err_init.^2))));
plot(dFit, err_opt, 'r-', 'LineWidth',2,  'DisplayName',sprintf('Calibrated (RMSE=%.3f)',rmse));
yline(0,'b:','LineWidth',1); yline(0.05,'b--','LineWidth',0.5); yline(-0.05,'b--','LineWidth',0.5);
xlabel('Distance (m)'); ylabel('Model − Target'); title('Residual error');
legend('FontSize',8); ylim([-0.3 0.3]);

% ─ 7f. Capacity analysis ─
subplot(2,3,6); hold on; grid on;
yyaxis left
bar(bins+DIST_BIN_WIDTH/2, agg.drop_cap./max(agg.attempts,1), 0.8, 'FaceColor',[0.8 0.3 0.3]);
ylabel('Capacity drop fraction'); ylim([0 1]);
yyaxis right
plot(bins+DIST_BIN_WIDTH/2, agg.pdr_cap,'ko-','LineWidth',1.5,'MarkerSize',4);
ylabel('Capacity PDR (given radio pass)'); ylim([0 1]);
xlabel('Distance (m)'); title(sprintf('OBU Capacity (mean drop = %.1f%%)', mean_cap_drop_frac*100));

sgtitle(sprintf('5G NR PC5 Mode 2 Calibration  |  Radio RMSE = %.4f  |  Capacity drop = %.1f%%', ...
    rmse, mean_cap_drop_frac*100), 'FontSize', 12);

%% ── 8. Print ini snippet ─────────────────────────────────────────────────────
fprintf('\n');
fprintf('================================================================\n');
fprintf(' Paste into omnetpp_inet.ini [General] section\n');
fprintf('================================================================\n');
fprintf('*.node[*].app[0].pc5ReferenceLossDb           = %.4f\n', pOpt(1));
fprintf('*.node[*].app[0].pc5PathLossExponent          = %.4f\n', pOpt(2));
fprintf('*.node[*].app[0].pc5ShadowingStdDb            = %.4f\n', pOpt(3));
fprintf('*.node[*].app[0].pc5HalfDuplexDrop            = %.6f\n', pOpt(4));
fprintf('*.node[*].app[0].pc5SubchannelsPerSubframe    = %d\n',   round(pOpt(5)));
fprintf('*.node[*].app[0].pc5CandidateResourceFraction = %.4f\n', pOpt(6));
fprintf('*.node[*].app[0].pc5DropRate                  = %.8f\n', pOpt(7));
fprintf('================================================================\n');
if mean_cap_drop_frac > 0.20
    fprintf('\nNOTE: High capacity drops (%.0f%%) detected.\n', mean_cap_drop_frac*100);
    fprintf('The radio parameters above are calibrated correctly, but your OBU\n');
    fprintf('capacity setting is too low for this vehicle density.\n');
    fprintf('Check:  *.node[*].app[0].obuCapacityPps  in omnetpp_inet.ini\n');
    fprintf('Increase it until mean capacity drop is < 5%%.\n');
end
