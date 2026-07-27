%% run_full_phy_model_pipeline_5gtoolbox_5000.m
% Run the full 5G Toolbox-backed generalized PHY modeling pipeline (5000-sample build).
%
% Training / validation split:
%   Seeds 0-4  : training   (5 seeds per adoption level)
%   Seed  5    : holdout    (1 seed per adoption level, never seen by production model fitting)
%
% The buildFitMetrics stage automatically identifies the highest-indexed
% seed (5) as holdout per adoption level, fits shadow models on seeds 0-4,
% and reports holdout R² / RMSE separately from the train metrics.
% The production model coefficients exported to runtime_coefficients/ are
% fitted on all 6 seeds for maximum accuracy; the holdout metrics validate
% that they generalise.
%
% Required MATLAB products:
%   - MATLAB (base)
%   - 5G Toolbox
%   - Statistics and Machine Learning Toolbox

clear;
clc;

rrp = rr_paths();
resultsRoot = rrp.resultsRoot;

% Recursive discovery so that runs inside sub-folders (e.g. "PHY Model/") are found.
allRunDirs = dir(fullfile(resultsRoot, "**", "Ideal_PHYModel_CAV*"));
allRunDirs = allRunDirs([allRunDirs.isdir]);

fprintf("Discovered %d adoption/seed result folders under %s\n", numel(allRunDirs), resultsRoot);
for k = 1:numel(allRunDirs)
    fprintf("  - %s\n", allRunDirs(k).name);
end
fprintf("\n");

% ── Run build and export sub-scripts ──────────────────────────────────────
% Each sub-script starts with 'clear; clc', which wipes the workspace.
% We re-assign resultsRoot after each one so the final summary printout works.
build_generalized_phy_model_5gtoolbox_5000;

resultsRoot = rrp.resultsRoot;
export_fitted_phy_model_coeffs_5gtoolbox_5000;

resultsRoot = rrp.resultsRoot;
fprintf("Full 5G Toolbox PHY modeling pipeline (5000-sample) complete.\n");
fprintf("Model folder       : %s\n", fullfile(resultsRoot, "phy_model_outputs_5gtoolbox_5000"));
fprintf("Runtime coefficients: %s\n", fullfile(resultsRoot, "phy_model_outputs_5gtoolbox_5000", "runtime_coefficients"));
