%% run_full_phy_model_pipeline_5gtoolbox.m
% Run the full 5G Toolbox-backed generalized PHY modeling pipeline.
%
% This script:
%   1. Builds the generalized PHY training dataset from all available
%      Ideal_PHYModel_CAV* result folders.
%   2. Fits and saves the generalized PHY model.
%   3. Exports runtime-friendly coefficient tables and readable equations.
%
% Required MATLAB products:
%   - MATLAB (base)
%   - 5G Toolbox
%   - Statistics and Machine Learning Toolbox

clear;
clc;

rrp = rr_paths();
resultsRoot = rrp.resultsRoot;
runDirs = dir(fullfile(resultsRoot, "Ideal_PHYModel_CAV*"));
runDirs = runDirs([runDirs.isdir]);

fprintf("Discovered %d adoption/seed result folders under %s\n", numel(runDirs), resultsRoot);
for k = 1:numel(runDirs)
    fprintf("  - %s\n", runDirs(k).name);
end
fprintf("\n");

build_generalized_phy_model_5gtoolbox;
export_fitted_phy_model_coeffs_5gtoolbox;

% Restore resultsRoot — cleared by the sub-scripts' own 'clear' calls
resultsRoot = rrp.resultsRoot;
fprintf("Full 5G Toolbox PHY modeling pipeline complete.\n");
fprintf("Model folder: %s\n", fullfile(resultsRoot, "phy_model_outputs_5gtoolbox"));
fprintf("Runtime coefficients: %s\n", fullfile(resultsRoot, "phy_model_outputs_5gtoolbox", "runtime_coefficients"));
