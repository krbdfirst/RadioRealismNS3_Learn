%% export_fitted_phy_model_coeffs_5gtoolbox.m
% Export runtime-ready coefficients and BLER curves for the standard PHY build.

clear;
clc;

rrp = rr_paths();
export_fitted_phy_model_coeffs_5gtoolbox_core( ...
    rrp.resultsRoot, ...
    "phy_model_outputs_5gtoolbox");
