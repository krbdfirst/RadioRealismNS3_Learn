function p = rr_paths()
%RR_PATHS Path definitions for the 5G Toolbox fitting pipeline.
%
%   p = rr_paths() returns a struct of the directories these scripts read and
%   write.
%
%   Fields
%     p.repoRoot     repository root
%     p.resultsRoot  root for simulation results and fitting inputs
%     p.phyOutDir    5G Toolbox model outputs (.mat, training dataset, figures)
%     p.runtimeDir   exported runtime coefficients read by the OMNeT model
%     p.projectDir   OMNeT working directory (01_omnet_project)
%
%   Each field is taken from an environment variable if set (RR_ROOT,
%   RR_RESULTS_DIR, RR_PHY_OUT_DIR, RR_COEFF_DIR), otherwise from a default
%   derived from this file's location.
%
%   p.runtimeDir defaults to 01_omnet_project/model_coefficients/phy_5gtoolbox,
%   so re-running the export scripts overwrites the shipped coefficients.

    thisDir = fileparts(mfilename('fullpath'));
    repoRoot = getenvOr('RR_ROOT', fullfile(thisDir, '..', '..'));
    p.repoRoot = char(java.io.File(repoRoot).getCanonicalPath());

    p.projectDir  = getenvOr('RR_PROJECT_DIR', fullfile(p.repoRoot, '01_omnet_project'));
    p.resultsRoot = getenvOr('RR_RESULTS_DIR', fullfile(p.projectDir, 'results'));
    p.phyOutDir   = getenvOr('RR_PHY_OUT_DIR', fullfile(p.resultsRoot, 'phy_model_outputs_5gtoolbox'));
    p.runtimeDir  = getenvOr('RR_COEFF_DIR',   fullfile(p.projectDir, 'model_coefficients', 'phy_5gtoolbox'));

    % Create output directories if needed; inputs are checked by the callers.
    for f = {'resultsRoot', 'phyOutDir', 'runtimeDir'}
        d = p.(f{1});
        if ~isfolder(d)
            mkdir(d);
        end
    end
end

function v = getenvOr(name, defaultValue)
    v = getenv(name);
    if isempty(v)
        v = defaultValue;
    end
    v = char(v);
end
