function build_bler_curve_mcs14(outCsv)
%BUILD_BLER_CURVE_MCS14  CDL-C block-error-rate curve at the teacher's link setting.
%
% The Combined Analytical Reference decodes against an independent link-level
% curve. Independence comes from generating it in the 5G Toolbox rather than
% from ns-3 output, so the modulation and coding setting is free to match the
% teacher, and matching it removes a confound: any residual gap between the
% reference and the teacher is then attributable to medium access alone.
%
% The teacher transmits at index 14 of NR modulation and coding table 1, which
% the ns-3 nr module implements as 16-QAM at code rate 553/1024. This routine
% reproduces buildProxyBlerCurve from
% export_fitted_phy_model_coeffs_5gtoolbox_core.m with those two values
% substituted for the QPSK 490/1024 pair used previously; every other element
% of the chain is unchanged.
%
% Usage:
%   build_bler_curve_mcs14                       % writes beside the other curves
%   build_bler_curve_mcs14('/path/to/out.csv')

if nargin < 1 || isempty(outCsv)
    here   = fileparts(mfilename('fullpath'));
    outCsv = fullfile(here, '..', '..', '01_omnet_project', 'model_coefficients', ...
                      'phy_5gtoolbox', 'bler_curve_cdl_c_mcs14.csv');
end

% ---- link setting: matched to the teacher -------------------------------
targetCodeRate = 553 / 1024;    % NR MCS table 1, index 14
modulation     = '16QAM';       % NR MCS table 1, index 14

% ---- everything below reproduces the existing generator ------------------
scsKhz      = 30;               % runtime_constants.csv
nSizeGrid   = 52;               % runtime_constants.csv
snrRangeDb  = (-10:2:30)';
numTrials   = 60;
profileName = "CDL-C";
% Representative point carried over from the QPSK curve so the two are
% directly comparable.
delaySpreadNs = 32.6985585070477;
dopplerHz     = 71.9283774752388;

carrier = nrCarrierConfig('SubcarrierSpacing', scsKhz, 'NSizeGrid', nSizeGrid);

pdsch = nrPDSCHConfig;
pdsch.Modulation      = modulation;
pdsch.NumLayers       = 1;
pdsch.MappingType     = 'A';
pdsch.SymbolAllocation = [0 14];
pdsch.PRBSet          = 0:(carrier.NSizeGrid - 1);
pdsch.RNTI            = 1;
pdsch.NID             = 1;

[~, pdschInfo] = nrPDSCHIndices(carrier, pdsch);
G   = pdschInfo.G;
tbs = nrTBS(pdsch.Modulation, pdsch.NumLayers, numel(pdsch.PRBSet), ...
            pdschInfo.NREPerPRB, targetCodeRate);

encoder = nrDLSCH;
encoder.TargetCodeRate = targetCodeRate;
decoder = nrDLSCHDecoder;
decoder.TargetCodeRate           = targetCodeRate;
decoder.TransportBlockLength     = tbs;
decoder.MaximumLDPCIterationCount = 12;

fprintf('modulation %s, target code rate %.4f, TBS %d bits, G %d\n', ...
        modulation, targetCodeRate, tbs, G);

rng(1103, 'twister');   % reproducible
bler = zeros(numel(snrRangeDb), 1);
psr  = zeros(numel(snrRangeDb), 1);

for si = 1:numel(snrRangeDb)
    snrDb = snrRangeDb(si);
    N0    = 1 / 10^(snrDb / 10);

    blockErrors = 0;
    for trial = 1:numTrials %#ok<UNUSED>
        reset(decoder);
        trBlk = randi([0 1], tbs, 1);
        setTransportBlock(encoder, trBlk, 0);
        codedBits = encoder(pdsch.Modulation, pdsch.NumLayers, G, 0);

        txSym     = nrSymbolModulate(codedBits, pdsch.Modulation);
        noise     = sqrt(N0 / 2) * (randn(size(txSym)) + 1j * randn(size(txSym)));
        dlschLLRs = nrSymbolDemodulate(txSym + noise, pdsch.Modulation, N0);

        [~, blkErr] = decoder(dlschLLRs, pdsch.Modulation, pdsch.NumLayers, 0);
        blockErrors = blockErrors + double(blkErr);
    end

    bler(si) = blockErrors / numTrials;
    psr(si)  = 1 - bler(si);
    fprintf('  SNR %+5.1f dB   BLER %.4f\n', snrDb, bler(si));
end

curve = table( ...
    repmat(profileName,   numel(snrRangeDb), 1), ...
    repmat(delaySpreadNs, numel(snrRangeDb), 1), ...
    repmat(dopplerHz,     numel(snrRangeDb), 1), ...
    repmat(numTrials,     numel(snrRangeDb), 1), ...
    snrRangeDb, bler, psr, ...
    'VariableNames', {'profile','delay_spread_ns','doppler_hz','trial_count','snr_db','bler','psr'});

writetable(curve, outCsv);
fprintf('wrote %s\n', outCsv);
end
