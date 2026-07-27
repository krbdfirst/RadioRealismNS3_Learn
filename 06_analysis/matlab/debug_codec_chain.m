%% debug_codec_chain.m
% Three targeted tests that narrow down exactly where the codec chain breaks.
%
% Test 1 — decoder alone (perfect LLRs, no channel at all)
% Test 2 — nrPDSCH → nrPDSCHDecode with zero noise (no OFDM, no channel)
% Test 3 — full OFDM + AWGN (20 dB, identity channel)
%
% Interpretation:
%   Test 1 fails  → nrDLSCHDecoder itself is misconfigured
%   Test 1 OK, Test 2 fails  → scrambling/modulation bug in PDSCH layer
%   Tests 1+2 OK, Test 3 fails  → OFDM or noise-normalisation issue
clear; clc;

%% ── Shared config ────────────────────────────────────────────────────────
carrier = nrCarrierConfig;
carrier.NSizeGrid       = 25;
carrier.SubcarrierSpacing = 30;
carrier.NSlot           = 0;

pdsch = nrPDSCHConfig;
pdsch.Modulation        = 'QPSK';
pdsch.NumLayers         = 1;
pdsch.MappingType       = 'A';
pdsch.SymbolAllocation  = [0 14];
pdsch.PRBSet            = 0:24;
pdsch.RNTI = 1;  pdsch.NID = 1;

targetCodeRate = 490 / 1024;
[pdschIndices, pdschInfo] = nrPDSCHIndices(carrier, pdsch);
dmrsIndices = nrPDSCHDMRSIndices(carrier, pdsch);
G   = pdschInfo.G;
tbs = nrTBS(pdsch.Modulation, pdsch.NumLayers, numel(pdsch.PRBSet), ...
            pdschInfo.NREPerPRB, targetCodeRate);

fprintf('TBS=%d  G=%d\n\n', tbs, G);

encoder = nrDLSCH;
encoder.TargetCodeRate = targetCodeRate;

decoder = nrDLSCHDecoder;
decoder.TargetCodeRate       = targetCodeRate;
decoder.TransportBlockLength = tbs;
decoder.MaximumLDPCIterationCount = 12;

%% ── Encode once ──────────────────────────────────────────────────────────
trBlk = randi([0 1], tbs, 1);
setTransportBlock(encoder, trBlk, 0);
codedBits = encoder(pdsch.Modulation, pdsch.NumLayers, G, 0);

% =========================================================================
%% Test 1 — perfect LLRs directly into decoder  (no RF at all)
% =========================================================================
fprintf('── Test 1: perfect LLRs → decoder ─────────────────────────────\n');
% MATLAB convention: LLR < 0  →  bit = 1
%                    LLR > 0  →  bit = 0
llrs_perfect = (1 - 2*double(codedBits)) * 100;   % +100 for bit=0, -100 for bit=1
reset(decoder);
[rxBits1, e1] = decoder(llrs_perfect, pdsch.Modulation, pdsch.NumLayers, 0);
fprintf('  crcErr=%d   bits_match=%d\n\n', e1, isequal(rxBits1, trBlk));

% =========================================================================
%% Test 2 — nrPDSCH → nrPDSCHDecode with negligible noise (no OFDM)
% =========================================================================
fprintf('── Test 2: nrPDSCH → nrPDSCHDecode (no OFDM, no channel) ──────\n');
txSym = nrPDSCH(carrier, pdsch, codedBits);      % modulate + scramble
noiseTiny = 1e-9 * (randn(size(txSym)) + 1j*randn(size(txSym)));
llrs2 = nrPDSCHDecode(carrier, pdsch, txSym + noiseTiny, 1e-18);
if iscell(llrs2), llrs2 = llrs2{1}; end
sign_match2 = mean(double(codedBits) == double(llrs2 < 0));
fprintf('  LLR sign match: %.4f  (expect ~1.0)\n', sign_match2);
reset(decoder);
[rxBits2, e2] = decoder(llrs2, pdsch.Modulation, pdsch.NumLayers, 0);
fprintf('  crcErr=%d   bits_match=%d\n\n', e2, isequal(rxBits2, trBlk));

% =========================================================================
%% Test 3 — full OFDM + AWGN at 20 dB, identity channel estimate
% =========================================================================
fprintf('── Test 3: OFDM + AWGN (20 dB), identity h=1 ───────────────────\n');
N0 = 1 / 10^(20/10);   % = 0.01

txGrid = zeros(12*carrier.NSizeGrid, 14, 1);
txGrid(pdschIndices) = nrPDSCH(carrier, pdsch, codedBits);
txGrid(dmrsIndices)  = nrPDSCHDMRS(carrier, pdsch);
txWav  = nrOFDMModulate(carrier, txGrid);
rxWav  = txWav + sqrt(N0/2)*(randn(size(txWav)) + 1j*randn(size(txWav)));
rxGrid = nrOFDMDemodulate(carrier, rxWav);

hId = ones(12*carrier.NSizeGrid, 14);
[pr, ph]    = nrExtractResources(pdschIndices, rxGrid(:,:,1), hId);
[eq, csi]   = nrEqualizeMMSE(pr, ph, N0);

fprintf('  mean|eqSym|=%.4f  (QPSK symbols have |s|=1, expect close to 1)\n', mean(abs(eq)));
fprintf('  Re(eq) range: [%.3f, %.3f]\n', min(real(eq)), max(real(eq)));

llrs3 = nrPDSCHDecode(carrier, pdsch, eq, N0);
if iscell(llrs3), llrs3 = llrs3{1}; end

sign_match3 = mean(double(codedBits) == double(llrs3 < 0));
fprintf('  LLR sign match (no CSI weight): %.4f\n', sign_match3);

reset(decoder);
[rxBits3a, e3a] = decoder(llrs3, pdsch.Modulation, pdsch.NumLayers, 0);
fprintf('  Without CSI weight: crcErr=%d\n', e3a);

if ~isempty(csi)
    llrs3w = llrs3 .* repelem(csi(:), 2, 1);
    reset(decoder);
    [rxBits3b, e3b] = decoder(llrs3w, pdsch.Modulation, pdsch.NumLayers, 0);
    fprintf('  With    CSI weight: crcErr=%d\n', e3b);
end

% =========================================================================
%% Bonus — does N0 normalisation look right?
% =========================================================================
fprintf('\n── Normalisation check ─────────────────────────────────────────\n');
fprintf('  N0 = %.4f\n', N0);
fprintf('  Expected QPSK symbol amplitude |s| = 1.0\n');
fprintf('  Noise sigma per component = sqrt(N0/2) = %.4f\n', sqrt(N0/2));
fprintf('  SNR per component (linear) = (1/sqrt(2))^2 / (N0/2) = %.1f\n', 0.5/(N0/2));
fprintf('  i.e. %.1f dB\n', 10*log10(0.5/(N0/2)));
