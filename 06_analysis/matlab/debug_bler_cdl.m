%% debug_bler_cdl.m
% Single-trial CDL chain diagnostic.
% Mirrors buildProxyBlerCurve exactly and prints every intermediate
% quantity so we can see where the decode fails.
clear; clc;

%% ── Carrier & PDSCH config (same as buildProxyBlerCurve) ─────────────────
carrier = nrCarrierConfig;
carrier.NSizeGrid       = 25;
carrier.SubcarrierSpacing = 30;
carrier.NSlot           = 0;

pdsch = nrPDSCHConfig;
pdsch.Modulation        = 'QPSK';
pdsch.NumLayers         = 1;
pdsch.MappingType       = 'A';
pdsch.SymbolAllocation  = [0 14];
pdsch.PRBSet            = 0:(carrier.NSizeGrid - 1);
pdsch.RNTI = 1;  pdsch.NID = 1;

targetCodeRate = 490 / 1024;
[pdschIndices, pdschInfo] = nrPDSCHIndices(carrier, pdsch);
dmrsIndices  = nrPDSCHDMRSIndices(carrier, pdsch);
G   = pdschInfo.G;
tbs = nrTBS(pdsch.Modulation, pdsch.NumLayers, numel(pdsch.PRBSet), ...
            pdschInfo.NREPerPRB, targetCodeRate);
qm  = 2;   % QPSK
ofdmInfo = nrOFDMInfo(carrier);

fprintf('TBS=%d  G=%d  Nfft=%d  SampleRate=%.3f MHz\n', ...
    tbs, G, ofdmInfo.Nfft, ofdmInfo.SampleRate/1e6);

%% ── Encoder / decoder ────────────────────────────────────────────────────
encoder = nrDLSCH;
encoder.TargetCodeRate = targetCodeRate;

decoder = nrDLSCHDecoder;
decoder.TargetCodeRate       = targetCodeRate;
decoder.TransportBlockLength = tbs;
decoder.MaximumLDPCIterationCount = 12;

%% ── Channel parameters (CDL-A at median training values) ─────────────────
profileName   = 'CDL-A';
delaySpreadNs = 31;      % median from training data
dopplerHz     = 76;
fcHz          = 5.9e9;

snrDb = 20;
N0    = 1 / 10^(snrDb / 10);
fprintf('\nSNR=%d dB   N0=%.5f\n', snrDb, N0);

%% ── Build channel object ─────────────────────────────────────────────────
ch = nrCDLChannel;
ch.DelayProfile             = profileName;
ch.DelaySpread              = max(delaySpreadNs, 1.0) * 1e-9;
ch.MaximumDopplerShift      = max(1.0, dopplerHz);
ch.CarrierFrequency         = fcHz;
ch.SampleRate               = ofdmInfo.SampleRate;
ch.NormalizePathGains       = true;
ch.NormalizeChannelOutputs  = true;
ch.ChannelFiltering         = true;
ch.RandomStream             = 'mt19937ar with seed';
ch.Seed                     = 1104;
ch.TransmitAntennaArray.Size = [1 1 1 1 1];
ch.ReceiveAntennaArray.Size  = [1 1 1 1 1];

info_ch = info(ch);
fprintf('Channel: nTx=%d  nRx=%d  nPaths=%d\n', ...
    info_ch.NumTransmitAntennas, info_ch.NumReceiveAntennas, ...
    numel(info_ch.AveragePathGains));

%% ── Single trial ─────────────────────────────────────────────────────────
reset(decoder);
trBlk = randi([0 1], tbs, 1);
setTransportBlock(encoder, trBlk, 0);
codedBits = encoder(pdsch.Modulation, pdsch.NumLayers, G, 0);

txGrid = zeros(12 * carrier.NSizeGrid, 14, pdsch.NumLayers);
txGrid(pdschIndices) = nrSymbolModulate(codedBits, pdsch.Modulation);  % no scrambling
txGrid(dmrsIndices)  = nrPDSCHDMRS(carrier, pdsch);

txWaveform = nrOFDMModulate(carrier, txGrid);
nTxSamples = size(txWaveform, 1);
txPadded   = [txWaveform; zeros(ofdmInfo.Nfft, size(txWaveform, 2))];
fprintf('\ntxWaveform: %d samples   padded input: %d samples\n', ...
    nTxSamples, size(txPadded, 1));

%% ── CDL channel ──────────────────────────────────────────────────────────
[rxWav, pathGains, sampleTimes] = ch(txPadded);
pathFilters = getPathFilters(ch);
fprintf('rxWav from channel: %d samples   pathGains: %s\n', ...
    size(rxWav, 1), mat2str(size(pathGains)));

%% ── Timing offset ────────────────────────────────────────────────────────
offset = nrPerfectTimingEstimate(pathGains, pathFilters);
fprintf('Timing offset: %d samples\n', offset);

rxWav = rxWav(1 + offset : offset + nTxSamples, :);
fprintf('rxWav after timing crop: %d samples\n', size(rxWav, 1));

%% ── AWGN ──────────────────────────────────────────────────────────────────
noise = sqrt(N0 / 2) * (randn(size(rxWav)) + 1j * randn(size(rxWav)));
rxWav = rxWav + noise;

%% ── OFDM demodulate ───────────────────────────────────────────────────────
rxGrid = nrOFDMDemodulate(carrier, rxWav);
fprintf('rxGrid size: %s\n', mat2str(size(rxGrid)));

%% ── Perfect channel estimate ──────────────────────────────────────────────
hPerfect = nrPerfectChannelEstimate(carrier, pathGains, pathFilters, offset, sampleTimes);
fprintf('hPerfect size: %s\n', mat2str(size(hPerfect)));
hSiso = squeeze(hPerfect(:, :, 1, 1));
fprintf('hSiso size: %s   mean|h|=%.4f   min|h|=%.4f   max|h|=%.4f\n', ...
    mat2str(size(hSiso)), mean(abs(hSiso(:))), min(abs(hSiso(:))), max(abs(hSiso(:))));

%% ── Extract resources ─────────────────────────────────────────────────────
[pdschRx, pdschHest] = nrExtractResources(pdschIndices, rxGrid(:,:,1), hSiso);
fprintf('pdschRx: %d elements   pdschHest: %d elements\n', ...
    numel(pdschRx), numel(pdschHest));
fprintf('mean|pdschRx|=%.4f   mean|pdschHest|=%.4f\n', ...
    mean(abs(pdschRx)), mean(abs(pdschHest)));

%% ── MMSE equalize (diagnostic printout only — not used for decode) ────────
[eqSym, csi] = nrEqualizeMMSE(pdschRx, pdschHest, N0);
fprintf('mean|eqSym|=%.4f   std|eqSym|=%.4f   mean|csi|=%.4f\n', ...
    mean(abs(eqSym)), std(abs(eqSym)), mean(abs(csi)));

% Quick constellation sanity: QPSK symbols should cluster near ±1/sqrt(2)
fprintf('Re(eqSym) in [%.2f, %.2f]   Im in [%.2f, %.2f]\n', ...
    min(real(eqSym)), max(real(eqSym)), min(imag(eqSym)), max(imag(eqSym)));

%% ── Soft demodulate → LLRs ───────────────────────────────────────────────
% nrSymbolDemodulate (not nrPDSCHDecode): avoids the version-specific sign
% inversion in nrPDSCHDecode.  Paired with nrSymbolModulate on the TX side.
dlschLLRs = nrSymbolDemodulate(eqSym, pdsch.Modulation, N0);
if ~isempty(csi)
    dlschLLRs = dlschLLRs .* repelem(csi(:), qm);
end
fprintf('LLR length: %d   mean|LLR|=%.4f   max|LLR|=%.4f   NaN/Inf: %d\n', ...
    numel(dlschLLRs), mean(abs(dlschLLRs)), max(abs(dlschLLRs)), ...
    sum(~isfinite(dlschLLRs)));

% Sign sanity — positive LLR = bit 0 (standard).
% bit=0→LLR>0→sign=1, cb=0: FALSE.  bit=1→LLR<0→sign=0, cb=1: FALSE.
% 0.000 = perfect alignment.  0.500 = random.  1.000 = inverted.
llr_sign_check = mean(double(codedBits) == double(dlschLLRs > 0));
fprintf('LLR sign check: %.4f  (0.000 = perfect, 0.500 = random)\n', llr_sign_check);

%% ── Decode ────────────────────────────────────────────────────────────────
[rxBits, blkErr] = decoder(dlschLLRs, pdsch.Modulation, pdsch.NumLayers, 0);
fprintf('\nblkErr=%d   bits_match=%d\n', blkErr, isequal(rxBits, trBlk));

%% ── Bonus: AWGN-only baseline (same trBlk, no channel) ──────────────────
fprintf('\n── AWGN-only baseline (no fading) ──────────────────────────────\n');
txWav2  = nrOFDMModulate(carrier, txGrid);
noise2  = sqrt(N0/2)*(randn(size(txWav2))+1j*randn(size(txWav2)));
rxGrid2 = nrOFDMDemodulate(carrier, txWav2 + noise2);
hId     = ones(12*carrier.NSizeGrid, 14);
[pr2, ph2] = nrExtractResources(pdschIndices, rxGrid2(:,:,1), hId);
[eq2, csi2] = nrEqualizeMMSE(pr2, ph2, N0);
llrs2 = nrSymbolDemodulate(eq2, pdsch.Modulation, N0);
if ~isempty(csi2)
    llrs2 = llrs2 .* repelem(csi2(:), qm);
end
reset(decoder);
[rxBits2, blkErr2] = decoder(llrs2, pdsch.Modulation, pdsch.NumLayers, 0);
fprintf('blkErr=%d   bits_match=%d\n', blkErr2, isequal(rxBits2, trBlk));
