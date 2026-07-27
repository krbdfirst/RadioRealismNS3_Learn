%% debug_bler_chain.m — surgical codec diagnostics
clear; clc;

carrier = nrCarrierConfig;
carrier.NSizeGrid = 25;
carrier.SubcarrierSpacing = 30;
carrier.NSlot = 0;

pdsch = nrPDSCHConfig;
pdsch.Modulation = 'QPSK';
pdsch.NumLayers  = 1;
pdsch.SymbolAllocation = [0 14];
pdsch.PRBSet = 0:(carrier.NSizeGrid-1);
pdsch.RNTI = 1; pdsch.NID = 1;

targetCodeRate = 490/1024;
[pdschIndices, pdschInfo] = nrPDSCHIndices(carrier, pdsch);
dmrsIndices = nrPDSCHDMRSIndices(carrier, pdsch);
G   = pdschInfo.G;
tbs = nrTBS(pdsch.Modulation, pdsch.NumLayers, numel(pdsch.PRBSet), pdschInfo.NREPerPRB, targetCodeRate);

encoder = nrDLSCH; encoder.TargetCodeRate = targetCodeRate;
decoder = nrDLSCHDecoder;
decoder.TargetCodeRate = targetCodeRate;
decoder.TransportBlockLength = tbs;
decoder.MaximumLDPCIterationCount = 12;

fprintf('TBS=%d  G=%d  PRBs=%d\n', tbs, G, numel(pdsch.PRBSet));

%% Test A: encoder → perfect LLRs → decoder (no RF at all)
fprintf('\n── Test A: perfect LLRs ─────────────────────────────────────\n');
trBlk = randi([0 1], tbs, 1);
setTransportBlock(encoder, trBlk, 0);
codedBits = encoder(pdsch.Modulation, pdsch.NumLayers, G, 0);
llrs_perfect = (1 - 2*double(codedBits)) * 100;   % +100 for bit=0, -100 for bit=1 (MATLAB LLR convention)
reset(decoder);
[rxBits, crcErr] = decoder(llrs_perfect, pdsch.Modulation, pdsch.NumLayers, 0);
fprintf('  CRC error=%d   bits_match=%d\n', crcErr, isequal(rxBits, trBlk));

%% Test B: OFDM roundtrip — no noise, no channel
fprintf('\n── Test B: OFDM roundtrip (no noise, no channel) ────────────\n');
setTransportBlock(encoder, trBlk, 0);
codedBits = encoder(pdsch.Modulation, pdsch.NumLayers, G, 0);
txGrid = zeros(12*carrier.NSizeGrid, 14, 1);
txGrid(pdschIndices) = nrPDSCH(carrier, pdsch, codedBits);
txGrid(dmrsIndices)  = nrPDSCHDMRS(carrier, pdsch);
txWav   = nrOFDMModulate(carrier, txGrid);
rxGrid0 = nrOFDMDemodulate(carrier, txWav);
residual = mean(abs(rxGrid0(pdschIndices) - txGrid(pdschIndices)).^2);
fprintf('  OFDM roundtrip residual power = %.2e  (should be ~0)\n', residual);

%% Test C: AWGN only — SNR sweep (nrSymbolModulate / nrSymbolDemodulate)
% Bypasses nrPDSCH / nrPDSCHDecode which have a version-specific sign
% inversion.  nrSymbolDemodulate returns positive LLR = bit 0, matching
% nrDLSCHDecoder's convention.  No OFDM needed for this AWGN baseline.
fprintf('\n── Test C: AWGN only — SNR sweep ───────────────────────────\n');
for snrDb = [0 5 10 15 20 30 40]
    N0 = 1/10^(snrDb/10);
    nErr = 0;
    for t = 1:40
        setTransportBlock(encoder, trBlk, 0);
        cb = encoder(pdsch.Modulation, pdsch.NumLayers, G, 0);
        txSym = nrSymbolModulate(cb, pdsch.Modulation);
        noise = sqrt(N0/2)*(randn(size(txSym))+1j*randn(size(txSym)));
        llrs = nrSymbolDemodulate(txSym + noise, pdsch.Modulation, N0);
        if t==1 && snrDb==20
            fprintf('  [diag snr=20] N0=%.4f  mean|llr|=%.2f  max|llr|=%.2f\n', ...
                N0, mean(abs(llrs)), max(abs(llrs)));
        end
        reset(decoder);
        [~, blkErr] = decoder(llrs, pdsch.Modulation, pdsch.NumLayers, 0);
        nErr = nErr + double(blkErr);
    end
    fprintf('  SNR=%3d dB   BLER=%.3f\n', snrDb, nErr/40);
end

%% Test D: LLR sign check at 20 dB (nrSymbolDemodulate)
fprintf('\n── Test D: LLR sign check at 20 dB ─────────────────────────\n');
snrDb = 20; N0 = 1/10^(snrDb/10);
setTransportBlock(encoder, trBlk, 0);
cb = encoder(pdsch.Modulation, pdsch.NumLayers, G, 0);
txSym = nrSymbolModulate(cb, pdsch.Modulation);
noise = sqrt(N0/2)*(randn(size(txSym))+1j*randn(size(txSym)));
llrs = nrSymbolDemodulate(txSym + noise, pdsch.Modulation, N0);

reset(decoder);
[~, e1] = decoder(llrs, pdsch.Modulation, pdsch.NumLayers, 0);
fprintf('  Decode (nrSymbolDemodulate): crcErr=%d  (expect 0)\n', e1);

% Sign sanity — MATLAB standard convention: positive LLR = bit 0.
% So: bit=0 → LLR>0 → llr_sign=1, cb_double=0  → cb==llr_sign is FALSE.
%     bit=1 → LLR<0 → llr_sign=0, cb_double=1  → cb==llr_sign is FALSE.
% When LLRs are perfectly correct, every comparison is FALSE → match = 0.000.
% match ≈ 0.000 is PERFECT.  match ≈ 0.500 is random.  match ≈ 1.000 is inverted.
cb_double = double(cb);
llr_sign  = double(llrs > 0);   % 1 where LLR > 0 (predicts bit = 0)
match_frac = mean(cb_double == llr_sign);
fprintf('  LLR sign check: %.3f  (0.000 = perfect, 0.500 = random, 1.000 = inverted)\n', match_frac);
