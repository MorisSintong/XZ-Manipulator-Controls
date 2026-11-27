function summary = analyze_as5600_noise(eventsPath, outDir)
% ANALYZE_AS5600_NOISE  Analyze uniform-cadence AS5600 noise captures (enc_noise / as5600_tuner).
%
%   summary = analyze_as5600_noise(eventsPath)
%   summary = analyze_as5600_noise(eventsPath, outDir)
%
% Reads NDJSON `events.jsonl` emitted by:
%   - `motion-characterize enc-noise ...` (`MotionFirmware_Characterization`)
%   - `MotionFirmware_AS5600Tuner` (`STREAM` / `CAPTURE` commands)
%
% For each trial (`test == "enc_noise"`), computes:
%   - Mean, RMS (std), peak-to-peak counts and degrees (0.087890625 deg/count)
%   - Single-sided FFT / PSD (Hann window) and dominant noise frequency
%   - Candidate 1st-order IIR (EMA) / 2nd-order Butterworth software filter
%     attenuation vs. phase lag tradeoff at the capture sample rate
%   - Automatic recommendation: whether AS5600 hardware SF is sufficient or
%     digital attenuation / EMI mitigation is warranted
%
% Headless CLI usage (from repository root):
%   matlab -batch "addpath(genpath('MotorControls/matlab')); analyze_as5600_noise('docs/testing/records/<run>/events.jsonl', 'MotorControls/matlab/figures/<run>')"

arguments
    eventsPath (1,1) string
    outDir (1,1) string = ""
end

if ~isfile(eventsPath)
    error("analyze_as5600_noise:FileNotFound", "Events file not found: %s", eventsPath);
end

if outDir == ""
    outDir = fullfile(fileparts(eventsPath), "matlab_as5600_analysis");
end
if ~isfolder(outDir)
    mkdir(outDir);
end

DEG_PER_COUNT = 360.0 / 4096.0;
USTEPS_PER_COUNT = 3200.0 / 4096.0;

lines = readlines(eventsPath);
sampleMap = containers.Map("KeyType", "int32", "ValueType", "any");
resultMap = containers.Map("KeyType", "int32", "ValueType", "any");

for i = 1:numel(lines)
    ln = strtrim(lines(i));
    if strlength(ln) == 0
        continue;
    end
    try
        obj = jsondecode(char(ln));
    catch
        continue;
    end
    if ~isstruct(obj) || ~isfield(obj, "kind") || ~isfield(obj, "test")
        continue;
    end
    if ~strcmp(obj.test, "enc_noise")
        continue;
    end
    tid = int32(obj.trial_id);
    if strcmp(obj.kind, "sample") && isfield(obj, "unwrapped")
        if ~isKey(sampleMap, tid)
            sampleMap(tid) = double(obj.unwrapped);
        else
            arr = sampleMap(tid);
            arr(end+1, 1) = double(obj.unwrapped); %#ok<AGROW>
            sampleMap(tid) = arr;
        end
    elseif strcmp(obj.kind, "result") && isfield(obj, "rate_hz") && isfield(obj, "sf") && isfield(obj, "n") && ~isfield(obj, "reason")
        resultMap(tid) = obj;
    end
end
resultList = values(resultMap);

if isempty(resultList)
    error("analyze_as5600_noise:NoData", "No enc_noise result records found in %s", eventsPath);
end

nTrials = numel(resultList);
rows = repmat(struct( ...
    "trial_id", 0, "axis", "", "sf", 0, "rate_hz", 0, "hold", false, ...
    "n", 0, "agc", 0, "mag", 0, "status", 0, ...
    "rms_cnt", NaN, "rms_deg", NaN, "rms_usteps", NaN, ...
    "p2p_cnt", NaN, "p2p_deg", NaN, ...
    "dom_freq_hz", NaN, "dom_amp_cnt", NaN, ...
    "iir_fc_hz", NaN, "iir_rms_cnt", NaN, "iir_lag_ms", NaN, ...
    "recommendation", ""), nTrials, 1);
valid = false(nTrials, 1);

fig = figure("Visible", "off", "Color", "w", "Position", [100 100 1100 720]);
tiledlayout(fig, 2, 1, "TileSpacing", "compact", "Padding", "compact");
axTime = nexttile; hold(axTime, "on"); grid(axTime, "on");
axSpec = nexttile; hold(axSpec, "on"); grid(axSpec, "on");

for k = 1:nTrials
    res = resultList{k};
    tid = int32(res.trial_id);
    if ~isKey(sampleMap, tid)
        continue;
    end
    x = sampleMap(tid);
    x = x - mean(x);
    N = numel(x);
    fs = double(res.rate_hz);
    t_ms = (0:N-1).' * (1000.0 / fs);

    rmsCnt = std(x);
    p2pCnt = max(x) - min(x);

    % Single-sided Hann-windowed amplitude spectrum (no toolbox dependency)
    win = 0.5 * (1 - cos(2 * pi * (0:N-1).' / max(1, N - 1)));
    cg = sum(win) / N;
    X = abs(fft(x .* win)) / (N * cg);
    nHalf = floor(N / 2) + 1;
    P1 = X(1:nHalf);
    if nHalf > 2
        P1(2:end-1) = 2 * P1(2:end-1);
    end
    fHz = (0:nHalf-1).' * (fs / N);

    % Dominant non-DC tone
    if numel(P1) > 1
        [domAmp, domIdx] = max(P1(2:end));
        domFreq = fHz(domIdx + 1);
    else
        domAmp = 0; domFreq = 0;
    end

    % Candidate 1st-order EMA low-pass at fc = 25 Hz (alpha = dt / (rc + dt))
    fc = min(25.0, 0.25 * fs);
    dt = 1.0 / fs;
    rc = 1.0 / (2 * pi * fc);
    alpha = dt / (rc + dt);
    y = zeros(N, 1);
    for n = 2:N
        y(n) = y(n-1) + alpha * (x(n) - y(n-1));
    end
    iirRmsCnt = std(y(max(1, round(0.1*N)):end));
    iirLagMs = rc * 1000.0;

    % Heuristic engineering recommendation
    if p2pCnt <= 2.0 && rmsCnt <= 0.6
        rec = "Hardware SF sufficient; no software filter needed";
    elseif domAmp > 0.5 * rmsCnt && domFreq >= 40.0
        rec = sprintf("Narrowband tone at %.1f Hz (likely EMI/chopper): check grounding/shielding or add notch/SF>=8x", domFreq);
    elseif rmsCnt > 1.0
        rec = "Elevated broadband noise: inspect magnet airgap (target AGC~64 at 3.3V) or increase SF to 8x/16x";
    else
        rec = "Moderate noise: compare SF=4x/8x hardware filter before adding software IIR";
    end

    lbl = sprintf("%s SF=%dx hold=%d (RMS=%.2f cnt, p2p=%.0f)", ...
        string(res.axis), res.sf, logical(res.hold), rmsCnt, p2pCnt);
    plot(axTime, t_ms, x, "DisplayName", lbl);
    plot(axSpec, fHz(2:end), P1(2:end), "DisplayName", lbl);

    rows(k).trial_id = double(tid);
    rows(k).axis = string(res.axis);
    rows(k).sf = double(res.sf);
    rows(k).rate_hz = fs;
    rows(k).hold = logical(res.hold);
    rows(k).n = N;
    rows(k).agc = double(res.agc);
    rows(k).mag = double(res.mag);
    rows(k).status = double(res.status);
    rows(k).rms_cnt = rmsCnt;
    rows(k).rms_deg = rmsCnt * DEG_PER_COUNT;
    rows(k).rms_usteps = rmsCnt * USTEPS_PER_COUNT;
    rows(k).p2p_cnt = p2pCnt;
    rows(k).p2p_deg = p2pCnt * DEG_PER_COUNT;
    rows(k).dom_freq_hz = domFreq;
    rows(k).dom_amp_cnt = domAmp;
    rows(k).iir_fc_hz = fc;
    rows(k).iir_rms_cnt = iirRmsCnt;
    rows(k).iir_lag_ms = iirLagMs;
    rows(k).recommendation = rec;
    valid(k) = true;
end

rows = rows(valid);
if isempty(rows)
    error("analyze_as5600_noise:NoData", "No valid completed enc_noise captures in %s", eventsPath);
end

xlabel(axTime, "Time (ms)");
ylabel(axTime, "Zero-Mean Angle (counts)");
title(axTime, "AS5600 Stationary Noise Time Series");
legend(axTime, "Location", "best");

xlabel(axSpec, "Frequency (Hz)");
ylabel(axSpec, "Single-Sided Amplitude (counts)");
title(axSpec, "AS5600 Noise Spectrum (Hann FFT)");
legend(axSpec, "Location", "best");

exportgraphics(fig, fullfile(outDir, "as5600_noise_fft.png"), "Resolution", 200);
close(fig);

T = struct2table(rows);
writetable(T, fullfile(outDir, "as5600_noise_summary.csv"));

summary = struct("source", eventsPath, "out_dir", outDir, "trials", rows);
fid = fopen(fullfile(outDir, "as5600_noise_summary.json"), "w");
fwrite(fid, jsonencode(summary, "PrettyPrint", true), "char");
fclose(fid);

disp(T(:, ["axis", "sf", "hold", "agc", "rms_cnt", "p2p_cnt", "dom_freq_hz", "recommendation"]));
end
