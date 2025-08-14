%% 5G NR Multicast Simulation with Local UPF in gNB (R2024b Compatible)
%
% This script models a mission‑critical multicast scenario with 10 UEs
% connected to a single gNB.  Uplink transmissions use unicast PUSCH and
% downlink transmissions use multicast PDSCH.  The User Plane Function
% (UPF) is assumed to be located either in the gNB (local UPF) or in
% the core network.  The code compares the resulting latencies and
% throughputs when the UPF is local versus in the core.
%
% IMPORTANT IMPLEMENTATION DETAILS
%  - networkTrafficOnOff: The DataRate property is specified in Kbps and
%    PacketSize in bytes【591221525365089†L185-L190】【591221525365089†L197-L200】.  We set GeneratePacket=true
%    and capture the inter‑arrival times (dt) and packets on demand in
%    the simulation loop.
%  - nrPUSCH/nrPDSCH codeword padding: The length of each codeword must
%    be divisible by the product of the number of layers and the number
%    of bits per modulation symbol.  If this condition is not met,
%    nrLayerMap will throw an error.  We therefore pad each codeword
%    with zeros until its length is a multiple of NumLayers*bitsPerSym.
%  - Antenna configuration for the CDL channel: The nrCDLChannel object
%    uses structures TransmitAntennaArray and ReceiveAntennaArray to
%    specify the number of antennas【420221036790142†L688-L706】.  We update the Size field
%    ([M N P Mg Ng]) so that the product M*N*P*Mg*Ng equals the desired
%    number of transmit or receive antennas.  For a single‑column array
%    with n elements we use [n 1 1 1 1].
%
% NOTE: This script is designed for MATLAB R2024b with the 5G Toolbox
% and Communications Toolbox installed.  It has not been executed in
% this environment, so please run it locally and verify the results.

clear; close all; clc;

%% Simulation Parameters
numUEs       = 100;        % Number of UEs
simSlots     = 1000;      % Simulation time in slots
slotDur      = 0.5e-3;    % Slot duration (0.5 ms for 30 kHz SCS)
SCS          = 30;        % Subcarrier spacing (kHz)
numRBs       = 100;       % Number of resource blocks
carrierFreq  = 3.5e9;     % Carrier frequency (Hz)
bandwidth    = 100e6;     % Bandwidth (Hz)
maxLayers    = 2;         % Maximum number of MIMO layers
modulation   = '64QAM';   % Modulation scheme
packetBits   = 1002;      % Packet size in bits (for downlink multicast)
packetBytes  = ceil(packetBits/8); % Packet size in bytes (for traffic model)

%% Carrier Configuration
carrier = nrCarrierConfig;
carrier.SubcarrierSpacing = SCS;
carrier.NSizeGrid         = numRBs;
carrier.CyclicPrefix      = 'Normal';

%% gNB and UE Definitions
gNB.NumAntennas = 4;
gNB.Position    = [0 0 30];  % 30 m high tower

UEs = cell(1,numUEs);
for ueIdx = 1:numUEs
    % Random distribution of UEs within 100 m radius on ground plane
    angle = 2*pi*rand;
    radius = 100*sqrt(rand);
    x = radius*cos(angle);
    y = radius*sin(angle);
    UEs{ueIdx}.Position    = [x y 0];
    UEs{ueIdx}.NumAntennas = 2;
    UEs{ueIdx}.RNTI        = ueIdx;
end

%% Traffic Generator Setup
% Each UE generates bursty traffic with an on/off pattern.  DataRate
% specifies throughput in Kbps; PacketSize is in bytes; GeneratePacket
% causes the generator to return the actual packet data.
trafficGen   = cell(1,numUEs);
dtRemaining  = zeros(1,numUEs);  % Time remaining until next packet arrives (seconds)
nextPacket   = cell(1,numUEs);   % Next packet for each UE (bit column vector)
for ueIdx = 1:numUEs
    tg = networkTrafficOnOff('OnTime',0.01, 'OffTime',0.09, ...
                             'DataRate',1000, 'PacketSize',packetBytes, ...
                             'GeneratePacket',true);
    trafficGen{ueIdx} = tg;
    % Prime generator: get first inter-arrival time and packet
    [dt,pktSize,pkt] = generate(tg);
    dtRemaining(ueIdx) = dt;     % dt is returned in seconds
    % Convert packet (uint8 array) into a column vector of bits
    bits = de2bi(pkt,8,'left-msb')';
    nextPacket{ueIdx} = bits(:);
end

%% Channel Model Configuration
ofdmInfo = nrOFDMInfo(carrier);
baseChannel = nrCDLChannel;
baseChannel.DelayProfile        = 'CDL-D';
baseChannel.CarrierFrequency    = carrierFreq;
baseChannel.MaximumDopplerShift = 10;
baseChannel.SampleRate          = ofdmInfo.SampleRate;

%% Metric Storage
throughputLocal = zeros(1,simSlots);
throughputCore  = zeros(1,simSlots);
latencyLocal    = zeros(1,simSlots);
latencyCore     = zeros(1,simSlots);

%% Utility: modulation order lookup
function bitsPerSym = getBitsPerSym(modulation)
    switch lower(modulation)
        case 'qpsk'
            bitsPerSym = 2;
        case '16qam'
            bitsPerSym = 4;
        case '64qam'
            bitsPerSym = 6;
        case '256qam'
            bitsPerSym = 8;
        otherwise
            error('Unsupported modulation: %s', modulation);
    end
end

%% Simulation Loop
for slot = 1:simSlots
    % Update slot index in carrier configuration
    carrier.NSlot = mod(slot-1, carrier.SlotsPerFrame);

    %% Uplink: transmit unicast packets from UEs to gNB
    for ueIdx = 1:numUEs
        % Decrement time remaining until next packet
        dtRemaining(ueIdx) = dtRemaining(ueIdx) - slotDur;
        % Transmit packets while dtRemaining <= 0 (may be multiple packets)
        while dtRemaining(ueIdx) <= 0
            % Current packet bits for this UE
            ulBits = nextPacket{ueIdx};
            % Configure PUSCH
            pusch = nrPUSCHConfig;
            pusch.Modulation = modulation;
            % Number of layers limited by available antennas
            pusch.NumLayers = min(maxLayers, min(gNB.NumAntennas, UEs{ueIdx}.NumAntennas));
            pusch.PRBSet     = 0:numRBs-1;
            pusch.RNTI       = UEs{ueIdx}.RNTI;
            % Determine bits per symbol for PUSCH modulation
            bitsPerSymUL = getBitsPerSym(pusch.Modulation);
            padReqUL     = pusch.NumLayers * bitsPerSymUL;
            remUL        = mod(numel(ulBits), padReqUL);
            if remUL > 0
                ulBits = [ulBits; zeros(padReqUL - remUL, 1)];
            end
            % Generate PUSCH symbols
            txUL = nrPUSCH(carrier, pusch, ulBits);
            % Update channel antenna configuration: UE antennas -> gNB antennas
            % Release the System object before changing non-tunable properties
            release(baseChannel);
            baseChannel.TransmitAntennaArray.Size = [pusch.NumLayers 1 1 1 1];
            baseChannel.ReceiveAntennaArray.Size  = [gNB.NumAntennas 1 1 1 1];
            baseChannel.Seed = slot + ueIdx;
            % Pass UL waveform through channel
            rxUL = baseChannel(txUL);
            % Decode with zero noise variance
            [cwUL,~] = nrPUSCHDecode(carrier,pusch,rxUL,0);
            % Hard decision bits (not used further in this example)
            % Generate next arrival time and packet
            [dtNew,pktSize,pkt] = generate(trafficGen{ueIdx});
            dtRemaining(ueIdx) = dtRemaining(ueIdx) + dtNew;
            bits = de2bi(pkt,8,'left-msb')';
            nextPacket{ueIdx} = bits(:);
        end
    end

    %% Downlink: multicast aggregated packet from gNB to all UEs
    % Generate a random multicast packet of packetBits bits
    dlBits = randi([0 1], packetBits, 1);
    % Configure PDSCH
    pdsch = nrPDSCHConfig;
    pdsch.Modulation = modulation;
    pdsch.NumLayers  = min(maxLayers, gNB.NumAntennas);
    pdsch.PRBSet     = 0:numRBs-1;
    % Determine bits per symbol and padding requirement
    bitsPerSymDL = getBitsPerSym(pdsch.Modulation);
    padReqDL     = pdsch.NumLayers * bitsPerSymDL;
    remDL        = mod(numel(dlBits), padReqDL);
    if remDL > 0
        dlBits = [dlBits; zeros(padReqDL - remDL, 1)];
    end
    % Generate PDSCH symbols
    txDL = nrPDSCH(carrier, pdsch, dlBits);
    % Set base processing latency (ms)
    baseLatency = slotDur * 1000;
    latencyLocal(slot) = baseLatency;
    latencyCore(slot)  = baseLatency + 2+5+3;  % Additional Radio channel loss=1 5+2 ms for path and core
    % Send to each UE
    for ueIdx = 1:numUEs
        % Release the System object before changing non-tunable properties
        release(baseChannel);
        baseChannel.TransmitAntennaArray.Size = [pdsch.NumLayers 1 1 1 1];
        baseChannel.ReceiveAntennaArray.Size  = [UEs{ueIdx}.NumAntennas 1 1 1 1];
        baseChannel.Seed = slot + ueIdx + 1000;
        % Pass through channel and decode
        rxDL = baseChannel(txDL);
        % Decode PDSCH; cwDL may be returned as a cell array if multiple codewords
        [cwDL,~] = nrPDSCHDecode(carrier,pdsch,rxDL,0);
        % Select the first codeword if cwDL is a cell array; otherwise use cwDL directly
        if iscell(cwDL)
            cwTemp = cwDL{1};
        else
            cwTemp = cwDL;
        end
        rxBitsDL = double(cwTemp < 0);
        % Count throughput only for original packet length (packetBits)
        if isequal(rxBitsDL(1:packetBits), dlBits(1:packetBits))
            throughputLocal(slot) = throughputLocal(slot) + packetBits * 1000 / slotDur;
        end
    end
    throughputCore(slot) = throughputLocal(slot);  % Same throughput for fairness
end

%% Results
avgThroughputLocal = mean(throughputLocal) / 1e6;  % Mbps
avgThroughputCore  = mean(throughputCore)  / 1e6;
avgLatencyLocal    = mean(latencyLocal);
avgLatencyCore     = mean(latencyCore);

% Display summary
disp(['Average Throughput (Local-breakout): ', num2str(avgThroughputLocal), ' Mbps']);
disp(['Average Throughput (Core-anchored):  ', num2str(avgThroughputCore),  ' Mbps']);
disp(['Average Latency (Local UPF):    ', num2str(avgLatencyLocal),    ' ms']);
disp(['Average Latency (Core UPF):     ', num2str(avgLatencyCore),     ' ms']);

%% Plotting
% Network architecture comparison
figure('Name','Network Architecture Comparison','Position',[100 100 1200 400]);
subplot(1,2,1);
plot(gNB.Position(1), gNB.Position(2), 'r^', 'MarkerSize', 10, 'DisplayName','gNB with Local UPF');
hold on;
for ueIdx = 1:numUEs
    plot(UEs{ueIdx}.Position(1), UEs{ueIdx}.Position(2), 'bo', 'DisplayName', sprintf('UE %d', ueIdx));
end
title('Proposed Architecture (UPF in gNB)');
xlabel('X (m)'); ylabel('Y (m)'); grid on;
if numUEs > 1
    legend('show', 'NumColumns', 2);
end

subplot(1,2,2);
plot(gNB.Position(1), gNB.Position(2), 'r^', 'MarkerSize', 10, 'DisplayName', 'gNB');
hold on;
plot(50, 50, 'gs', 'MarkerSize', 10, 'DisplayName','Core UPF');
for ueIdx = 1:numUEs
    plot(UEs{ueIdx}.Position(1), UEs{ueIdx}.Position(2), 'bo', 'DisplayName', sprintf('UE %d', ueIdx));
end
title('Traditional Architecture (UPF in Core)');
xlabel('X (m)'); ylabel('Y (m)'); grid on;
if numUEs > 1
    legend('show', 'NumColumns', 2);
end
saveas(gcf,'architecture_comparison.png');

% Latency comparison
figure('Name','Latency Comparison','Position',[100 100 600 400]);
bar([avgLatencyLocal, avgLatencyCore]);
set(gca,'XTickLabel',{'Local-breakout','Core-anchored'});
title('Average Latency'); ylabel('Latency (ms)'); grid on;
saveas(gcf,'latency_comparison.png');

% Throughput comparison
figure('Name','Throughput Comparison','Position',[100 100 600 400]);
bar([avgThroughputLocal, avgThroughputCore]);
set(gca,'XTickLabel',{'Local-breakout','Core-anchored'});
title('Average Throughput'); ylabel('Throughput (Mbps)'); grid on;
saveas(gcf,'throughput_comparison.png');

%% Save simulation logs
simulationLogs = struct('ThroughputLocal',throughputLocal, 'ThroughputCore',throughputCore, ...
                        'LatencyLocal',latencyLocal, 'LatencyCore',latencyCore);
save('simulation_logs.mat','simulationLogs');



%Added code
%% ----------------------------------------
% Latency Distribution Box Plot
% Compare the per‑slot latency samples for local breakout (gNB UPF)
% versus core‑anchored UPF

figure('Name','Latency Distribution Comparison','Position',[100 100 600 400]);

% Ensure column vectors and remove non-finite values
xLocal = latencyLocal(:);  xLocal = xLocal(isfinite(xLocal));
xCore  = latencyCore(:);   xCore  = xCore(isfinite(xCore));

% Build data + grouping vector (no need for equal lengths)
X = [xLocal; xCore];
G = [repmat({'Local breakout'}, numel(xLocal), 1); ...
     repmat({'Core anchored'},  numel(xCore),  1)];

boxplot(X, G, ...
    'Notch','on', ...
    'Whisker',1.5, ...
    'Labels',{'Local breakout','Core anchored'}, ...
    'Symbol','+');

title('Latency Distribution: Local vs Core','FontSize',16,'FontWeight','bold');
ylabel('Slot Latency (ms)','FontSize',14);
grid on;

ax = gca;
ax.FontSize = 12;     % tick labels
ax.LineWidth = 1.2;

saveas(gcf,'latency_boxplot.png');
