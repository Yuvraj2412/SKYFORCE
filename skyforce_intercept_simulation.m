function skyforce_intercept_simulation
% SKYFORCE MATLAB intercept simulation (2-D point-mass model)
% -------------------------------------------------------------------------
% Implements the scenario in "SKYFORCE - MATLAB Intercept Simulation:
% What It Should Show.pdf" as an illustrative kinematic simulation.
%
% UPDATED TO MATCH: "SKYFORCE - Resolution of Simulation Issues"
% AUTHORITATIVE BASELINE (first-pass U8 II / 22-in / 236 Wh / 4.0 kW data
% is withdrawn): U8 Lite KV100, G28x9.2 CF 28-in propeller, Alpha 60A 12S ESC,
% 18.0 kg nominal mass, 23.5 kg post-capture mass, 1.56 kW hover, 3.61 kW
% sprint, 207 Wh no-tow sizing budget, and 253 Wh with-tow sizing budget.
%
% IMPORTANT LIMITATIONS
% - This is a 2-D point-mass scenario simulation, not flight-controller code.
% - The 5-15 m event is a geometric capture candidate, not net deployment or
%   proof of a successful physical capture.
% - Static-thrust acceleration and speed checks below are estimates based on
%   the guide-review note. Forward-flight propeller performance, detailed
%   electrical losses, wind, sensor errors, obstacles, structural dynamics,
%   parachute dynamics and actual capture probability are not modelled.
% - The 3.22 kg mass allowance is explicitly represented but still needs to be
%   closed with actual items; propeller and ESC weights remain unverified.
% - Keep-out/safe-area circles and example tracks are illustrative, not
%   approved operational boundaries.
% - A fifth, in-flight offset-heading scenario is a guidance diagnostic, not a
%   launch-to-landing mission; it is used to show the PN correction explicitly.
% - Exports a scenario summary CSV and a sample-by-sample trajectory CSV with
%   exact capture-event state, LOS, range rate, closing speed, control limits,
%   PN activity, phase, power estimate, and cumulative energy.
%
% MATLAB R2018b or newer is expected (local functions, xline/yline, sgtitle).

clc;
close all;

%% Authoritative baseline from the guide-review resolution PDF
% The earlier U8 II / 22-inch / 9.2 kgf / 4.0 kW / 236 Wh baseline is withdrawn.
cfg.dt                    = 0.05;       % s, numerical integration step
cfg.maxSimTime            = 240;        % s, maximum simulation horizon
cfg.reactionDelay         = 1.8;        % s, stacked working assumption (1.5-2 s)
cfg.maxSpeed              = 25;         % m/s, selected design/control cap
cfg.g                     = 9.81;       % m/s^2
cfg.PNgain                = 3;          % proportional-navigation gain, N = 3
cfg.captureRange          = [5, 15];    % m, unvalidated geometric engagement window
cfg.massNominal           = 18.00;      % kg, authoritative nominal sizing mass
cfg.massWithCaptured      = 23.50;      % kg, authoritative post-capture mass
cfg.massLedgerSubtotal    = 14.78;      % kg, component ledger subtotal in resolution note
cfg.massGrowthAllowance   = 3.22;       % kg, named integration/growth allowance
cfg.motorName             = 'T-Motor U8 Lite KV100';
cfg.propellerName         = 'T-Motor G28x9.2 CF';
cfg.escName               = 'T-Motor Alpha 60A, 12S';
cfg.numMotors             = 6;
cfg.motorMassEach_kg      = 0.238;      % kg, published motor mass including cable
cfg.maxThrustPerMotor_kgf = 7.3;        % kgf, published standard-variant figure
cfg.propDiameter_in       = 28;         % in, 28-inch G28x9.2 propeller
cfg.armLength_m           = 0.800;      % m, centre-to-motor length from resolution note
cfg.frameSpan_m           = 2.31;       % m, overall frame span from resolution note
cfg.batteryCapacity_Ah    = 8.0;        % Ah, 8000 mAh pack
cfg.batteryMass_kg        = 1.97;       % kg, 8000 mAh pack estimate
cfg.batteryVoltageNom_V   = 44.4;       % V, 12S nominal voltage
cfg.batteryNominalEnergyWh = cfg.batteryCapacity_Ah * cfg.batteryVoltageNom_V; % 355.2 Wh
cfg.batteryUsableFraction = 0.80;       % only 80% of rated capacity counted as usable
cfg.batteryUsableEnergyWh = cfg.batteryNominalEnergyWh * cfg.batteryUsableFraction; % 284.16 Wh
cfg.hoverPower            = 1.56e3;     % W, corrected propulsion baseline
cfg.sprintPower           = 3.61e3;     % W, corrected sprint power at 25 m/s
cfg.energyDemandMargin    = 0.20;       % 20% demand-side sizing margin
cfg.sizingHoverDuration_s = 120;        % s, sizing-profile hover-equivalent duration
cfg.sizingSprintDuration_s = 120;       % s, sizing-profile sprint duration
cfg.energyBudgetNoTow     = ceil((cfg.hoverPower*cfg.sizingHoverDuration_s + ...
                                  cfg.sprintPower*cfg.sizingSprintDuration_s) / 3600 * ...
                                  (1 + cfg.energyDemandMargin)); % 207 Wh after rounding
cfg.energyBudgetWithTow   = 253;        % Wh, authoritative sizing ceiling with a 60 s tow
cfg.maxTowDuration        = 60;         % s, target tow-phase duration limit
cfg.maxTowSpeed           = 8.0;        % m/s, provisional low-speed tow assumption; guide gives no numeric value
% Guide-approved added-phase buffers. These are estimates, not flight measurements.
cfg.energyBufferNoTow_Wh  = 9.7;        % launch + descent + avionics allowance
cfg.energyBufferTow_Wh    = 18.8;       % launch + loaded descent + avionics allowance
cfg.forwardFlightDeratingHeadline = 0.30; % assumed conservative thrust de-rating
cfg.forwardFlightDeratingSensitivity = [0.15 0.20 0.30]; % assumed, not measured
cfg.workingRangeMin       = 1500;       % m, illustrative 1.5-2 km working-range assumption
cfg.workingRangeMax       = 2000;       % m

%% Derived checks from the resolution note (static estimates, not flight-test data)
cfg.totalMaxThrustN = cfg.numMotors * cfg.maxThrustPerMotor_kgf * cfg.g;
cfg.designTWR = cfg.totalMaxThrustN / (cfg.massNominal * cfg.g);
cfg.weightNominalN = cfg.massNominal * cfg.g;
cfg.weightPostCaptureN = cfg.massWithCaptured * cfg.g;
cfg.airDensity = 1.225;                % kg/m^3, sea-level ISA estimate
cfg.dragCoefficient = 1.2;             % provisional blunt-body estimate
cfg.frontalArea_m2 = 0.15;              % m^2, provisional estimate
cfg.dragFactor_N_per_mps2 = 0.5 * cfg.airDensity * cfg.dragCoefficient * cfg.frontalArea_m2;
cfg.dragAtMaxSpeedN = cfg.dragFactor_N_per_mps2 * cfg.maxSpeed^2;
cfg.maxHorizontalThrustNominalN = sqrt(max(cfg.totalMaxThrustN^2 - cfg.weightNominalN^2, 0));
cfg.maxHorizontalThrustPostCaptureN = sqrt(max(cfg.totalMaxThrustN^2 - cfg.weightPostCaptureN^2, 0));
cfg.staticAccelNominal_mps2 = cfg.maxHorizontalThrustNominalN / cfg.massNominal;
cfg.staticAccelPostCapture_mps2 = cfg.maxHorizontalThrustPostCaptureN / cfg.massWithCaptured;
cfg.requiredThrustAt25NominalN = hypot(cfg.weightNominalN, cfg.dragAtMaxSpeedN);
cfg.requiredThrustAt25PostCaptureN = hypot(cfg.weightPostCaptureN, cfg.dragAtMaxSpeedN);
cfg.thrustMarginAt25Nominal = cfg.totalMaxThrustN / cfg.requiredThrustAt25NominalN;
cfg.thrustMarginAt25PostCapture = cfg.totalMaxThrustN / cfg.requiredThrustAt25PostCaptureN;
cfg.thrustMargin30pctNominal = (1-cfg.forwardFlightDeratingHeadline) * cfg.totalMaxThrustN / cfg.requiredThrustAt25NominalN;
cfg.thrustMargin30pctPostCapture = (1-cfg.forwardFlightDeratingHeadline) * cfg.totalMaxThrustN / cfg.requiredThrustAt25PostCaptureN;
cfg.staticDragBalanceSpeedNominal_mps = sqrt(cfg.maxHorizontalThrustNominalN / cfg.dragFactor_N_per_mps2);
cfg.staticDragBalanceSpeedPostCapture_mps = sqrt(cfg.maxHorizontalThrustPostCaptureN / cfg.dragFactor_N_per_mps2);

%% Provisional scenario/control assumptions — label as illustrative
cfg.accelLimitAssumed   = 4.5;         % m/s^2, deliberately conservative control cap
cfg.thrustAccelBound    = cfg.staticAccelNominal_mps2;
cfg.accelLimit          = min(cfg.accelLimitAssumed, cfg.thrustAccelBound);
cfg.PNswitchRange       = 600;         % m, provisional blend range for terminal PN
cfg.abortTimeMargin     = 2.0;         % s, abort warning margin
cfg.safeAreaRadius      = 120;         % m, illustrative safe-area radius, not approved limit
cfg.waypointTolerance   = 8;           % m, illustrative point-mass waypoint tolerance
cfg.waypointSpeedTol    = 1.0;         % m/s, considered settled near waypoint
cfg.towArrivalTolerance = 12;          % m, tow completion distance

fprintf('\nSKYFORCE corrected sizing baseline\n');
fprintf('  Mass ledger: %.2f kg subtotal + %.2f kg named integration/growth allowance = %.2f kg nominal.\n', ...
    cfg.massLedgerSubtotal, cfg.massGrowthAllowance, cfg.massNominal);
fprintf('  Post-capture mass: %.2f kg. Propeller/ESC weights still need independent confirmation.\n', cfg.massWithCaptured);
fprintf('  Propulsion: %s + %s (%g in) + %s.\n', cfg.motorName, cfg.propellerName, cfg.propDiameter_in, cfg.escName);
fprintf('  Six-motor maximum static thrust estimate: %.1f N; nominal T/W = %.2f.\n', ...
    cfg.totalMaxThrustN, cfg.designTWR);
fprintf('  Static acceleration estimate: %.1f m/s^2 nominal, %.1f m/s^2 post-capture; model cap = %.1f m/s^2.\n', ...
    cfg.staticAccelNominal_mps2, cfg.staticAccelPostCapture_mps2, cfg.accelLimit);
fprintf('  Thrust margin at 25 m/s (static estimate): %.2fx nominal, %.2fx post-capture.\n', ...
    cfg.thrustMarginAt25Nominal, cfg.thrustMarginAt25PostCapture);
fprintf('  Conservative %.0f%% assumed forward-flight de-rating margins: %.2fx nominal / %.2fx post-capture.\n', ...
    100*cfg.forwardFlightDeratingHeadline, cfg.thrustMargin30pctNominal, cfg.thrustMargin30pctPostCapture);
fprintf('  De-rating range [%%]: '); fprintf('%.0f ',100*cfg.forwardFlightDeratingSensitivity); fprintf('(assumed sensitivity, not measured).\n');
fprintf('  Added-phase buffers: %.1f Wh no-tow / %.1f Wh tow; continuous launch-to-landing integration remains a recommended follow-up.\n', ...
    cfg.energyBufferNoTow_Wh, cfg.energyBufferTow_Wh);
fprintf('  Low-speed tow cap: %.1f m/s (provisional assumption; confirm numeric value with guide).\n', cfg.maxTowSpeed);
fprintf('  Drag-balance speed estimates %.1f / %.1f m/s are static idealizations, not verified top speeds.\n', ...
    cfg.staticDragBalanceSpeedNominal_mps, cfg.staticDragBalanceSpeedPostCapture_mps);
fprintf('  Energy sizing ceilings: %d Wh no-tow, %d Wh with tow; corrected baseline retires the old 236 Wh figure.\n', ...
    cfg.energyBudgetNoTow, cfg.energyBudgetWithTow);
fprintf('  Battery estimate: %.1f Ah, %.1f Wh nominal, %.1f Wh counted usable (80%%).\n\n', ...
    cfg.batteryCapacity_Ah, cfg.batteryNominalEnergyWh, cfg.batteryUsableEnergyWh);

%% Four mission scenarios plus one separate PN diagnostic scenario
% Coordinate system is arbitrary, in metres. x/y are horizontal coordinates.
% Keep-out circles, starting geometries, and target speeds below are examples
% because the PDF does not specify actual site coordinates or target tracks.

scenarios(1).name = 'Attack mode - slower target, immediate release';
scenarios(1).pI0 = [-600; -450];
scenarios(1).pT0 = [-1500; 0];
scenarios(1).vT  = [14; 5]; % target velocity vector; speed and heading exported to CSV
scenarios(1).keepOutCenter = [1200; 0];
scenarios(1).keepOutRadius = 300;
scenarios(1).safePoints = [-1050, 150; 500, 400]'; % rows are [x y] centres; transpose => 2-by-N; capture is inside first area
scenarios(1).leadPoint = [];
scenarios(1).expectTow = false;
scenarios(1).vI0 = [0; 0];
scenarios(1).reactionDelayOverride_s = cfg.reactionDelay;
scenarios(1).isGuidanceDiagnostic = false;

scenarios(2).name = 'Defense mode - faster target, lead-point intercept';
scenarios(2).pI0 = [0; 100];
scenarios(2).pT0 = [-1950; 0]; % tests a successful engagement near the 2 km working-range limit
scenarios(2).vT  = [27; 0];
scenarios(2).keepOutCenter = [0; 0];
scenarios(2).keepOutRadius = 300;
scenarios(2).safePoints = [-500, 0; 500, 600]'; % rows are [x y] centres; lead point is inside first area
scenarios(2).leadPoint = [-500; 0]; % outside the illustrative 300 m keep-out radius
scenarios(2).expectTow = false;
scenarios(2).vI0 = [0; 0];
scenarios(2).reactionDelayOverride_s = cfg.reactionDelay;
scenarios(2).isGuidanceDiagnostic = false;

scenarios(3).name = 'Tow required - capture outside safe area';
scenarios(3).pI0 = [0; -400];
scenarios(3).pT0 = [-1200; 300];
scenarios(3).vT  = [12; 0];
scenarios(3).keepOutCenter = [1200; 0];
scenarios(3).keepOutRadius = 300;
scenarios(3).safePoints = [-400, 400; 0, 600]'; % illustrative centres; nearest is reachable at low tow speed
scenarios(3).leadPoint = [];
scenarios(3).expectTow = true;
scenarios(3).vI0 = [0; 0];
scenarios(3).reactionDelayOverride_s = cfg.reactionDelay;
scenarios(3).isGuidanceDiagnostic = false;

scenarios(4).name = 'Abort - target reaches boundary too soon';
scenarios(4).pI0 = [0; -900];
scenarios(4).pT0 = [600; 0];
scenarios(4).vT  = [-20; 0];
scenarios(4).keepOutCenter = [0; 0];
scenarios(4).keepOutRadius = 300;
scenarios(4).safePoints = [0, 600; 600, 600]'; % rows are [x y] centres
scenarios(4).leadPoint = [];
scenarios(4).expectTow = false;
scenarios(4).vI0 = [0; 0];
scenarios(4).reactionDelayOverride_s = cfg.reactionDelay;
scenarios(4).isGuidanceDiagnostic = false;

% Fifth case requested by the guide: deliberately offset initial heading to
% expose proportional-navigation (PN) corrections. This is an in-flight
% guidance diagnostic, not a launch/reaction-delay mission scenario.
scenarios(5).name = 'PN correction diagnostic - offset initial heading (diagnostic only)';
scenarios(5).pI0 = [0; 0];
scenarios(5).pT0 = [1200; 600];
scenarios(5).vI0 = [20; 0];       % initial flight heading 0 deg, 20 m/s
scenarios(5).vT = [0; 12];        % target heading 90 deg; crossing geometry
scenarios(5).keepOutCenter = [5000; 5000];
scenarios(5).keepOutRadius = 300;
scenarios(5).safePoints = [1200; 1550]; % illustrative centre near expected candidate
scenarios(5).leadPoint = [];
scenarios(5).expectTow = false;
scenarios(5).reactionDelayOverride_s = 0;
scenarios(5).isGuidanceDiagnostic = true;

%% Run cases
results = cell(1, numel(scenarios));
for i = 1:numel(scenarios)
    results{i} = simulateOneScenario(scenarios(i), cfg);
    results{i}.scenarioID = i;
    if ~results{i}.isGuidanceDiagnostic
        plotOneScenario(results{i}, cfg);
    end
end
plotTerminalApproachSummary(results, cfg);
plotPNCorrectionDiagnostic(results, cfg);

%% Summary table: mission estimates, exact event state and propulsion sensitivity
n = numel(results);
Scenario = strings(n,1);
ScenarioPurpose = strings(n,1);
ScenarioID = (1:n)';
Mode = strings(n,1);
InitialRange_m = nan(n,1);
TargetSpeed_mps = nan(n,1);
TargetVelocityX_mps = nan(n,1);
TargetVelocityY_mps = nan(n,1);
TargetHeading_deg = nan(n,1);
InitialInterceptorSpeed_mps = nan(n,1);
InitialInterceptorHeading_deg = nan(n,1);
InitialLOSAngle_deg = nan(n,1);
InitialHeadingOffset_deg = nan(n,1);
CaptureSampleIndex = nan(n,1);
CaptureTime_s = nan(n,1);
CaptureCandidateRange_m = nan(n,1);
CaptureInterceptorX_m = nan(n,1); CaptureInterceptorY_m = nan(n,1);
CaptureInterceptorVx_mps = nan(n,1); CaptureInterceptorVy_mps = nan(n,1);
CaptureTargetX_m = nan(n,1); CaptureTargetY_m = nan(n,1);
CaptureTargetVx_mps = nan(n,1); CaptureTargetVy_mps = nan(n,1);
CaptureLOSUnitX = nan(n,1); CaptureLOSUnitY = nan(n,1);
RangeRateAtCandidate_mps = nan(n,1);
ClosingSpeedAtCandidate_mps = nan(n,1);
CandidateGuidanceCommandX_mps2 = nan(n,1); CandidateGuidanceCommandY_mps2 = nan(n,1);
CandidateAppliedAccelerationX_mps2 = nan(n,1); CandidateAppliedAccelerationY_mps2 = nan(n,1);
CandidateActiveLimit = strings(n,1);
CommandLeadingIntoCandidateX_mps2 = nan(n,1); CommandLeadingIntoCandidateY_mps2 = nan(n,1);
AppliedAccelerationIntoCandidateX_mps2 = nan(n,1); AppliedAccelerationIntoCandidateY_mps2 = nan(n,1);
ActiveLimitIntoCandidate = strings(n,1);
PNActiveAtCandidate = false(n,1);
PNAccelerationAtCandidate_mps2 = nan(n,1);
PNPeakAccelerationObserved_mps2 = nan(n,1);
PNActiveDuration_s = nan(n,1);
GuidanceCorrectionDemonstrated = false(n,1);
TimeToCapture_s = nan(n,1);
ClosestRange_m = nan(n,1);
MaxSpeedObserved_mps = nan(n,1);
SpeedWithin25mpsLimit = false(n,1);
MaxAccelerationObserved_mps2 = nan(n,1);
AssumedAccelerationLimit_mps2 = nan(n,1);
AccelerationWithinAssumedLimit = false(n,1);
WithinWorkingRange_1p5to2km = false(n,1);
CaptureWithin5to15m = false(n,1);
EnergyUsed_SimulatedPhases_Wh = nan(n,1);
AdditionalPhaseBuffer_Wh = nan(n,1);
CompleteMissionEnergyEstimate_Wh = nan(n,1);
EnergyBudget_SizingCeiling_Wh = nan(n,1);
CompleteMissionEstimateWithinCeiling = false(n,1);
EnergyBudgetStatus = strings(n,1);
FullMissionEnergyValidated = false(n,1);
MissionOutcome = strings(n,1);
CaptureCandidateRegistered = false(n,1);
CaptureInsideSafeArea = false(n,1);
NearestSafePointDistanceAtCapture_m = nan(n,1);
SafeAreaRadius_m = nan(n,1);
ReleaseTowDecision = strings(n,1);
TowDuration_s = nan(n,1);
TowCompletedWithin60s = false(n,1);
NominalMass_kg = repmat(cfg.massNominal,n,1);
PostCaptureMass_kg = repmat(cfg.massWithCaptured,n,1);
MassLedgerSubtotal_kg = repmat(cfg.massLedgerSubtotal,n,1);
IntegrationGrowthAllowance_kg = repmat(cfg.massGrowthAllowance,n,1);
MotorModel = repmat(string(cfg.motorName),n,1);
PropellerModel = repmat(string(cfg.propellerName),n,1);
ESCModel = repmat(string(cfg.escName),n,1);
PropellerDiameter_in = repmat(cfg.propDiameter_in,n,1);
ArmLength_m = repmat(cfg.armLength_m,n,1);
FrameSpan_m = repmat(cfg.frameSpan_m,n,1);
MaxTotalStaticThrust_N = repmat(cfg.totalMaxThrustN,n,1);
NominalTWR_Static = repmat(cfg.designTWR,n,1);
PostCaptureTWR_Static = repmat(cfg.totalMaxThrustN/cfg.weightPostCaptureN,n,1);
StaticAvailableAccelerationNominal_mps2 = repmat(cfg.staticAccelNominal_mps2,n,1);
StaticAvailableAccelerationPostCapture_mps2 = repmat(cfg.staticAccelPostCapture_mps2,n,1);
ThrustMarginAt25mps_Nominal = repmat(cfg.thrustMarginAt25Nominal,n,1);
ThrustMarginAt25mps_PostCapture = repmat(cfg.thrustMarginAt25PostCapture,n,1);
Conservative30pctThrustMarginNominal = repmat(cfg.thrustMargin30pctNominal,n,1);
Conservative30pctThrustMarginPostCapture = repmat(cfg.thrustMargin30pctPostCapture,n,1);
StaticDragBalanceSpeedNominal_mps = repmat(cfg.staticDragBalanceSpeedNominal_mps,n,1);
StaticDragBalanceSpeedPostCapture_mps = repmat(cfg.staticDragBalanceSpeedPostCapture_mps,n,1);
MotorMassEach_kg = repmat(cfg.motorMassEach_kg,n,1);
MotorSetMass_kg = repmat(cfg.numMotors*cfg.motorMassEach_kg,n,1);
BatteryMass_kg = repmat(cfg.batteryMass_kg,n,1);
BatteryCapacity_Ah = repmat(cfg.batteryCapacity_Ah,n,1);
BatteryNominalEnergy_Wh = repmat(cfg.batteryNominalEnergyWh,n,1);
BatteryUsableEnergy_Wh = repmat(cfg.batteryUsableEnergyWh,n,1);
ProvisionalTowSpeedCap_mps = repmat(cfg.maxTowSpeed,n,1);

for i = 1:n
    r = results{i};
    Scenario(i) = string(r.name);
    if r.isGuidanceDiagnostic
        ScenarioPurpose(i) = "PN guidance diagnostic only; initialized in flight with offset heading";
    else
        ScenarioPurpose(i) = "Mission scenario";
    end
    Mode(i) = string(r.mode);
    InitialRange_m(i) = r.initialRange;
    TargetSpeed_mps(i) = r.targetSpeed;
    TargetVelocityX_mps(i) = r.targetVelocity(1);
    TargetVelocityY_mps(i) = r.targetVelocity(2);
    TargetHeading_deg(i) = atan2d(r.targetVelocity(2), r.targetVelocity(1));
    InitialInterceptorSpeed_mps(i) = norm(r.initialInterceptorVelocity);
    if InitialInterceptorSpeed_mps(i) > 1e-8
        InitialInterceptorHeading_deg(i) = atan2d(r.initialInterceptorVelocity(2), r.initialInterceptorVelocity(1));
        InitialLOSAngle_deg(i) = atan2d(r.initialLOS(2), r.initialLOS(1));
        InitialHeadingOffset_deg(i) = mod(InitialInterceptorHeading_deg(i)-InitialLOSAngle_deg(i)+180,360)-180;
    end
    if isfinite(r.captureSampleIndex)
        cs = r.captureState;
        CaptureSampleIndex(i) = cs.sampleIndex;
        CaptureTime_s(i) = cs.time;
        CaptureCandidateRange_m(i) = cs.range;
        CaptureInterceptorX_m(i) = cs.pI(1); CaptureInterceptorY_m(i) = cs.pI(2);
        CaptureInterceptorVx_mps(i) = cs.vI(1); CaptureInterceptorVy_mps(i) = cs.vI(2);
        CaptureTargetX_m(i) = cs.pT(1); CaptureTargetY_m(i) = cs.pT(2);
        CaptureTargetVx_mps(i) = cs.vT(1); CaptureTargetVy_mps(i) = cs.vT(2);
        CaptureLOSUnitX(i) = cs.los(1); CaptureLOSUnitY(i) = cs.los(2);
        RangeRateAtCandidate_mps(i) = cs.rangeRate;
        ClosingSpeedAtCandidate_mps(i) = cs.closingSpeed;
        CandidateGuidanceCommandX_mps2(i) = cs.commandedAccel(1);
        CandidateGuidanceCommandY_mps2(i) = cs.commandedAccel(2);
        CandidateAppliedAccelerationX_mps2(i) = cs.appliedAccel(1);
        CandidateAppliedAccelerationY_mps2(i) = cs.appliedAccel(2);
        CandidateActiveLimit(i) = string(cs.activeLimit);
        CommandLeadingIntoCandidateX_mps2(i) = cs.commandedAccelIntoCandidate(1);
        CommandLeadingIntoCandidateY_mps2(i) = cs.commandedAccelIntoCandidate(2);
        AppliedAccelerationIntoCandidateX_mps2(i) = cs.appliedAccelIntoCandidate(1);
        AppliedAccelerationIntoCandidateY_mps2(i) = cs.appliedAccelIntoCandidate(2);
        ActiveLimitIntoCandidate(i) = string(cs.activeLimitIntoCandidate);
        PNActiveAtCandidate(i) = cs.pnActive;
        PNAccelerationAtCandidate_mps2(i) = norm(cs.pnAcceleration);
    end
    TimeToCapture_s(i) = r.captureTime;
    ClosestRange_m(i) = r.minimumRange;
    MaxSpeedObserved_mps(i) = r.maxSpeedObserved;
    SpeedWithin25mpsLimit(i) = r.maxSpeedObserved <= cfg.maxSpeed + 1e-6;
    MaxAccelerationObserved_mps2(i) = r.maxAccelerationObserved;
    AssumedAccelerationLimit_mps2(i) = cfg.accelLimit;
    AccelerationWithinAssumedLimit(i) = r.maxAccelerationObserved <= cfg.accelLimit + 0.05;
    WithinWorkingRange_1p5to2km(i) = r.initialRange >= cfg.workingRangeMin && r.initialRange <= cfg.workingRangeMax;
    CaptureWithin5to15m(i) = isfinite(r.captureRangeAtEvent) && ...
        r.captureRangeAtEvent >= cfg.captureRange(1) && r.captureRangeAtEvent <= cfg.captureRange(2);
    EnergyUsed_SimulatedPhases_Wh(i) = r.energyWh(end);
    EnergyBudget_SizingCeiling_Wh(i) = r.energyBudget;
    PNPeakAccelerationObserved_mps2(i) = max(r.pnAccelerationMagnitude);
    PNActiveDuration_s(i) = sum(r.pnActive) * cfg.dt;
    GuidanceCorrectionDemonstrated(i) = r.isGuidanceDiagnostic && ...
        any(r.pnActive) && max(r.pnAccelerationMagnitude) > 0.1;
    MissionOutcome(i) = string(r.outcome);
    CaptureCandidateRegistered(i) = isfinite(r.captureTime);
    CaptureInsideSafeArea(i) = r.captureInsideSafeArea;
    NearestSafePointDistanceAtCapture_m(i) = r.nearestSafePointDistanceAtCapture;
    SafeAreaRadius_m(i) = cfg.safeAreaRadius;
    ReleaseTowDecision(i) = string(r.releaseTowDecision);
    TowDuration_s(i) = r.towDuration;
    TowCompletedWithin60s(i) = r.towCompleted && isfinite(r.towDuration) && r.towDuration <= cfg.maxTowDuration + cfg.dt;

    if r.isGuidanceDiagnostic
        AdditionalPhaseBuffer_Wh(i) = NaN;
        CompleteMissionEnergyEstimate_Wh(i) = NaN;
        CompleteMissionEstimateWithinCeiling(i) = false;
        EnergyBudgetStatus(i) = "Not applicable: guidance diagnostic, not a mission-energy case";
    elseif contains(r.outcome, 'ABORTED')
        AdditionalPhaseBuffer_Wh(i) = NaN;
        CompleteMissionEnergyEstimate_Wh(i) = NaN;
        CompleteMissionEstimateWithinCeiling(i) = false;
        EnergyBudgetStatus(i) = "Not assessed: engagement aborted before launch";
    else
        if contains(r.releaseTowDecision, 'Tow required') || r.towCompleted
            AdditionalPhaseBuffer_Wh(i) = cfg.energyBufferTow_Wh;
        else
            AdditionalPhaseBuffer_Wh(i) = cfg.energyBufferNoTow_Wh;
        end
        CompleteMissionEnergyEstimate_Wh(i) = EnergyUsed_SimulatedPhases_Wh(i) + AdditionalPhaseBuffer_Wh(i);
        CompleteMissionEstimateWithinCeiling(i) = CompleteMissionEnergyEstimate_Wh(i) <= EnergyBudget_SizingCeiling_Wh(i);
        if CompleteMissionEstimateWithinCeiling(i)
            EnergyBudgetStatus(i) = "Estimated total (simulated phases + assumed buffer) below sizing ceiling; not continuous integration";
        else
            EnergyBudgetStatus(i) = "Estimated total exceeds sizing ceiling";
        end
    end
end

summaryTable = table(ScenarioID, Scenario, ScenarioPurpose, Mode, InitialRange_m, ...
    TargetSpeed_mps, TargetVelocityX_mps, TargetVelocityY_mps, TargetHeading_deg, ...
    InitialInterceptorSpeed_mps, InitialInterceptorHeading_deg, InitialLOSAngle_deg, InitialHeadingOffset_deg, ...
    CaptureSampleIndex, CaptureTime_s, CaptureCandidateRange_m, ...
    CaptureInterceptorX_m, CaptureInterceptorY_m, CaptureInterceptorVx_mps, CaptureInterceptorVy_mps, ...
    CaptureTargetX_m, CaptureTargetY_m, CaptureTargetVx_mps, CaptureTargetVy_mps, ...
    CaptureLOSUnitX, CaptureLOSUnitY, RangeRateAtCandidate_mps, ClosingSpeedAtCandidate_mps, ...
    CandidateGuidanceCommandX_mps2, CandidateGuidanceCommandY_mps2, ...
    CandidateAppliedAccelerationX_mps2, CandidateAppliedAccelerationY_mps2, CandidateActiveLimit, ...
    CommandLeadingIntoCandidateX_mps2, CommandLeadingIntoCandidateY_mps2, ...
    AppliedAccelerationIntoCandidateX_mps2, AppliedAccelerationIntoCandidateY_mps2, ActiveLimitIntoCandidate, ...
    PNActiveAtCandidate, PNAccelerationAtCandidate_mps2, PNPeakAccelerationObserved_mps2, ...
    PNActiveDuration_s, GuidanceCorrectionDemonstrated, TimeToCapture_s, ClosestRange_m, ...
    MaxSpeedObserved_mps, SpeedWithin25mpsLimit, MaxAccelerationObserved_mps2, ...
    AssumedAccelerationLimit_mps2, AccelerationWithinAssumedLimit, WithinWorkingRange_1p5to2km, ...
    CaptureWithin5to15m, EnergyUsed_SimulatedPhases_Wh, AdditionalPhaseBuffer_Wh, ...
    CompleteMissionEnergyEstimate_Wh, EnergyBudget_SizingCeiling_Wh, CompleteMissionEstimateWithinCeiling, ...
    EnergyBudgetStatus, FullMissionEnergyValidated, MissionOutcome, CaptureCandidateRegistered, ...
    CaptureInsideSafeArea, NearestSafePointDistanceAtCapture_m, SafeAreaRadius_m, ...
    ReleaseTowDecision, TowDuration_s, TowCompletedWithin60s, ...
    NominalMass_kg, PostCaptureMass_kg, MassLedgerSubtotal_kg, IntegrationGrowthAllowance_kg, ...
    MotorModel, PropellerModel, ESCModel, PropellerDiameter_in, ArmLength_m, FrameSpan_m, ...
    MaxTotalStaticThrust_N, NominalTWR_Static, PostCaptureTWR_Static, ...
    StaticAvailableAccelerationNominal_mps2, StaticAvailableAccelerationPostCapture_mps2, ...
    ThrustMarginAt25mps_Nominal, ThrustMarginAt25mps_PostCapture, ...
    Conservative30pctThrustMarginNominal, Conservative30pctThrustMarginPostCapture, ...
    StaticDragBalanceSpeedNominal_mps, StaticDragBalanceSpeedPostCapture_mps, ...
    MotorMassEach_kg, MotorSetMass_kg, BatteryMass_kg, BatteryCapacity_Ah, ...
    BatteryNominalEnergy_Wh, BatteryUsableEnergy_Wh, ProvisionalTowSpeedCap_mps);

disp(' ');
disp('SKYFORCE point-mass simulation summary');
disp(summaryTable);

% Separate full-resolution trajectory log supports independent closing-speed
% verification and later phase-by-phase energy integration.
trajectoryTable = buildTrajectoryExport(results);
propulsionSensitivityTable = buildPropulsionSensitivityTable(cfg);
fprintf('\nForward-flight thrust sensitivity (assumed de-rating; not measured):\n');
disp(propulsionSensitivityTable);
summarySavedPath = writeTableWithFallback(summaryTable, 'skyforce_simulation_summary.csv');
trajectorySavedPath = writeTableWithFallback(trajectoryTable, 'skyforce_trajectory_export.csv');
sensitivitySavedPath = writeTableWithFallback(propulsionSensitivityTable, 'skyforce_propulsion_sensitivity.csv');
if ~isempty(summarySavedPath), fprintf('\nSaved scenario summary to: %s\n', summarySavedPath); end
if ~isempty(trajectorySavedPath), fprintf('Saved sample-by-sample trajectory export to: %s\n', trajectorySavedPath); end
if ~isempty(sensitivitySavedPath), fprintf('Saved propulsion sensitivity table to: %s\n', sensitivitySavedPath); end

fprintf('\nSimulation complete. All 5-15 m events are geometric capture candidates only.\n');
fprintf('No net-deployment success, capture probability, or parachute performance is modelled.\n');
fprintf('Complete-mission energy values in the summary are estimates: simulated-phase energy plus assumed buffers.\n');
fprintf('The PN diagnostic starts in flight with a 26.6-degree initial heading/LOS offset; it is not a launch mission.\n');
fprintf('Conservative assumed 30%% forward-flight thrust de-rating margins: %.2fx nominal / %.2fx post-capture.\n', ...
    cfg.thrustMargin30pctNominal, cfg.thrustMargin30pctPostCapture);
fprintf('Propeller and ESC masses, battery product mass, and forward-flight thrust loss remain to be confirmed.\n');

end

%% ------------------------------------------------------------------------
function out = simulateOneScenario(s, cfg)
N = floor(cfg.maxSimTime / cfg.dt) + 1;

pI = s.pI0(:);
pT = s.pT0(:);
vI = s.vI0(:);
vT = s.vT(:);
reactionDelay = s.reactionDelayOverride_s;
initialInterceptorVelocity = vI;
initialLOS = (pT-pI) / max(norm(pT-pI),1e-9);

if norm(vT) < cfg.maxSpeed
    mode = 'Attack (target slower)';
else
    mode = 'Defense (target faster/equal)';
end

% Storage
th = nan(N,1);
pIh = nan(2,N);
pTh = nan(2,N);
vIh = nan(2,N);
vTh = nan(2,N);
ah = zeros(2,N); % finite-difference applied acceleration over step (m/s^2)
requestedAh = zeros(2,N);
activeLimitHistory = repmat("none",N,1);
pnAh = zeros(2,N);
pnActiveHistory = false(N,1);
rh = nan(N,1);
energy = zeros(N,1);
power = zeros(N,1);
phaseHistory = strings(N,1);

phase = 'REACTION';
abortFlag = false;
abortTime = nan;
abortReason = '';
captured = false;
towStartTime = nan;
towGoal = [nan; nan];
captureTime = nan;
captureSampleIndex = nan;
capturePoint = [nan; nan];
captureRangeAtEvent = nan;
closingAtCapture = nan;
captureState = struct('sampleIndex',nan,'time',nan,'pI',[nan;nan],'pT',[nan;nan], ...
    'vI',[nan;nan],'vT',[nan;nan],'los',[nan;nan],'range',nan,'rangeRate',nan, ...
    'closingSpeed',nan,'commandedAccel',[nan;nan],'appliedAccel',[nan;nan], ...
    'activeLimit','not applicable','commandedAccelIntoCandidate',[nan;nan], ...
    'appliedAccelIntoCandidate',[nan;nan],'activeLimitIntoCandidate','not available', ...
    'pnAcceleration',[nan;nan],'pnActive',false);
captureInsideSafeArea = false;
nearestSafePointDistanceAtCapture = nan;
nearestSafeIndexAtCapture = nan;
releaseTowDecision = 'Not evaluated: no capture candidate';
towDuration = nan;
towCompleted = false;
outcome = 'No capture within simulation horizon';
massCurrent = cfg.massNominal;
if s.isGuidanceDiagnostic, energyBudget = NaN; else, energyBudget = cfg.energyBudgetNoTow; end
minimumRange = inf;

lastIndex = N;
for k = 1:N
    t = (k-1) * cfg.dt;
    % Label each integrated sample by the operational phase. The reaction
    % phase ends at the configured delay; subsequent pre-capture motion is
    % logged as INTERCEPT (including relocation to the defense lead point).
    if strcmp(phase,'REACTION') && t >= reactionDelay
        phase = 'INTERCEPT';
    end

    if ~captured
        % Range is meaningful only while interceptor and target are separate.
        R = norm(pT - pI);
        minimumRange = min(minimumRange, R);
    else
        % After the capture-state transition, the target is represented as
        % part of the combined load. Do not report that shared position as a
        % zero-distance engagement measurement.
        R = NaN;
    end

    th(k) = t;
    pIh(:,k) = pI;
    pTh(:,k) = pT;
    vIh(:,k) = vI;
    vTh(:,k) = vT;
    rh(k) = R;
    if k > 1
        energy(k) = energy(k-1);
    else
        energy(k) = 0;
    end
    phaseHistory(k) = string(phase);

    % When already aborted, the interceptor stays put while the target path
    % continues to the boundary, making the abort outcome visible in plots.
    if abortFlag
        if norm(pT - s.keepOutCenter) <= s.keepOutRadius || (t - abortTime) >= 45
            outcome = ['ABORTED: ' abortReason];
            lastIndex = k;
            break;
        end
        pT = pT + vT * cfg.dt;
        if k < N
            energy(k+1) = energy(k);
        end
        continue;
    end

    % Check the geometric engagement window before issuing new control.
    % In defense/lead-point mode, only register the candidate event once the
    % interceptor is settled near the lead point; this prevents a transient
    % waypoint pass from being mislabelled as a stable waiting intercept.
    captureGeometryOK = ~captured && t >= reactionDelay && ...
        R <= cfg.captureRange(2) && R >= cfg.captureRange(1);
    if captureGeometryOK && strcmp(mode,'Defense (target faster/equal)') && ...
            ~isempty(s.leadPoint)
        leadPointSettled = norm(s.leadPoint - pI) <= cfg.waypointTolerance && ...
            norm(vI) <= cfg.waypointSpeedTol;
        captureGeometryOK = captureGeometryOK && leadPointSettled;
    end
    if captureGeometryOK
        captured = true;
        captureTime = t;
        captureSampleIndex = k;
        captureRangeAtEvent = R;
        capturePoint = 0.5 * (pI + pT);
        relV = vT - vI;
        los = (pT-pI) / max(R,1e-9);
        rangeRate = dot(los, relV);
        closingAtCapture = -rangeRate;

        % Evaluate the geometric capture point against every safe-area circle.
        safeDistances = sqrt(sum((s.safePoints - capturePoint).^2, 1));
        [nearestSafePointDistanceAtCapture, safeIndex] = min(safeDistances);
        nearestSafeIndexAtCapture = safeIndex;
        captureInsideSafeArea = any(safeDistances <= cfg.safeAreaRadius);

        % Capture-state data are stored before joining the two point-mass paths.
        captureState.sampleIndex = k;
        captureState.time = t;
        captureState.pI = pI;
        captureState.pT = pT;
        captureState.vI = vI;
        captureState.vT = vT;
        captureState.los = los;
        captureState.range = R;
        captureState.rangeRate = rangeRate;
        captureState.closingSpeed = closingAtCapture;
        % The previous row's integration command/actual acceleration produced
        % this event state. Preserve those values separately from the command
        % evaluated at the candidate (which is not integrated at the event).
        if k > 1
            captureState.commandedAccelIntoCandidate = requestedAh(:,k-1);
            captureState.appliedAccelIntoCandidate = ah(:,k-1);
            captureState.activeLimitIntoCandidate = char(activeLimitHistory(k-1));
        end
        if strcmp(mode,'Defense (target faster/equal)') && ~isempty(s.leadPoint)
            if norm(s.leadPoint-pI) <= cfg.waypointTolerance && norm(vI) <= cfg.waypointSpeedTol
                candidateCmd = -2.0*vI;
            else
                candidateCmd = waypointController(pI,vI,s.leadPoint,cfg.maxSpeed,cfg);
            end
            candidatePN = [0;0];
            candidatePNActive = false;
        else
            [candidateCmd,candidatePN,candidatePNActive] = interceptGuidance(pI,vI,pT,vT,cfg);
        end
        captureState.commandedAccel = candidateCmd;
        captureState.appliedAccel = [nan;nan]; % no control interval is integrated at the event boundary
        captureState.activeLimit = 'Capture-event boundary; no integration on this sample';
        captureState.pnAcceleration = candidatePN;
        captureState.pnActive = candidatePNActive;
        requestedAh(:,k) = candidateCmd;
        ah(:,k) = [nan;nan];
        activeLimitHistory(k) = "Capture-event boundary; no integration";
        pnAh(:,k) = candidatePN;
        pnActiveHistory(k) = candidatePNActive;
        phaseHistory(k) = "CAPTURE_CANDIDATE";
        % Instantaneous power estimate at candidate speed is recorded for the
        % event sample; this endpoint sample is not integrated for another dt.
        power(k) = approximatePower(norm(vI), cfg.massNominal, cfg);

        % From this point the load is represented as one combined point mass.
        % Historical pI/pT values remain separate and preserve the event range.
        pI = capturePoint;
        pT = capturePoint;
        vT = vI;
        massCurrent = cfg.massWithCaptured;

        if captureInsideSafeArea
            phase = 'RELEASE';
            releaseTowDecision = 'Immediate release: capture candidate is inside a designated safe area';
            outcome = 'Geometric capture candidate; immediate release inside designated safe area';
            if ~s.isGuidanceDiagnostic, energyBudget = cfg.energyBudgetNoTow; end
            fprintf('\n%s\n  Candidate range %.2f m; nearest safe-area centre %.2f m away (radius %.1f m).\n', ...
                s.name, captureRangeAtEvent, nearestSafePointDistanceAtCapture, cfg.safeAreaRadius);
            fprintf('  Geometric decision: immediate release inside safe area (physical capture not modelled).\n');
            lastIndex = k;
            break;
        else
            phase = 'TOW';
            releaseTowDecision = 'Tow required: capture candidate is outside all designated safe areas';
            towStartTime = t;
            towGoal = s.safePoints(:,safeIndex);
            if ~s.isGuidanceDiagnostic, energyBudget = cfg.energyBudgetWithTow; end
            fprintf('\n%s\n  Candidate range %.2f m; nearest safe-area centre %.2f m away (radius %.1f m).\n', ...
                s.name, captureRangeAtEvent, nearestSafePointDistanceAtCapture, cfg.safeAreaRadius);
            fprintf('  Geometric decision: tow to safe-point centre %d.\n', safeIndex);
            % Do not integrate a tow-control step at the same time stamp as
            % the pre-capture state. The next sample starts the tow phase.
            if k < N, energy(k+1) = energy(k); end
            continue;
        end
    end

    % Abort decision before capture: (1) hard boundary, or (2) insufficient
    % estimated time before boundary compared with best-case radial intercept.
    if ~captured && ~strcmp(phase,'RELEASE')
        [abortNow, tBoundary, tIntercept, why] = abortDecision( ...
            pI, pT, vT, s.keepOutCenter, s.keepOutRadius, cfg);
        if abortNow
            abortFlag = true;
            abortTime = t;
            abortReason = why;
            phase = 'ABORTED';
            phaseHistory(k) = string(phase);
            outcome = ['ABORTED: ' abortReason];
            % Store the estimated times for diagnostic output.
            fprintf('\n%s\n  Abort at %.2f s: %s\n', s.name, t, why);
            if isfinite(tBoundary) && isfinite(tIntercept)
                fprintf('  Estimated time to boundary %.2f s; best-case intercept %.2f s.\n', ...
                    tBoundary, tIntercept);
            end
            pT = pT + vT * cfg.dt;
            if k < N
                energy(k+1) = energy(k);
            end
            continue;
        end
    end

    % State transition / guidance
    if strcmp(phase,'TOW')
        if (t - towStartTime) >= cfg.maxTowDuration
            towDuration = t - towStartTime;
            towCompleted = false;
            outcome = 'Tow failed: safe point not reached within 60 s';
            lastIndex = k;
            break;
        end
        dGoal = norm(towGoal - pI);
        if dGoal <= cfg.towArrivalTolerance && norm(vI) <= cfg.waypointSpeedTol
            towDuration = t - towStartTime;
            towCompleted = true;
            phase = 'SAFE RELEASE';
            releaseTowDecision = 'Tow completed: reached nearest designated safe point';
            outcome = 'Geometric capture candidate; tow to safe point completed';
            phaseHistory(k) = string(phase);
            lastIndex = k;
            break;
        end
        aCmd = waypointController(pI, vI, towGoal, cfg.maxTowSpeed, cfg);
        aLimitNow = cfg.accelLimit;
        speedLimitNow = cfg.maxTowSpeed;
    elseif t < reactionDelay
        % Reaction delay includes tracking, planning, radio and spin-up.
        aCmd = [0; 0];
        aLimitNow = cfg.accelLimit;
        speedLimitNow = cfg.maxSpeed;
    elseif strcmp(phase,'RELEASE') || strcmp(phase,'SAFE RELEASE')
        aCmd = [0; 0];
        aLimitNow = cfg.accelLimit;
        speedLimitNow = cfg.maxSpeed;
    elseif strcmp(mode,'Defense (target faster/equal)') && ~isempty(s.leadPoint)
        % Relocate to the assigned lead point and hover there for the target
        % to enter the specified 5-15 m window. This is a geometry model only.
        if norm(s.leadPoint - pI) <= cfg.waypointTolerance && ...
                norm(vI) <= cfg.waypointSpeedTol
            aCmd = -2.0 * vI; % damp velocity while holding at the lead point
        else
            aCmd = waypointController(pI, vI, s.leadPoint, cfg.maxSpeed, cfg);
        end
        aLimitNow = cfg.accelLimit;
        speedLimitNow = cfg.maxSpeed;
    else
        [aCmd, aPNNow, pnActiveNow] = interceptGuidance(pI, vI, pT, vT, cfg);
        pnAh(:,k) = aPNNow;
        pnActiveHistory(k) = pnActiveNow;
        aLimitNow = cfg.accelLimit;
        speedLimitNow = cfg.maxSpeed;
    end

    % Log requested control, active limiter, and applied finite-difference acceleration.
    requestedAh(:,k) = aCmd;
    accelCapActive = norm(aCmd) > aLimitNow + 1e-10;
    aLimited = limitVector(aCmd, aLimitNow);
    vIBefore = vI;
    vCandidate = vI + aLimited * cfg.dt;
    speedCapActive = norm(vCandidate) > speedLimitNow + 1e-10;
    vI = limitVector(vCandidate, speedLimitNow);
    if accelCapActive && speedCapActive
        activeLimitHistory(k) = "acceleration and speed limits";
    elseif accelCapActive
        activeLimitHistory(k) = "acceleration limit";
    elseif speedCapActive
        activeLimitHistory(k) = "speed limit";
    else
        activeLimitHistory(k) = "none";
    end
    ah(:,k) = (vI - vIBefore) / cfg.dt;
    pI = pI + vI * cfg.dt;

    if strcmp(phase,'TOW')
        % Captured target is represented as part of the same point mass.
        pT = pI;
        vT = vI;
    else
        pT = pT + s.vT(:) * cfg.dt;
    end

    % Simplified electrical-power estimate: hover power scales with mass^1.5
    % (actuator-disk approximation), while the sprint endpoint scales linearly
    % with mass. A speed^3 blend joins the two endpoints. This is an illustrative
    % approximation, not a measured motor/ESC/battery discharge model.
    if t >= reactionDelay && ~abortFlag
        power(k) = approximatePower(norm(vI), massCurrent, cfg);
        energy(k) = energy(k) + power(k) * cfg.dt / 3600;
    else
        power(k) = 0;
    end

    % Keep a running energy value for the next sample.
    if k < N
        energy(k+1) = energy(k);
    end
end

% Trim preallocated history to samples actually used.
if lastIndex == N
    used = N;
else
    used = max(1,lastIndex);
end
th = th(1:used);
pIh = pIh(:,1:used);
pTh = pTh(:,1:used);
vIh = vIh(:,1:used);
vTh = vTh(:,1:used);
ah = ah(:,1:used);
requestedAh = requestedAh(:,1:used);
activeLimitHistory = activeLimitHistory(1:used);
pnAh = pnAh(:,1:used);
pnActiveHistory = pnActiveHistory(1:used);
rh = rh(1:used);
energy = energy(1:used);
power = power(1:used);
phaseHistory = phaseHistory(1:used);

% Include finite recorded ranges only. Post-capture samples are NaN by design
% because interceptor-to-captured-load separation is not an engagement metric.
finiteRanges = rh(isfinite(rh));
if isempty(finiteRanges)
    minimumRange = nan;
else
    minimumRange = min(minimumRange, min(finiteRanges));
end

% If a capture occurred and tow is still underway at simulation end, report it.
if strcmp(phase,'TOW') && (th(end) - towStartTime) < cfg.maxTowDuration && ...
        ~contains(outcome,'completed')
    towDuration = th(end) - towStartTime;
    towCompleted = false;
    outcome = 'Capture candidate; tow phase incomplete at simulation horizon';
end
if ~isfinite(captureTime)
    releaseTowDecision = 'Not applicable: no capture candidate registered';
end

out.name = s.name;
out.mode = mode;
out.time = th(:);
out.pI = pIh;
out.pT = pTh;
out.vI = vIh;
out.vT = vTh;
out.interceptorSpeed = sqrt(sum(vIh.^2,1))';
out.acceleration = ah;
out.commandedAcceleration = requestedAh;
out.activeLimit = activeLimitHistory;
out.pnAcceleration = pnAh;
out.pnAccelerationMagnitude = sqrt(sum(pnAh.^2,1))';
out.pnActive = pnActiveHistory;
out.isGuidanceDiagnostic = s.isGuidanceDiagnostic;
out.initialInterceptorVelocity = initialInterceptorVelocity;
out.initialLOS = initialLOS;
if norm(initialInterceptorVelocity) > 1e-8
    initialHeading = atan2d(initialInterceptorVelocity(2), initialInterceptorVelocity(1));
    initialLOSAngle = atan2d(initialLOS(2), initialLOS(1));
    out.initialHeadingOffset_deg = mod(initialHeading-initialLOSAngle+180,360)-180;
else
    out.initialHeadingOffset_deg = NaN;
end
out.accelerationMagnitude = sqrt(sum(ah.^2,1))';
out.maxSpeedObserved = max(out.interceptorSpeed);
finiteAccMag = out.accelerationMagnitude(isfinite(out.accelerationMagnitude));
if isempty(finiteAccMag)
    out.maxAccelerationObserved = NaN;
else
    out.maxAccelerationObserved = max(finiteAccMag);
end
out.range = rh(:);
out.energyWh = energy(:);
out.powerW = power(:);
out.phase = phaseHistory(:);
out.initialRange = norm(s.pT0(:) - s.pI0(:));
out.targetSpeed = norm(s.vT(:));
out.targetVelocity = s.vT(:);
out.captureTime = captureTime;
out.captureSampleIndex = captureSampleIndex;
out.captureState = captureState;
out.capturePoint = capturePoint;
out.captureRangeAtEvent = captureRangeAtEvent;
out.closingAtCapture = closingAtCapture;
out.captureInsideSafeArea = captureInsideSafeArea;
out.nearestSafePointDistanceAtCapture = nearestSafePointDistanceAtCapture;
out.nearestSafeIndexAtCapture = nearestSafeIndexAtCapture;
out.releaseTowDecision = releaseTowDecision;
out.towDuration = towDuration;
out.towCompleted = towCompleted;
out.minimumRange = minimumRange;
out.energyBudget = energyBudget;
out.outcome = outcome;
out.abortTime = abortTime;
out.abortReason = abortReason;
out.keepOutCenter = s.keepOutCenter;
out.keepOutRadius = s.keepOutRadius;
out.safePoints = s.safePoints;
out.safeAreaRadius = cfg.safeAreaRadius;
out.leadPoint = s.leadPoint;
out.towGoal = towGoal;
end

%% ------------------------------------------------------------------------
function [aCmd, aPN, pnActive] = interceptGuidance(pI, vI, pT, vT, cfg)
% Long-range constant-velocity intercept-point guidance, with a terminal
% proportional-navigation acceleration blend inside a provisional range.
aPN = [0;0];
pnActive = false;
r = pT - pI;
R = max(norm(r), 1e-6);
aimPoint = constantSpeedInterceptPoint(pI, pT, vT, cfg.maxSpeed);
d = aimPoint - pI;
dNorm = max(norm(d), 1e-6);
uAim = d / dNorm;

% Braking profile prevents the point mass from demanding an instantaneous
% speed change at the aim point; acceleration is clipped by the model limit.
vDesMagnitude = min(cfg.maxSpeed, sqrt(2 * cfg.accelLimit * dNorm));
vDesired = vDesMagnitude * uAim;
aCmd = 2.0 * (vDesired - vI);

if R <= cfg.PNswitchRange && norm(vI) > 0.5
    pnActive = true;
    vRelative = vT - vI;
    closingSpeed = -dot(r, vRelative) / R;
    lambdaDot = (r(1)*vRelative(2) - r(2)*vRelative(1)) / (R^2);
    uVelocity = vI / norm(vI);
    normalToVelocity = [-uVelocity(2); uVelocity(1)];
    aPN = cfg.PNgain * max(closingSpeed, 0) * lambdaDot * normalToVelocity;
    aCmd = 0.25 * aCmd + 0.75 * aPN;
end
end

%% ------------------------------------------------------------------------
function pAim = constantSpeedInterceptPoint(pI, pT, vT, speedLimit)
r = pT - pI;
A = dot(vT,vT) - speedLimit^2;
B = 2 * dot(r,vT);
C = dot(r,r);

if abs(A) < 1e-9
    if abs(B) > 1e-9
        tCandidates = -C/B;
    else
        tCandidates = [];
    end
else
    discriminant = B^2 - 4*A*C;
    if discriminant < 0
        tCandidates = [];
    else
        rootDisc = sqrt(discriminant);
        tCandidates = [(-B + rootDisc)/(2*A), (-B - rootDisc)/(2*A)];
    end
end

tCandidates = tCandidates(isfinite(tCandidates) & tCandidates > 0);
if isempty(tCandidates)
    % Fallback for geometry without a positive constant-speed solution.
    tLead = min(5, max(1, norm(r)/max(norm(vT),1)));
else
    tLead = min(tCandidates);
end
pAim = pT + vT * tLead;
end

%% ------------------------------------------------------------------------
function aCmd = waypointController(p, v, waypoint, speedLimit, cfg)
% Damped position-velocity controller. The previous velocity-only waypoint
% law could reverse its desired direction after overshoot and oscillate.
% This PD form anticipates stopping through the -Kd*v term; acceleration and
% speed are still bounded by the caller. Gains are illustrative, not tuned
% flight-controller gains.
e = waypoint - p;
Kp = 0.35;  % 1/s^2, provisional point-mass position gain
Kd = 1.50;  % 1/s, provisional damping gain

aCmd = Kp * e - Kd * v;

% Inside the waypoint tolerance, prioritise damping residual velocity.
if norm(e) <= cfg.waypointTolerance && norm(v) <= cfg.waypointSpeedTol
    aCmd = -2.0 * v;
end

% Acceleration and speed limiting occur in the main integrator, where both
% requested and applied values are recorded. speedLimit remains an interface
% guard rather than a local limiter.
if speedLimit <= 0
    aCmd = [0; 0];
end
end

%% ------------------------------------------------------------------------
function [abortNow, tBoundary, tIntercept, reason] = abortDecision( ...
    pI, pT, vT, keepOutCenter, keepOutRadius, cfg)
abortNow = false;
tBoundary = inf;
tIntercept = inf;
reason = '';

fromCenter = pT - keepOutCenter;
dCenter = norm(fromCenter);
if dCenter <= keepOutRadius
    abortNow = true;
    tBoundary = 0;
    reason = 'hard keep-out boundary has already been crossed';
    return;
end

clearance = dCenter - keepOutRadius;
radialClosing = -dot(fromCenter, vT) / max(dCenter,1e-6);
if radialClosing > 1e-6
    tBoundary = clearance / radialClosing;
end

r = pT - pI;
R = max(norm(r),1e-6);
lineOfSight = r/R;
targetRadialSpeed = dot(vT,lineOfSight); % positive means target moving away along LOS
bestCaseClosing = cfg.maxSpeed - targetRadialSpeed;
if bestCaseClosing > 0.5
    tIntercept = R / bestCaseClosing;
end

if isfinite(tBoundary) && tBoundary <= tIntercept + cfg.abortTimeMargin
    abortNow = true;
    reason = sprintf('time-to-boundary (%.1f s) is no greater than estimated intercept time (%.1f s) plus margin', ...
        tBoundary, tIntercept);
end
end

%% ------------------------------------------------------------------------
function P = approximatePower(speed, mass, cfg)
speedFraction = min(max(speed / cfg.maxSpeed, 0), 1);
massScale = mass / cfg.massNominal;
hover = cfg.hoverPower * massScale^1.5;
sprint = cfg.sprintPower * massScale;
P = hover + (sprint - hover) * speedFraction^3;
end

%% ------------------------------------------------------------------------
function vOut = limitVector(vIn, limitValue)
vOut = vIn;
mag = norm(vIn);
if mag > limitValue && mag > 0
    vOut = vIn * (limitValue / mag);
end
end

%% ------------------------------------------------------------------------
function plotOneScenario(out, cfg)
fig = figure('Name', out.name, 'Color', 'w', 'Position', [60 40 1450 980]);

% 1) Plan-view trajectory
ax1 = subplot(3,2,[1 3],'Parent',fig);
hold(ax1,'on'); grid(ax1,'on'); axis(ax1,'equal');
ang = linspace(0,2*pi,300);
xc = out.keepOutCenter(1) + out.keepOutRadius*cos(ang);
yc = out.keepOutCenter(2) + out.keepOutRadius*sin(ang);
hKeep = plot(ax1,xc,yc,'r--','LineWidth',1.5);
hInterceptor = plot(ax1,out.pI(1,:),out.pI(2,:),'b-','LineWidth',1.7);
hTarget = plot(ax1,out.pT(1,:),out.pT(2,:),'Color',[0.85 0.35 0.1],'LineStyle','-','LineWidth',1.7);
hIstart = plot(ax1,out.pI(1,1),out.pI(2,1),'bo','MarkerFaceColor','b');
hTstart = plot(ax1,out.pT(1,1),out.pT(2,1),'o','Color',[0.85 0.35 0.1],'MarkerFaceColor',[0.85 0.35 0.1]);
hSafe = plot(ax1,out.safePoints(1,:),out.safePoints(2,:),'g^','MarkerFaceColor','g','MarkerSize',8);

% Explicit circles show the same geometric test used by simulateOneScenario.
% A capture candidate inside any circle gets immediate release; otherwise the
% simulation records a tow to the nearest circle centre. Radius is provisional.
hSafeArea = gobjects(1,1);
for q = 1:size(out.safePoints,2)
    xs = out.safePoints(1,q) + out.safeAreaRadius*cos(ang);
    ys = out.safePoints(2,q) + out.safeAreaRadius*sin(ang);
    hTmp = plot(ax1,xs,ys,'g:','LineWidth',1.1);
    if q == 1
        hSafeArea = hTmp;
    end
end
hLegend = [hKeep, hInterceptor, hTarget, hIstart, hTstart, hSafe, hSafeArea];
legendText = {'Keep-out boundary','Interceptor','Target / captured load', ...
    'Interceptor start','Target start','Safe-point centres','Designated safe-area boundary'};
if ~isempty(out.leadPoint)
    hLead = plot(ax1,out.leadPoint(1),out.leadPoint(2),'ks','MarkerFaceColor','y','MarkerSize',8);
    hLegend(end+1) = hLead;
    legendText{end+1} = 'Lead point';
end
if all(isfinite(out.capturePoint))
    hCapture = plot(ax1,out.capturePoint(1),out.capturePoint(2),'mp','MarkerFaceColor','m','MarkerSize',12);
    hLegend(end+1) = hCapture;
    legendText{end+1} = 'Capture candidate';
end
xlabel(ax1,'x (m)'); ylabel(ax1,'y (m)');
title(ax1,'2-D trajectories (illustrative 300 m keep-out / 120 m safe radii)','Color','k');
lg = legend(ax1,hLegend,legendText,'Location','bestoutside');
set(lg,'Color','w','TextColor','k','EdgeColor',[0.65 0.65 0.65]);
styleAxes(ax1);

% Show the geometric release/tow decision explicitly on the trajectory panel.
if isfinite(out.captureTime)
    if out.captureInsideSafeArea
        safeMsg = sprintf('Capture point inside safe area (nearest centre %.1f m)', ...
            out.nearestSafePointDistanceAtCapture);
    else
        safeMsg = sprintf('Capture point outside safe areas (nearest centre %.1f m): TOW', ...
            out.nearestSafePointDistanceAtCapture);
    end
    text(ax1,0.02,0.02,safeMsg,'Units','normalized','FontSize',8, ...
        'Color','k','BackgroundColor','w','EdgeColor',[0.7 0.7 0.7], ...
        'Interpreter','none');
end

% 2) Range versus time: range samples stop at the capture transition.
ax2 = subplot(3,2,2,'Parent',fig);
validRange = isfinite(out.range);
plot(ax2,out.time(validRange),out.range(validRange),'k-','LineWidth',1.5);
hold(ax2,'on'); grid(ax2,'on');
yline(ax2,cfg.captureRange(1),'r--','5 m');
yline(ax2,cfg.captureRange(2),'r--','15 m');
if isfinite(out.captureTime)
    xline(ax2,out.captureTime,'m:','Candidate');
    xlim(ax2,[0, out.captureTime + 1.0]);
end
xlabel(ax2,'Time (s)'); ylabel(ax2,'Interceptor-target range (m)');
title(ax2,'Range while vehicles remain separate','Color','k');
styleAxes(ax2);

% Near-range inset: the 5-15 m interval is the assumed geometric engagement
% window, not a model of net deployment or proof of capture success.
if isfinite(out.captureTime) && any(validRange)
    pos = ax2.Position;
    insetPos = [pos(1)+0.55*pos(3), pos(2)+0.50*pos(4), ...
                0.40*pos(3), 0.38*pos(4)];
    axInset = axes('Parent',fig,'Position',insetPos,'Color','w');
    hold(axInset,'on'); grid(axInset,'on');
    plot(axInset,out.time(validRange),out.range(validRange),'k-','LineWidth',1.0);
    yline(axInset,cfg.captureRange(1),'r--');
    yline(axInset,cfg.captureRange(2),'r--');
    xlim(axInset,[max(0,out.captureTime-2.0), out.captureTime+0.1]);
    ylim(axInset,[0,60]);
    title(axInset,'Engagement window (assumed)','FontSize',8,'Color','k');
    set(axInset,'FontSize',7,'XColor','k','YColor','k','GridColor',[0.8 0.8 0.8]);
    axInset.Box = 'on';
end

% 3) Speed check
ax3 = subplot(3,2,4,'Parent',fig);
plot(ax3,out.time,out.interceptorSpeed,'b-','LineWidth',1.5); hold(ax3,'on'); grid(ax3,'on');
yline(ax3,cfg.maxSpeed,'r--','25 m/s limit');
if any(out.phase == "TOW")
    yline(ax3,cfg.maxTowSpeed,'g:','Provisional low-speed tow cap');
end
xlabel(ax3,'Time (s)'); ylabel(ax3,'Interceptor speed (m/s)');
title(ax3,sprintf('Speed check (max observed %.2f m/s)',out.maxSpeedObserved),'Color','k');
styleAxes(ax3);

% 4) Acceleration check. This checks the model's assumed cap only; it does
% not establish actual aircraft thrust, power or controller capability.
ax4 = subplot(3,2,5,'Parent',fig);
plot(ax4,out.time,out.accelerationMagnitude,'Color',[0.45 0.15 0.65],'LineWidth',1.4);
hold(ax4,'on'); grid(ax4,'on');
yline(ax4,cfg.accelLimit,'r--',sprintf('Assumed %.1f m/s^2 cap',cfg.accelLimit));
xlabel(ax4,'Time (s)'); ylabel(ax4,'Acceleration magnitude (m/s^2)');
title(ax4,sprintf('Model acceleration (max %.2f m/s^2)',out.maxAccelerationObserved),'Color','k');
styleAxes(ax4);

% 5) Simulated-phase energy, corrected sizing ceiling, and usable battery energy.
ax5 = subplot(3,2,6,'Parent',fig);
plot(ax5,out.time,out.energyWh,'Color',[0.1 0.55 0.2],'LineWidth',1.5);
hold(ax5,'on'); grid(ax5,'on');
if isfinite(out.energyBudget)
    yline(ax5,out.energyBudget,'r--',sprintf('Sizing ceiling %.0f Wh',out.energyBudget));
end
yline(ax5,cfg.batteryUsableEnergyWh,'b:', ...
    sprintf('Battery energy counted usable %.0f Wh',cfg.batteryUsableEnergyWh));
ylim(ax5,[0, 1.08 * max([cfg.batteryUsableEnergyWh, out.energyBudget(isfinite(out.energyBudget)), max(out.energyWh)])]);
xlabel(ax5,'Time (s)'); ylabel(ax5,'Estimated energy (Wh)');
title(ax5,'Energy for simulated phases only','Color','k');
if out.isGuidanceDiagnostic
    msg = 'PN diagnostic only; not a mission-energy case';
elseif contains(out.outcome,'ABORTED')
    msg = 'Not assessed: engagement aborted before launch';
else
    msg = sprintf('Complete-mission estimate: %.2f Wh (assumed buffer; not continuous integration)', ...
        out.energyWh(end) + selectEnergyBuffer(out,cfg));
end
text(ax5,0.04,0.88,msg,'Units','normalized', ...
    'Color','k','BackgroundColor','w','EdgeColor',[0.7 0.7 0.7],'FontSize',8);
text(ax5,0.04,0.78,'Sizing ceilings include 20% demand margin; 80% of nominal pack energy counted usable', ...
    'Units','normalized','Color',[0.2 0.2 0.6],'BackgroundColor','w', ...
    'EdgeColor',[0.7 0.7 0.7],'FontSize',7);
styleAxes(ax5);

sg = sgtitle(sprintf('%s\nMode: %s | Outcome: %s', out.name, out.mode, out.outcome), ...
    'Interpreter','none');
sg.Color = 'k';
end



function plotTerminalApproachSummary(results, cfg)
% Compact local views show the last separate-vehicle positions before a
% geometric capture candidate. These plots improve readability; they do not
% prove net deployment or physical capture success.
capturedIdx = [];
for k = 1:numel(results)
    if isfinite(results{k}.captureTime) && ~results{k}.isGuidanceDiagnostic
        capturedIdx(end+1) = k; %#ok<AGROW>
    end
end
if isempty(capturedIdx)
    return;
end

fig = figure('Name','SKYFORCE terminal approach details','Color','w', ...
    'Position',[80 80 1500 520]);
numPanels = numel(capturedIdx);
for j = 1:numPanels
    out = results{capturedIdx(j)};
    ax = subplot(1,numPanels,j,'Parent',fig);
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    preCapture = out.time <= out.captureTime + 1e-9;
    plot(ax,out.pI(1,preCapture),out.pI(2,preCapture),'b-','LineWidth',1.5);
    plot(ax,out.pT(1,preCapture),out.pT(2,preCapture), ...
        'Color',[0.85 0.35 0.1],'LineWidth',1.5);

    % Mark the final distinct positions and connect them to illustrate the
    % measured pre-transition separation. The midpoint is only a marker.
    pIfinal = out.pI(:,find(preCapture,1,'last'));
    pTfinal = out.pT(:,find(preCapture,1,'last'));
    plot(ax,pIfinal(1),pIfinal(2),'bo','MarkerFaceColor','b','MarkerSize',6);
    plot(ax,pTfinal(1),pTfinal(2),'o','Color',[0.85 0.35 0.1], ...
        'MarkerFaceColor',[0.85 0.35 0.1],'MarkerSize',6);
    plot(ax,[pIfinal(1),pTfinal(1)],[pIfinal(2),pTfinal(2)],'k--','LineWidth',1.0);
    plot(ax,out.capturePoint(1),out.capturePoint(2),'mp','MarkerFaceColor','m','MarkerSize',11);

    xlim(ax,[out.capturePoint(1)-250, out.capturePoint(1)+250]);
    ylim(ax,[out.capturePoint(2)-250, out.capturePoint(2)+250]);
    xlabel(ax,'x (m)'); ylabel(ax,'y (m)');
    title(ax,sprintf('%s\nRange at candidate: %.2f m', ...
        strrep(out.name,' - ',' - '),out.captureRangeAtEvent), ...
        'Interpreter','none','FontSize',9,'Color','k');
    legend(ax,{'Interceptor path','Target path','Interceptor at candidate', ...
        'Target at candidate','Measured final separation','Capture midpoint (geometric)'}, ...
        'Location','best','FontSize',7);
    styleAxes(ax);
end
sg = sgtitle('Terminal approach details — pre-capture kinematics only');
sg.Color = 'k';
end

function styleAxes(ax)
set(ax,'Color','w','XColor','k','YColor','k','FontSize',9, ...
    'GridColor',[0.82 0.82 0.82],'GridAlpha',0.75,'LineWidth',0.8);
end

function bufferWh = selectEnergyBuffer(out,cfg)
% Assumed allowances for phases not integrated in the point-mass trajectory.
if contains(out.releaseTowDecision,'Tow required') || out.towCompleted
    bufferWh = cfg.energyBufferTow_Wh;
else
    bufferWh = cfg.energyBufferNoTow_Wh;
end
end

function T = buildTrajectoryExport(results)
% One row per integration sample for independent kinematic/control auditing.
parts = cell(numel(results),1);
for ir = 1:numel(results)
    out = results{ir};
    ns = numel(out.time);
    sampleIndex = (1:ns)';
    ScenarioID = repmat(out.scenarioID,ns,1);
    Scenario = repmat(string(out.name),ns,1);
    ScenarioPurpose = repmat(string(ifelseLocal(out.isGuidanceDiagnostic, ...
        'PN guidance diagnostic only; in-flight initial condition', 'Mission scenario')),ns,1);
    Time_s = out.time(:);
    CaptureEventSampleIndex = repmat(out.captureSampleIndex,ns,1);
    EventIndex = nan(ns,1);
    IsCaptureCandidateSample = false(ns,1);
    if isfinite(out.captureSampleIndex) && out.captureSampleIndex >= 1 && out.captureSampleIndex <= ns
        idx = out.captureSampleIndex;
        EventIndex(idx) = idx;
        IsCaptureCandidateSample(idx) = true;
    end
    InterceptorX_m = out.pI(1,:)'; InterceptorY_m = out.pI(2,:)';
    InterceptorVx_mps = out.vI(1,:)'; InterceptorVy_mps = out.vI(2,:)';
    TargetX_m = out.pT(1,:)'; TargetY_m = out.pT(2,:)';
    TargetVx_mps = out.vT(1,:)'; TargetVy_mps = out.vT(2,:)';
    Range_m = out.range(:);
    delta = out.pT - out.pI;
    LOSUnitX = nan(ns,1); LOSUnitY = nan(ns,1);
    valid = isfinite(Range_m) & Range_m > 1e-9;
    LOSUnitX(valid) = delta(1,valid)' ./ Range_m(valid);
    LOSUnitY(valid) = delta(2,valid)' ./ Range_m(valid);
    relV = out.vT - out.vI;
    RangeRate_mps = nan(ns,1);
    RangeRate_mps(valid) = LOSUnitX(valid).*relV(1,valid)' + LOSUnitY(valid).*relV(2,valid)';
    ClosingSpeed_mps = -RangeRate_mps;
    CommandedAccelerationX_mps2 = out.commandedAcceleration(1,:)';
    CommandedAccelerationY_mps2 = out.commandedAcceleration(2,:)';
    CommandedAccelerationMagnitude_mps2 = sqrt(CommandedAccelerationX_mps2.^2 + CommandedAccelerationY_mps2.^2);
    AppliedAccelerationX_mps2 = out.acceleration(1,:)';
    AppliedAccelerationY_mps2 = out.acceleration(2,:)';
    AppliedAccelerationMagnitude_mps2 = sqrt(AppliedAccelerationX_mps2.^2 + AppliedAccelerationY_mps2.^2);
    ActiveLimit = out.activeLimit(:);
    PNAccelerationX_mps2 = out.pnAcceleration(1,:)';
    PNAccelerationY_mps2 = out.pnAcceleration(2,:)';
    PNAccelerationMagnitude_mps2 = out.pnAccelerationMagnitude(:);
    PNGuidanceActive = out.pnActive(:);
    Phase = out.phase(:);
    ElectricalPowerEstimate_W = out.powerW(:);
    CumulativeEnergy_Wh = out.energyWh(:);
    IncrementalEnergy_Wh = [0; diff(CumulativeEnergy_Wh)];
    parts{ir} = table(ScenarioID,Scenario,ScenarioPurpose,sampleIndex,Time_s, ...
        CaptureEventSampleIndex,EventIndex,IsCaptureCandidateSample, ...
        InterceptorX_m,InterceptorY_m,InterceptorVx_mps,InterceptorVy_mps, ...
        TargetX_m,TargetY_m,TargetVx_mps,TargetVy_mps,Range_m,LOSUnitX,LOSUnitY, ...
        RangeRate_mps,ClosingSpeed_mps,CommandedAccelerationX_mps2,CommandedAccelerationY_mps2, ...
        CommandedAccelerationMagnitude_mps2,AppliedAccelerationX_mps2,AppliedAccelerationY_mps2, ...
        AppliedAccelerationMagnitude_mps2,ActiveLimit,PNAccelerationX_mps2,PNAccelerationY_mps2, ...
        PNAccelerationMagnitude_mps2,PNGuidanceActive,Phase,ElectricalPowerEstimate_W, ...
        IncrementalEnergy_Wh,CumulativeEnergy_Wh);
end
T = vertcat(parts{:});
end

function T = buildPropulsionSensitivityTable(cfg)
% Guide-approved 15/20/30 percent thrust-de-rating sensitivity table.
d = cfg.forwardFlightDeratingSensitivity(:);
AvailableThrustAfterDerating_N = (1-d) * cfg.totalMaxThrustN;
RequiredThrustAt25Nominal_N = repmat(cfg.requiredThrustAt25NominalN,numel(d),1);
RequiredThrustAt25PostCapture_N = repmat(cfg.requiredThrustAt25PostCaptureN,numel(d),1);
MarginAt25Nominal = AvailableThrustAfterDerating_N ./ RequiredThrustAt25Nominal_N;
MarginAt25PostCapture = AvailableThrustAfterDerating_N ./ RequiredThrustAt25PostCapture_N;
T = table(d,AvailableThrustAfterDerating_N,RequiredThrustAt25Nominal_N, ...
    RequiredThrustAt25PostCapture_N,MarginAt25Nominal,MarginAt25PostCapture, ...
    'VariableNames',{'AssumedThrustDeratingFraction','AvailableThrustAfterDerating_N', ...
    'RequiredThrustAt25Nominal_N','RequiredThrustAt25PostCapture_N', ...
    'MarginAt25Nominal','MarginAt25PostCapture'});
end

function savedPath = writeTableWithFallback(T,baseName)
% Save in current folder, retry timestamped name, then MATLAB temp folder.
savedPath = '';
try
    writetable(T,baseName);
    savedPath = fullfile(pwd,baseName);
catch firstError
    timestamp = datestr(now,'yyyymmdd_HHMMSS');
    fallbackName = [erase(baseName,'.csv') '_' timestamp '.csv'];
    try
        writetable(T,fallbackName);
        savedPath = fullfile(pwd,fallbackName);
    catch secondError
        fallbackPath = fullfile(tempdir,fallbackName);
        try
            writetable(T,fallbackPath);
            savedPath = fallbackPath;
        catch thirdError
            warning('SKYFORCE:CSVWriteFailed', ...
                'Could not write %s. Errors: %s | %s | %s', baseName, ...
                firstError.message,secondError.message,thirdError.message);
        end
    end
end
end

function out = ifelseLocal(condition,a,b)
if condition, out = a; else, out = b; end
end

function plotPNCorrectionDiagnostic(results,cfg)
% Dedicated plot for the guide-requested offset-heading PN correction case.
idx = [];
for k = 1:numel(results)
    if results{k}.isGuidanceDiagnostic
        idx = k; break;
    end
end
if isempty(idx), return; end
out = results{idx};
fig = figure('Name','SKYFORCE PN correction diagnostic','Color','w', ...
    'Position',[100 100 1300 560]);

ax1 = subplot(1,2,1,'Parent',fig);
hold(ax1,'on'); grid(ax1,'on'); axis(ax1,'equal');
plot(ax1,out.pI(1,:),out.pI(2,:),'b-','LineWidth',1.7);
plot(ax1,out.pT(1,:),out.pT(2,:),'Color',[0.85 0.35 0.1],'LineWidth',1.7);
plot(ax1,out.pI(1,1),out.pI(2,1),'bo','MarkerFaceColor','b');
plot(ax1,out.pT(1,1),out.pT(2,1),'o','Color',[0.85 0.35 0.1],'MarkerFaceColor',[0.85 0.35 0.1]);
plot(ax1,[out.pI(1,1),out.pT(1,1)],[out.pI(2,1),out.pT(2,1)],'k--','LineWidth',1.0);
if isfinite(out.captureTime)
    plot(ax1,out.capturePoint(1),out.capturePoint(2),'mp','MarkerFaceColor','m','MarkerSize',12);
end
% Show initial interceptor velocity direction and initial target velocity.
quiver(ax1,out.pI(1,1),out.pI(2,1),out.initialInterceptorVelocity(1)*3, ...
    out.initialInterceptorVelocity(2)*3,0,'b','LineWidth',1.5,'MaxHeadSize',1.2);
quiver(ax1,out.pT(1,1),out.pT(2,1),out.targetVelocity(1)*3, ...
    out.targetVelocity(2)*3,0,'Color',[0.85 0.35 0.1],'LineWidth',1.5,'MaxHeadSize',1.2);
xlabel(ax1,'x (m)'); ylabel(ax1,'y (m)');
title(ax1,sprintf('Offset initial heading: %.1f deg',out.initialHeadingOffset_deg),'Color','k');
legend(ax1,{'Interceptor path','Target path','Interceptor start','Target start', ...
    'Initial line of sight','Initial interceptor velocity (scaled)','Target velocity (scaled)','Capture candidate'}, ...
    'Location','best','FontSize',8);
styleAxes(ax1);

ax2 = subplot(1,2,2,'Parent',fig);
hold(ax2,'on'); grid(ax2,'on');
plot(ax2,out.time,out.pnAccelerationMagnitude,'m-','LineWidth',1.5);
plot(ax2,out.time,out.accelerationMagnitude,'b-','LineWidth',1.1);
yline(ax2,cfg.accelLimit,'r--','4.5 m/s^2 model cap');
xlabel(ax2,'Time (s)'); ylabel(ax2,'Acceleration magnitude (m/s^2)');
title(ax2,sprintf('PN correction diagnostic (peak PN command %.2f m/s^2)', ...
    max(out.pnAccelerationMagnitude)),'Color','k');
legend(ax2,{'PN acceleration command','Applied model acceleration','Assumed acceleration cap'}, ...
    'Location','best');
styleAxes(ax2);

sg = sgtitle(sprintf('PN guidance diagnostic only — %d active samples, %.2f s', ...
    sum(out.pnActive),sum(out.pnActive)*cfg.dt));
sg.Color = 'k';
end

