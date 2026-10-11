function skyforce_intercept_simulation_final_report
% SKYFORCE MATLAB intercept simulation (2-D point-mass model; bounded guidance command)
% -------------------------------------------------------------------------
% Implements the scenario in "SKYFORCE - MATLAB Intercept Simulation:
% What It Should Show.pdf" as an illustrative kinematic simulation.
%
% UPDATED TO MATCH: "SKYFORCE - Resolution of Simulation Issues"
% AUTHORITATIVE BASELINE (first-pass U8 II / 22-in / 236 Wh / 4.0 kW data
% is withdrawn): U8 Lite KV100, G28x9.2 CF 28-in propeller, Alpha 60A 12S ESC,
% 18.0 kg nominal mass, 23.0 kg post-capture mass for a 5 kg target, 1.56 kW hover, 3.61 kW
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
% - The 14.78 kg component ledger uses first-pass estimates; the separate 3.22 kg
%   figure is arithmetic integration/growth headroom, not an extra component.
%   Selected bare-component reference masses are exported separately and are not
%   substituted into the conservative 16-line mass ledger without team approval.
% - The 1 km reference circle, safe-area circles and example tracks are illustrative, not
%   approved operational boundaries.
% - The launch-gate demonstration is offline/synthetic. The outer 2,000 m ring
%   starts track assessment and the 1,000 m circle is now a reference boundary,
%   not a hard limit on interceptor position or candidate location. Pursuit can
%   continue outside the circle only while the effective radio/telemetry envelope
%   and estimated energy-reserve guard pass. Range and energy settings are
%   placeholders until replaced by tested link-budget and propulsion data.
% - Overweight targets (>5 kg) do not trigger a mass-only launch abort. If an
%   overweight target reaches a geometric capture-candidate event, the simulation
%   flags immediate parachute-request/no-tow and stops that trajectory there.
%   This is a simulation flag only, not a real deployment command or proof of safety.
% - All scenario initial interceptor positions use the same illustrative station
%   coordinate [0; -1000] m, exactly on the 1,000 m reference-circle circumference.
%   Mission cases remain stationary there until their simulated launch authorization;
%   the PN diagnostic starts there with its specified nonzero in-flight velocity.
%   These are 2-D coordinates; vertical takeoff/climb dynamics are not modelled.
% - The main guidance law still includes a PN blend; removing the diagnostic case
%   would not remove PN from the main scenarios.
% - Exports the CSV tables (including a working-range assessment) plus every
%   generated figure to the MATLAB current folder as PNG and editable FIG files.
% - Tow completion is defined as entering the designated safe-area radius
%   within 60 s; this does not model landing or a stationary release at centre.
% - Energy reporting separates simulated phase energy, estimated remaining
%   segment energy, phase buffer, and projected total. Failed tow and unmodelled
%   overweight recovery do not receive a misleading complete-mission projection.
% - An isolated communication-guard logic test is run and exported separately; it
%   tests the same range predicate used by the flight loop and is not a flight test.
% - The trajectory CSV records the exact capture-event state, LOS, range rate,
%   closing speed, control limits, PN activity, phase, power and cumulative energy.
% - Candidate closing speeds are kinematic range-rate calculations only. The
%   model defines no validated safe capture-speed threshold and does not model
%   net deployment, tether loads, impact/shock loads, or capture reliability.
% - Intercept velocity feedback now uses a configurable response-time parameter
%   (5 s provisional assumption) instead of the former 2.0 1/s gain that produced
%   a 40.46 m/s^2 raw command in the standalone diagnostic. The raw request,
%   controller-limited command and applied point-mass acceleration are logged
%   separately. The 5 s value is a simulation assumption, not measured hardware data.
% - The model's command limiter is not proof that a real aircraft can deliver the
%   requested manoeuvre. There is no measured actuator or attitude-response model.
% - The 1.5-2.0 km initial-range band is assessed per scenario but is not treated
%   as a universal hard launch gate pending guide confirmation of its scope.
% - The gate response estimate is consistently calculated as reaction/latency
%   + flight-response estimate + safety margin (currently 33.8 s).
% - Mass ledger entries are first-pass estimates even though the 16-line sum
%   reconciles arithmetically to 14.78 kg. The 18.00 kg nominal mass, 23.00 kg
%   loaded mass, propulsion figures, battery mass/power, and energy buffers are
%   provisional model inputs pending hardware-level verification.
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
cfg.PNgain                = 3;          % proportional-navigation gain, N = 3 (provisional)
cfg.interceptVelocityResponseTime_s = 5.0; % s; assumed smooth velocity-servo response, not measured
cfg.captureRange          = [5, 15];    % m, unvalidated geometric engagement window
cfg.massNominal           = 18.00;      % kg, authoritative nominal sizing mass
cfg.massWithCaptured      = 23.00;      % kg = 18.00 kg nominal + 5.00 kg target; net/tether already in nominal mass
cfg.massLedgerSubtotal    = 14.78;      % kg, 16-line first-pass mass ledger subtotal
cfg.massGrowthAllowance   = 3.22;       % kg, arithmetic headroom above subtotal; not extra installed parts
cfg.airframeLedgerMass_kg = 4.50;
cfg.motorSetLedgerMass_kg = 1.43;
cfg.escSetLedgerMass_kg = 0.78;        % ledger placeholder; bare six-ESC reference = 0.438 kg
cfg.propellerSetLedgerMass_kg = 0.60;  % ledger placeholder; bare six-prop reference = 0.354 kg
cfg.landingGearLedgerMass_kg = 0.80;
cfg.batteryMass_kg        = 1.97;       % kg, product not selected; estimated
cfg.powerWiringLedgerMass_kg = 0.80;
cfg.flightControlLedgerMass_kg = 0.40;
cfg.companionComputerLedgerMass_kg = 0.30;
cfg.cameraGimbalLedgerMass_kg = 0.50;
cfg.datalinkLedgerMass_kg = 0.50;     % unresolved placeholder; bare modem reference = 0.014 kg
cfg.netLauncherLedgerMass_kg = 0.80;
cfg.netTetherMass_kg = 0.30;          % already included in nominal mass
cfg.co2HardwareLedgerMass_kg = 0.20;
cfg.parachuteReleaseLedgerMass_kg = 0.40;
cfg.miscLedgerMass_kg = 0.50;
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
cfg.maxTowSpeed           = 5.0;        % m/s, guide's towing-transition correction assumption; ramp-limited target speed
% Guide-approved added-phase buffers. These are estimates, not flight measurements.
cfg.energyBufferNoTow_Wh  = 9.7;        % launch + descent + avionics allowance
cfg.energyBufferTow_Wh    = 18.8;       % launch + loaded descent + avionics allowance
cfg.forwardFlightDeratingHeadline = 0.30; % assumed conservative thrust de-rating
cfg.forwardFlightDeratingSensitivity = [0.15 0.20 0.30]; % assumed, not measured
cfg.workingRangeMin       = 1500;       % m, illustrative 1.5-2 km working-range assumption
cfg.workingRangeMax       = 2000;       % m
cfg.interceptorStationPosition_m = [0; -1000]; % m, fixed south point on 1,000 m inner engagement-zone boundary
% Initial station-to-site radius is intentionally equal to the 1,000 m reference-circle radius.

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
% Two-boundary launch-gate and payload-limit demonstrations (illustrative).
cfg.outerTrackingRadius_m = 2000;        % m, outer track-acquisition ring (assumption)
cfg.innerBoundaryRadius_m = 1000;        % m, reference circle only; not a hard interceptor/capture limit
% Radio and telemetry ranges are independent configurable placeholders. Replace
% these values with tested link-budget/radio-datasheet figures before drawing conclusions.
cfg.telemetryStationPosition_m = [0; 0]; % m, assumed ground-control / telemetry origin
cfg.radioRangeMax_m = 3000;               % m, assumed maximum radio-link distance; not measured
cfg.telemetryRangeMax_m = 3000;            % m, assumed maximum telemetry-link distance; not measured
cfg.maxOperationalLinkRange_m = min(cfg.radioRangeMax_m,cfg.telemetryRangeMax_m);
% Energy guard: reserve enough modeled energy for the remaining return/delivery
% segment plus phase buffer, and enforce the mission sizing ceiling and usable battery.
cfg.returnReserveCruiseSpeed_mps = 15;    % m/s, conservative illustrative return-speed assumption
cfg.returnReserveTimeMargin_s = 10;       % s, extra route/approach allowance; not measured
cfg.targetMassLimit_kg = 5.0;             % kg, nominal maximum target mass
cfg.parachuteSuspendedMassNominal_kg = cfg.targetMassLimit_kg + cfg.netTetherMass_kg; % 5.30 kg
cfg.parachuteDescentRateEstimate_mps = 4.65; % supplied recalculation; estimate, not validated test data
cfg.gateEstimatedInterceptTime_s = 30;   % s, illustrative response-time estimate for gate example
cfg.gateSafetyMargin_s = 2.0;            % s, additional time margin for gate example
cfg.gateResponseTimeTotal_s = cfg.reactionDelay + cfg.gateEstimatedInterceptTime_s + cfg.gateSafetyMargin_s; % 33.8 s total
cfg.workingRangeIsHardGate = false;      % pending guide confirmation: current band is an assessment, not a universal gate
cfg.overweightDemoMass_kg = 6.0;         % kg, hypothetical over-limit target for logic demo

fprintf('\nSKYFORCE corrected sizing baseline\n');
fprintf('  Mass ledger: %.2f kg 16-line subtotal + %.2f kg arithmetic integration/growth headroom = %.2f kg nominal.\n', ...
    cfg.massLedgerSubtotal, cfg.massGrowthAllowance, cfg.massNominal);
fprintf('  Post-capture mass for %.1f kg target: %.2f kg (net/tether already counted in nominal mass).\n', ...
    cfg.targetMassLimit_kg, cfg.massWithCaptured);
fprintf('  Ledger mass placeholders: airframe %.2f, motor set %.2f, ESC set %.2f, propeller set %.2f kg.\n', ...
    cfg.airframeLedgerMass_kg, cfg.motorSetLedgerMass_kg, cfg.escSetLedgerMass_kg, cfg.propellerSetLedgerMass_kg);
fprintf('  Bare-component references are exported separately; ledger estimates are not silently replaced.\n');
fprintf('  Parachute sizing reference: %.2f kg suspended load; estimated descent %.2f m/s (unvalidated estimate).\n', ...
    cfg.parachuteSuspendedMassNominal_kg, cfg.parachuteDescentRateEstimate_mps);
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
fprintf('  Tow cruise target: %.1f m/s; transitions are ramp-limited by %.1f m/s^2.\n', cfg.maxTowSpeed, cfg.accelLimit);
fprintf('  Two-boundary reference: outer tracking radius %.0f m; inner reference radius %.0f m.\n', ...
    cfg.outerTrackingRadius_m, cfg.innerBoundaryRadius_m);
fprintf('  Radio range %.0f m; telemetry range %.0f m; effective link limit %.0f m from (%.0f, %.0f).\n', ...
    cfg.radioRangeMax_m, cfg.telemetryRangeMax_m, cfg.maxOperationalLinkRange_m, ...
    cfg.telemetryStationPosition_m(1), cfg.telemetryStationPosition_m(2));
fprintf('  Pursuit stops when the link limit is reached or return/delivery energy plus phase buffer exceeds the mission ceiling.\n');
fprintf('  Payload policy threshold: %.1f kg; overweight alone does not block launch. A candidate event triggers a simulated parachute-request flag and no tow.\n', cfg.targetMassLimit_kg);
fprintf('  Drag-balance speed estimates %.1f / %.1f m/s are static idealizations, not verified top speeds.\n', ...
    cfg.staticDragBalanceSpeedNominal_mps, cfg.staticDragBalanceSpeedPostCapture_mps);
fprintf('  Energy sizing ceilings: %d Wh no-tow, %d Wh with tow; corrected baseline retires the old 236 Wh figure.\n', ...
    cfg.energyBudgetNoTow, cfg.energyBudgetWithTow);
fprintf('  Battery estimate: %.1f Ah, %.1f Wh nominal, %.1f Wh counted usable (80%%).\n\n', ...
    cfg.batteryCapacity_Ah, cfg.batteryNominalEnergyWh, cfg.batteryUsableEnergyWh);

%% Post-launch mission cases, one PN diagnostic, gate logic, and overweight contingency
% Coordinate system is arbitrary, in metres. x/y are horizontal coordinates.
% Reference circles, starting geometries, and target speeds below are examples
% because the PDF does not specify actual site coordinates or target tracks.

scenarios(1).name = 'Attack mode - slower target, candidate outside safe area';
scenarios(1).pI0 = cfg.interceptorStationPosition_m;
scenarios(1).pT0 = [-1500; 0];
scenarios(1).vT  = [14; 5]; % target velocity vector; speed and heading exported to CSV
scenarios(1).keepOutCenter = [1200; 0];
scenarios(1).keepOutRadius = cfg.innerBoundaryRadius_m;
scenarios(1).safePoints = [-1050, 150; 500, 400]'; % illustrative safe-area centres; translated to site-centred frame
scenarios(1).leadPoint = [];
scenarios(1).expectTow = false;
scenarios(1).vI0 = [0; 0];
scenarios(1).reactionDelayOverride_s = cfg.reactionDelay;
scenarios(1).isGuidanceDiagnostic = false;

scenarios(2).name = 'Defense mode - faster target, lead-point intercept';
scenarios(2).pI0 = cfg.interceptorStationPosition_m;
scenarios(2).pT0 = [-2300; -500]; % faster target approaches a track that crosses the engagement zone
scenarios(2).vT  = [27; 0];
scenarios(2).keepOutCenter = [0; 0];
scenarios(2).keepOutRadius = cfg.innerBoundaryRadius_m;
scenarios(2).safePoints = [-300, -500; 1500, 1500]'; % illustrative release area near the crossing track
scenarios(2).leadPoint = [-300; -500]; % lead point is inside the 1,000 m engagement zone and on the target track
scenarios(2).expectTow = false;
scenarios(2).vI0 = [0; 0];
scenarios(2).reactionDelayOverride_s = cfg.reactionDelay;
scenarios(2).isGuidanceDiagnostic = false;

scenarios(3).name = 'Tow required - capture outside safe area';
scenarios(3).pI0 = cfg.interceptorStationPosition_m;
scenarios(3).pT0 = [-1200; 300];
scenarios(3).vT  = [12; 0];
scenarios(3).keepOutCenter = [1200; 0];
scenarios(3).keepOutRadius = cfg.innerBoundaryRadius_m;
scenarios(3).safePoints = [-400, 400; 0, 600]'; % illustrative safe-area centres; tow completion is computed dynamically
scenarios(3).leadPoint = [];
scenarios(3).expectTow = true;
scenarios(3).vI0 = [0; 0];
scenarios(3).reactionDelayOverride_s = cfg.reactionDelay;
scenarios(3).isGuidanceDiagnostic = false;

scenarios(4).name = 'Abort - target reaches boundary too soon';
scenarios(4).pI0 = cfg.interceptorStationPosition_m;
scenarios(4).pT0 = [1300; 0]; % initially outside the 1,000 m engagement zone, but too little time remains
scenarios(4).vT  = [-20; 0];
scenarios(4).keepOutCenter = [0; 0];
scenarios(4).keepOutRadius = cfg.innerBoundaryRadius_m;
scenarios(4).safePoints = [0, 600; 600, 600]'; % rows are [x y] centres
scenarios(4).leadPoint = [];
scenarios(4).expectTow = false;
scenarios(4).vI0 = [0; 0];
scenarios(4).reactionDelayOverride_s = cfg.reactionDelay;
scenarios(4).isGuidanceDiagnostic = false;
scenarios(4).gateOnlyScenario = true; % late threat: the gate refuses takeoff before launch

% Fifth case requested by the guide: deliberately offset initial heading to
% expose proportional-navigation (PN) corrections. This is an in-flight
% guidance diagnostic, not a launch/reaction-delay mission scenario.
% The translated initial target position preserves the original relative position (1200, 600) m.
scenarios(5).name = 'PN correction diagnostic - offset initial heading (diagnostic only)';
scenarios(5).pI0 = cfg.interceptorStationPosition_m;
scenarios(5).pT0 = [1200; -400]; % translated with the interceptor to preserve the 26.6-deg LOS offset
scenarios(5).vI0 = [20; 0];       % initial flight heading 0 deg, 20 m/s
scenarios(5).vT = [0; 12];        % target heading 90 deg; crossing geometry
scenarios(5).keepOutCenter = [0; 0];
scenarios(5).keepOutRadius = cfg.innerBoundaryRadius_m;
scenarios(5).safePoints = [1200; 550]; % translated with the diagnostic geometry
scenarios(5).leadPoint = [];
scenarios(5).expectTow = false;
scenarios(5).reactionDelayOverride_s = 0;
scenarios(5).isGuidanceDiagnostic = true;
scenarios(5).targetMassEstimate_kg = cfg.targetMassLimit_kg;

% Sixth scenario demonstrates the conservative gate-only case where an outer-ring track
% misses the 1,000 m reference circle; it remains a gate illustration, not an out-of-zone flight. Its 6 kg estimate is
% intentional: mass alone must not veto launch; the track/time gate decides.
% Mission cases are still illustrative post-launch trajectories, separate from gate demo.
scenarios(6).name = 'Gate logic - outer tracking only, no engagement-zone threat';
scenarios(6).pI0 = cfg.interceptorStationPosition_m; % fixed interceptor station on inner boundary
scenarios(6).pT0 = [-3000; 1800];
scenarios(6).vT = [20; 0]; % straight track with 1.8 km miss distance from site centre
scenarios(6).keepOutCenter = [0; 0];
scenarios(6).keepOutRadius = cfg.innerBoundaryRadius_m;
scenarios(6).safePoints = [-100; 600];
scenarios(6).leadPoint = [];
scenarios(6).expectTow = false;
scenarios(6).vI0 = [0; 0];
scenarios(6).reactionDelayOverride_s = cfg.reactionDelay;
scenarios(6).isGuidanceDiagnostic = false;
scenarios(6).targetMassEstimate_kg = 6.0; % mass does not override the trajectory/time launch gate
scenarios(6).gateOnlyScenario = true;

% Seventh case: a >5 kg target reaches a geometric capture-candidate event.
% The policy is immediate simulated parachute-request/no-tow, irrespective of
% safe-area location; actual chute deployment is deliberately not modelled.
scenarios(7) = scenarios(3);
scenarios(7).name = 'Overweight target (>5 kg) - immediate parachute request, no tow';
scenarios(7).targetMassEstimate_kg = cfg.overweightDemoMass_kg;
scenarios(7).gateOnlyScenario = false;
scenarios(7).isOverweightContingencyScenario = true;

% Eighth scenario: integrates outer-ring track acquisition and launch authorization,
% then allows the interceptor to cross the 1 km reference circle while link and
% energy-reserve constraints remain satisfied. It is a stress case for pursuit limits.
scenarios(8).name = 'Extended pursuit - target exits 1 km reference zone';
scenarios(8).pI0 = cfg.interceptorStationPosition_m;
scenarios(8).pT0 = [-3000; 900];
scenarios(8).vT = [27; 0];
scenarios(8).keepOutCenter = [0; 0];
scenarios(8).keepOutRadius = cfg.innerBoundaryRadius_m;
scenarios(8).safePoints = [-300; -500];
scenarios(8).leadPoint = [];
scenarios(8).expectTow = false;
scenarios(8).vI0 = [0; 0];
scenarios(8).reactionDelayOverride_s = cfg.reactionDelay;
scenarios(8).isGuidanceDiagnostic = false;
scenarios(8).targetMassEstimate_kg = cfg.targetMassLimit_kg;
scenarios(8).integratedLaunchGateScenario = true;
scenarios(8).extendedPursuitScenario = true;

% Express all mission tracks in a common site-centred frame. The original scenario
% definitions used different arbitrary keep-out-centre offsets; translate each target
% track, safe-point set and optional lead point by the same offset so their geometry
% relative to the protected site is preserved. Place every mission scenario at the same
% fixed station [0, -1000] m, on the inner engagement-zone boundary. Scenario 5 uses
% the same initial coordinate but retains nonzero velocity as an in-flight diagnostic.
% Scenario 6 remains stationary at the station while its offline gate decision is evaluated.
missionLaunchScenarioIDs = [1 2 3 4 6 7 8];
for launchIdx = missionLaunchScenarioIDs
    siteOffset = scenarios(launchIdx).keepOutCenter(:);
    scenarios(launchIdx).pT0 = scenarios(launchIdx).pT0(:) - siteOffset;
    scenarios(launchIdx).safePoints = scenarios(launchIdx).safePoints - siteOffset;
    if ~isempty(scenarios(launchIdx).leadPoint)
        scenarios(launchIdx).leadPoint = scenarios(launchIdx).leadPoint(:) - siteOffset;
    end
    scenarios(launchIdx).keepOutCenter = [0; 0];
    scenarios(launchIdx).pI0 = cfg.interceptorStationPosition_m;
end

% Validate that every scenario begins at the same fixed interceptor-station
% coordinate on the inner engagement-zone circumference (including the in-flight
% diagnostic's initial position). Equality with the inner radius is intentional.
for scenarioIdx = 1:numel(scenarios)
    stationRadius = norm(scenarios(scenarioIdx).pI0(:) - [0; 0]);
    assert(abs(stationRadius - cfg.innerBoundaryRadius_m) <= 1e-6, ...
        'Interceptor initial position must lie on the %.0f m inner boundary for scenario %d.', ...
        cfg.innerBoundaryRadius_m, scenarioIdx);
end

fprintf('Fixed interceptor station (site-centred XY coordinates): (%.0f, %.0f) m; radius = %.0f m\n', ...
    cfg.interceptorStationPosition_m(1), cfg.interceptorStationPosition_m(2), norm(cfg.interceptorStationPosition_m));
fprintf('All scenario initial interceptor positions are on the %.0f m inner engagement-zone boundary.\n', ...
    cfg.innerBoundaryRadius_m);

% Run the communication-guard predicate test independently, before the flight
% scenarios. The test does not depend on any scenario result or vehicle dynamics.
communicationGuardTestTable = runCommunicationGuardUnitTest(cfg);

%% Run cases
results = cell(1, numel(scenarios));
for i = 1:numel(scenarios)
    results{i} = simulateOneScenario(scenarios(i), cfg);
    results{i}.scenarioID = i;
    results{i} = normalizeEnergyReporting(results{i}, cfg);
    if ~results{i}.isGuidanceDiagnostic
        plotOneScenario(results{i}, cfg);
    end
end

plotTerminalApproachSummary(results, cfg);
plotPNCorrectionDiagnostic(results, cfg);

% Separate offline demonstrations of the requested launch gate and overload
% policy. They do not send commands to radar, autopilot, ESC, or parachute hardware.
boundaryGateTable = runBoundaryGateDemo(cfg);
overloadPolicyTable = runOverweightContingencyDemo(cfg);

% Export every generated figure to the same folder used for the CSV exports.
% Each figure is saved as a report-ready PNG and an editable MATLAB FIG.
figureExportManifest = exportAllFiguresToFolder(pwd);
if ~isempty(figureExportManifest)
    fprintf('\nSaved %d figures as PNG and editable FIG files in: %s\n', ...
        height(figureExportManifest), pwd);
end

%% Summary table: mission estimates, exact event state and propulsion sensitivity
n = numel(results);
Scenario = strings(n,1);
ScenarioPurpose = strings(n,1);
ScenarioID = (1:n)';
Mode = strings(n,1);
LaunchGateDecision = strings(n,1);
LaunchAuthorizedInScenario = false(n,1);
IntegratedLaunchGateScenario = false(n,1);
ExtendedPursuitScenario = false(n,1);
TargetEnteredInnerReferenceCircle = false(n,1);
TargetExitedInnerZoneBeforeCandidate = false(n,1);
InnerZoneExitTime_s = nan(n,1);
InnerZoneExitX_m = nan(n,1); InnerZoneExitY_m = nan(n,1);
TrackAcquisitionTime_s = nan(n,1);
PredictedTimeToReferenceCircleAtTrack_s = nan(n,1);
TargetMassEstimate_kg = nan(n,1);
TargetMassLimit_kg = repmat(cfg.targetMassLimit_kg,n,1);
OverweightTargetAboveLimit = false(n,1);
ParachuteRequestFlag_SimOnly = false(n,1);
TowDisabledForOverweight = false(n,1);
PostCaptureMassEstimate_kg = nan(n,1);
InitialRange_m = nan(n,1);
InterceptorStationX_m = nan(n,1);
InterceptorStationY_m = nan(n,1);
InterceptorStationRadiusFromProtectedSite_m = nan(n,1);
InterceptorOnInnerBoundary = false(n,1);
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
CaptureKinematicsQualification = strings(n,1);
CapturePhysicsValidated = false(n,1); % no physical capture model exists in this script
MassBaselineStatus = repmat("Provisional first-pass ledger; physical installed mass not measured",n,1);
CandidateGuidanceCommandX_mps2 = nan(n,1); CandidateGuidanceCommandY_mps2 = nan(n,1);
CandidateAppliedAccelerationX_mps2 = nan(n,1); CandidateAppliedAccelerationY_mps2 = nan(n,1);
CandidateActiveLimit = strings(n,1);
CommandLeadingIntoCandidateX_mps2 = nan(n,1); CommandLeadingIntoCandidateY_mps2 = nan(n,1);
AppliedAccelerationIntoCandidateX_mps2 = nan(n,1); AppliedAccelerationIntoCandidateY_mps2 = nan(n,1);
ActiveLimitIntoCandidate = strings(n,1);
PNActiveAtCandidate = false(n,1);
PNAccelerationAtCandidate_mps2 = nan(n,1);
PNPeakAccelerationObserved_mps2 = nan(n,1);
PNComponentPeakExceedsModelCap = false(n,1);
PeakRequestedAcceleration_mps2 = nan(n,1);
RequestedAccelerationExceedsModelCap = false(n,1);
RequestedAccelerationOverCapDuration_s = nan(n,1);
RequestedAccelerationPeakToCapRatio = nan(n,1);
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
WorkingRangeStatus = strings(n,1);
CaptureWithin5to15m = false(n,1);
GeometricCandidateWindowReached = false(n,1);
CaptureCandidateRejectedByEnergyGuard = false(n,1);
CaptureOutsideInnerBoundary = false(n,1);
RadioRangeLimit_m = repmat(cfg.radioRangeMax_m,n,1);
TelemetryRangeLimit_m = repmat(cfg.telemetryRangeMax_m,n,1);
EffectiveLinkRangeLimit_m = repmat(cfg.maxOperationalLinkRange_m,n,1);
MaxTelemetryDistance_m = nan(n,1);
TelemetryLimitTriggered = false(n,1);
EnergyReserveLimitTriggered = false(n,1);
OperationalLimitReason = strings(n,1);
EstimatedFutureEnergyReserve_Wh = nan(n,1); % legacy-compatible name; excludes phase buffer
EstimatedRemainingSegmentEnergy_Wh = nan(n,1); % clearer alias for the same component
ProjectedMissionEnergyWithReserve_Wh = nan(n,1);
EnergyGuardMissionCeiling_Wh = nan(n,1);
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
TowCompletionRadius_m = repmat(cfg.safeAreaRadius,n,1);
TowCompletionCriterion = strings(n,1);
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
TowCruiseSetpoint_mps = repmat(cfg.maxTowSpeed,n,1);

for i = 1:n
    r = results{i};
    Scenario(i) = string(r.name);
    if r.isGuidanceDiagnostic
        ScenarioPurpose(i) = "PN guidance diagnostic only; initialized in flight with offset heading";
    elseif r.extendedPursuitScenario
        ScenarioPurpose(i) = "Integrated launch-gate and extended-pursuit test; 1 km circle is a reference boundary";
    elseif r.isGateOnlyScenario
        ScenarioPurpose(i) = "Two-boundary gate demonstration only; no flight trajectory";
    elseif r.targetMassEstimate_kg > cfg.targetMassLimit_kg
        ScenarioPurpose(i) = "Overweight-target contingency; immediate simulated parachute request; no tow";
    else
        ScenarioPurpose(i) = "Post-launch trajectory scenario; assumes separate gate authorized launch";
    end
    Mode(i) = string(r.mode);
    LaunchGateDecision(i) = string(r.launchGateDecision);
    LaunchAuthorizedInScenario(i) = r.launchAuthorized;
    IntegratedLaunchGateScenario(i) = r.integratedLaunchGateScenario;
    ExtendedPursuitScenario(i) = r.extendedPursuitScenario;
    TargetEnteredInnerReferenceCircle(i) = r.targetEnteredInnerReferenceCircle;
    TargetExitedInnerZoneBeforeCandidate(i) = r.targetExitedInnerZoneBeforeCandidate;
    InnerZoneExitTime_s(i) = r.innerZoneExitTime_s;
    InnerZoneExitX_m(i) = r.innerZoneExitPosition(1); InnerZoneExitY_m(i) = r.innerZoneExitPosition(2);
    TrackAcquisitionTime_s(i) = r.trackAcquisitionTime;
    PredictedTimeToReferenceCircleAtTrack_s(i) = r.predictedTimeToReferenceCircleAtTrack_s;
    TargetMassEstimate_kg(i) = r.targetMassEstimate_kg;
    OverweightTargetAboveLimit(i) = r.targetMassEstimate_kg > cfg.targetMassLimit_kg;
    ParachuteRequestFlag_SimOnly(i) = r.parachuteRequestFlag;
    TowDisabledForOverweight(i) = r.towDisabledForOverweight;
    PostCaptureMassEstimate_kg(i) = r.postCaptureMassEstimate_kg;
    InitialRange_m(i) = r.initialRange;
    InterceptorStationX_m(i) = r.pI(1,1);
    InterceptorStationY_m(i) = r.pI(2,1);
    InterceptorStationRadiusFromProtectedSite_m(i) = norm(r.pI(:,1));
    InterceptorOnInnerBoundary(i) = ...
        abs(InterceptorStationRadiusFromProtectedSite_m(i) - cfg.innerBoundaryRadius_m) <= 1e-6;
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
        CaptureKinematicsQualification(i) = string(sprintf(['V_close = %.2f m/s is a relative-kinematic result only; ' ...
            'no validated safe capture-speed threshold or net/tether capture physics is modelled.'], cs.closingSpeed));
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
    else
        CaptureKinematicsQualification(i) = "No geometric candidate registered; physical capture was not assessed.";
    end
    TimeToCapture_s(i) = r.captureTime;
    ClosestRange_m(i) = r.minimumRange;
    MaxSpeedObserved_mps(i) = r.maxSpeedObserved;
    SpeedWithin25mpsLimit(i) = r.maxSpeedObserved <= cfg.maxSpeed + 1e-6;
    MaxAccelerationObserved_mps2(i) = r.maxAccelerationObserved;
    AssumedAccelerationLimit_mps2(i) = cfg.accelLimit;
    AccelerationWithinAssumedLimit(i) = r.maxAccelerationObserved <= cfg.accelLimit + 0.05;
    WithinWorkingRange_1p5to2km(i) = r.initialRange >= cfg.workingRangeMin && r.initialRange <= cfg.workingRangeMax;
    if r.isGuidanceDiagnostic
        WorkingRangeStatus(i) = "Not applicable: in-flight PN diagnostic, not a launch-range trial";
    elseif r.isGateOnlyScenario
        WorkingRangeStatus(i) = "Not applicable: launch-gate track demonstration, no interceptor launch";
    elseif r.integratedLaunchGateScenario && ~r.launchAuthorized
        WorkingRangeStatus(i) = "Not applicable: integrated gate declined launch";
    elseif WithinWorkingRange_1p5to2km(i)
        WorkingRangeStatus(i) = "Within 1.5-2.0 km initial-range band";
    else
        WorkingRangeStatus(i) = "Outside 1.5-2.0 km band; confirm whether this scenario exception is permitted";
    end
    CaptureWithin5to15m(i) = isfinite(r.captureRangeAtEvent) && ...
        r.captureRangeAtEvent >= cfg.captureRange(1) && r.captureRangeAtEvent <= cfg.captureRange(2);
    GeometricCandidateWindowReached(i) = r.geometricCandidateWindowReached;
    CaptureCandidateRejectedByEnergyGuard(i) = r.captureCandidateRejectedByEnergyGuard;
    CaptureOutsideInnerBoundary(i) = r.captureOutsideInnerBoundary;
    MaxTelemetryDistance_m(i) = r.maxTelemetryDistance_m;
    TelemetryLimitTriggered(i) = r.telemetryLimitTriggered;
    EnergyReserveLimitTriggered(i) = r.energyReserveLimitTriggered;
    OperationalLimitReason(i) = string(r.operationalLimitReason);
    EstimatedFutureEnergyReserve_Wh(i) = r.estimatedFutureEnergyWh;
    EstimatedRemainingSegmentEnergy_Wh(i) = r.estimatedFutureEnergyWh;
    ProjectedMissionEnergyWithReserve_Wh(i) = r.projectedMissionEnergyWithReserveWh;
    EnergyGuardMissionCeiling_Wh(i) = r.missionEnergyCeilingWh;
    EnergyUsed_SimulatedPhases_Wh(i) = r.energyUsedSimulatedPhasesWh;
    EnergyBudget_SizingCeiling_Wh(i) = r.energyBudget;
    PNPeakAccelerationObserved_mps2(i) = max(r.pnAccelerationMagnitude);
    PNComponentPeakExceedsModelCap(i) = PNPeakAccelerationObserved_mps2(i) > cfg.accelLimit + 1e-6;
    % The total requested command is the input to the acceleration limiter.
    % Exclude the capture-candidate sample because no integration step is applied there.
    requestedMagnitude = sqrt(sum(r.commandedAcceleration.^2,1))';
    integratedCommandSample = isfinite(requestedMagnitude) & (r.phase(:) ~= "CAPTURE_CANDIDATE");
    if any(integratedCommandSample)
        PeakRequestedAcceleration_mps2(i) = max(requestedMagnitude(integratedCommandSample));
        overCapSamples = integratedCommandSample & requestedMagnitude > cfg.accelLimit + 1e-6;
        RequestedAccelerationExceedsModelCap(i) = any(overCapSamples);
        RequestedAccelerationOverCapDuration_s(i) = sum(overCapSamples) * cfg.dt;
        RequestedAccelerationPeakToCapRatio(i) = PeakRequestedAcceleration_mps2(i) / max(cfg.accelLimit,eps);
    end
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
    if contains(r.releaseTowDecision, 'Tow required') || r.towCompleted || ...
            (r.energyReserveLimitTriggered && any(r.phase == "TOW"))
        TowCompletionCriterion(i) = string(sprintf(['Enter designated %.1f m safe-area radius within the tow time limit; ' ...
            'centre arrival and landing are not modelled'], cfg.safeAreaRadius));
    else
        TowCompletionCriterion(i) = "Not applicable: no tow phase";
    end

    % All exported energy totals come from one normalization function so the
    % projected total always equals: simulated energy + remaining-segment estimate
    % + phase buffer, when those components are meaningfully available.
    AdditionalPhaseBuffer_Wh(i) = r.additionalPhaseBufferWh;
    CompleteMissionEnergyEstimate_Wh(i) = r.completeMissionEnergyEstimateWh;
    CompleteMissionEstimateWithinCeiling(i) = r.completeMissionEstimateWithinCeiling;
    EnergyBudgetStatus(i) = string(r.energyBudgetStatus);
end

summaryTable = table(ScenarioID, Scenario, ScenarioPurpose, Mode, LaunchGateDecision, ...
    LaunchAuthorizedInScenario, IntegratedLaunchGateScenario, ExtendedPursuitScenario, ...
    TargetEnteredInnerReferenceCircle, TargetExitedInnerZoneBeforeCandidate, InnerZoneExitTime_s, ...
    InnerZoneExitX_m, InnerZoneExitY_m, TrackAcquisitionTime_s, PredictedTimeToReferenceCircleAtTrack_s, ...
    TargetMassEstimate_kg, TargetMassLimit_kg, OverweightTargetAboveLimit, ParachuteRequestFlag_SimOnly, ...
    TowDisabledForOverweight, PostCaptureMassEstimate_kg, InitialRange_m, ...
    InterceptorStationX_m, InterceptorStationY_m, InterceptorStationRadiusFromProtectedSite_m, ...
    InterceptorOnInnerBoundary, ...
    TargetSpeed_mps, TargetVelocityX_mps, TargetVelocityY_mps, TargetHeading_deg, ...
    InitialInterceptorSpeed_mps, InitialInterceptorHeading_deg, InitialLOSAngle_deg, InitialHeadingOffset_deg, ...
    CaptureSampleIndex, CaptureTime_s, CaptureCandidateRange_m, ...
    CaptureInterceptorX_m, CaptureInterceptorY_m, CaptureInterceptorVx_mps, CaptureInterceptorVy_mps, ...
    CaptureTargetX_m, CaptureTargetY_m, CaptureTargetVx_mps, CaptureTargetVy_mps, ...
    CaptureLOSUnitX, CaptureLOSUnitY, RangeRateAtCandidate_mps, ClosingSpeedAtCandidate_mps, ...
    CaptureKinematicsQualification, CapturePhysicsValidated, MassBaselineStatus, ...
    CandidateGuidanceCommandX_mps2, CandidateGuidanceCommandY_mps2, ...
    CandidateAppliedAccelerationX_mps2, CandidateAppliedAccelerationY_mps2, CandidateActiveLimit, ...
    CommandLeadingIntoCandidateX_mps2, CommandLeadingIntoCandidateY_mps2, ...
    AppliedAccelerationIntoCandidateX_mps2, AppliedAccelerationIntoCandidateY_mps2, ActiveLimitIntoCandidate, ...
    PNActiveAtCandidate, PNAccelerationAtCandidate_mps2, PNPeakAccelerationObserved_mps2, ...
    PNComponentPeakExceedsModelCap, PeakRequestedAcceleration_mps2, RequestedAccelerationExceedsModelCap, ...
    RequestedAccelerationOverCapDuration_s, RequestedAccelerationPeakToCapRatio, ...
    PNActiveDuration_s, GuidanceCorrectionDemonstrated, TimeToCapture_s, ClosestRange_m, ...
    MaxSpeedObserved_mps, SpeedWithin25mpsLimit, MaxAccelerationObserved_mps2, ...
    AssumedAccelerationLimit_mps2, AccelerationWithinAssumedLimit, WithinWorkingRange_1p5to2km, WorkingRangeStatus, ...
    CaptureWithin5to15m, GeometricCandidateWindowReached, CaptureCandidateRejectedByEnergyGuard, ...
    CaptureOutsideInnerBoundary, RadioRangeLimit_m, TelemetryRangeLimit_m, ...
    EffectiveLinkRangeLimit_m, MaxTelemetryDistance_m, TelemetryLimitTriggered, EnergyReserveLimitTriggered, ...
    OperationalLimitReason, EstimatedFutureEnergyReserve_Wh, ProjectedMissionEnergyWithReserve_Wh, ...
    EstimatedRemainingSegmentEnergy_Wh, EnergyGuardMissionCeiling_Wh, ...
    EnergyUsed_SimulatedPhases_Wh, AdditionalPhaseBuffer_Wh, ...
    CompleteMissionEnergyEstimate_Wh, EnergyBudget_SizingCeiling_Wh, CompleteMissionEstimateWithinCeiling, ...
    EnergyBudgetStatus, FullMissionEnergyValidated, MissionOutcome, CaptureCandidateRegistered, ...
    CaptureInsideSafeArea, NearestSafePointDistanceAtCapture_m, SafeAreaRadius_m, ...
    ReleaseTowDecision, TowDuration_s, TowCompletedWithin60s, TowCompletionRadius_m, TowCompletionCriterion, ...
    NominalMass_kg, PostCaptureMass_kg, MassLedgerSubtotal_kg, IntegrationGrowthAllowance_kg, ...
    MotorModel, PropellerModel, ESCModel, PropellerDiameter_in, ArmLength_m, FrameSpan_m, ...
    MaxTotalStaticThrust_N, NominalTWR_Static, PostCaptureTWR_Static, ...
    StaticAvailableAccelerationNominal_mps2, StaticAvailableAccelerationPostCapture_mps2, ...
    ThrustMarginAt25mps_Nominal, ThrustMarginAt25mps_PostCapture, ...
    Conservative30pctThrustMarginNominal, Conservative30pctThrustMarginPostCapture, ...
    StaticDragBalanceSpeedNominal_mps, StaticDragBalanceSpeedPostCapture_mps, ...
    MotorMassEach_kg, MotorSetMass_kg, BatteryMass_kg, BatteryCapacity_Ah, ...
    BatteryNominalEnergy_Wh, BatteryUsableEnergy_Wh, TowCruiseSetpoint_mps);

disp(' ');
disp('SKYFORCE point-mass simulation summary');
disp(summaryTable);

% Separate full-resolution trajectory log supports independent closing-speed
% verification and later phase-by-phase energy integration.
trajectoryTable = buildTrajectoryExport(results,cfg);
workingRangeAssessmentTable = buildWorkingRangeAssessment(results,cfg);
propulsionSensitivityTable = buildPropulsionSensitivityTable(cfg);
massLedgerTable = buildMassLedgerTable(cfg);
componentMassReferenceTable = buildComponentMassReferenceTable(cfg);
modelAssumptionsTable = buildModelAssumptionsTable(cfg);
fprintf('\nInitial working-range assessment (band applicability pending guide confirmation):\n');
disp(workingRangeAssessmentTable);
fprintf('\nForward-flight thrust sensitivity (assumed de-rating; not measured):\n');
disp(propulsionSensitivityTable);
summarySavedPath = writeTableWithFallback(summaryTable, 'skyforce_simulation_summary.csv');
trajectorySavedPath = writeTableWithFallback(trajectoryTable, 'skyforce_trajectory_export.csv');
workingRangeSavedPath = writeTableWithFallback(workingRangeAssessmentTable, 'skyforce_working_range_assessment.csv');
sensitivitySavedPath = writeTableWithFallback(propulsionSensitivityTable, 'skyforce_propulsion_sensitivity.csv');
massLedgerSavedPath = writeTableWithFallback(massLedgerTable, 'skyforce_mass_ledger.csv');
componentReferenceSavedPath = writeTableWithFallback(componentMassReferenceTable, 'skyforce_component_mass_references.csv');
assumptionRegisterSavedPath = writeTableWithFallback(modelAssumptionsTable, 'skyforce_model_assumptions_and_limitations.csv');
gateSavedPath = writeTableWithFallback(boundaryGateTable, 'skyforce_boundary_gate_summary.csv');
overloadSavedPath = writeTableWithFallback(overloadPolicyTable, 'skyforce_overweight_policy_summary.csv');
figureManifestSavedPath = writeTableWithFallback(figureExportManifest, 'skyforce_figure_export_manifest.csv');
communicationGuardTestSavedPath = writeTableWithFallback(communicationGuardTestTable, ...
    'skyforce_communication_guard_unit_test.csv');
if ~isempty(summarySavedPath), fprintf('\nSaved scenario summary to: %s\n', summarySavedPath); end
if ~isempty(trajectorySavedPath), fprintf('Saved sample-by-sample trajectory export to: %s\n', trajectorySavedPath); end
if ~isempty(workingRangeSavedPath), fprintf('Saved 1.5-2.0 km working-range assessment to: %s\n', workingRangeSavedPath); end
if ~isempty(sensitivitySavedPath), fprintf('Saved propulsion sensitivity table to: %s\n', sensitivitySavedPath); end
if ~isempty(massLedgerSavedPath), fprintf('Saved 16-line mass ledger to: %s\n', massLedgerSavedPath); end
if ~isempty(componentReferenceSavedPath), fprintf('Saved separate bare-component references to: %s\n', componentReferenceSavedPath); end
if ~isempty(assumptionRegisterSavedPath), fprintf('Saved model assumptions/limitations register to: %s\n', assumptionRegisterSavedPath); end
if ~isempty(gateSavedPath), fprintf('Saved two-boundary launch-gate demo to: %s\n', gateSavedPath); end
if ~isempty(overloadSavedPath), fprintf('Saved overweight policy demo to: %s\n', overloadSavedPath); end
if ~isempty(figureManifestSavedPath), fprintf('Saved figure export manifest to: %s\n', figureManifestSavedPath); end
if ~isempty(communicationGuardTestSavedPath), fprintf('Saved communication-guard unit test to: %s\n', communicationGuardTestSavedPath); end

fprintf('\nSimulation complete. All 5-15 m events are geometric capture candidates only.\n');
fprintf('No net-deployment success, capture probability, or parachute performance is modelled.\n');
fprintf('Candidate closing speeds are kinematic only; no acceptable physical capture-speed limit is validated in this model.\n');
fprintf('Two-boundary tracking and parachute-request outputs are simulation-only; no hardware commands are issued.\n');
fprintf('The %.0f m circle is a reference boundary. Pursuit may continue outside it until the effective %.0f m link limit or energy guard stops it.\n', ...
    cfg.innerBoundaryRadius_m, cfg.maxOperationalLinkRange_m);
fprintf('Energy fields are consistently reported as simulated phase energy + estimated remaining segment + phase buffer, where defined.\n');
fprintf('Failed tow and unmodelled overweight recovery are reported as phase energy only; no complete-mission projection is claimed.\n');
fprintf('The communication-range predicate has passed an isolated unit test; this verifies logic only, not flight dynamics.\n');
fprintf('The fixed interceptor station is (%.0f, %.0f) m; radio/telemetry distance is measured from (%.0f, %.0f) m.\n', ...
    cfg.interceptorStationPosition_m(1), cfg.interceptorStationPosition_m(2), ...
    cfg.telemetryStationPosition_m(1), cfg.telemetryStationPosition_m(2));
fprintf('The PN diagnostic starts from the same boundary coordinate with nonzero velocity and preserves its 26.6-degree heading/LOS offset.\n');
fprintf('Inner-circle rule: %.0f m is a plotted reference boundary; pursuit/candidate events may occur outside it if link and energy guards pass.\n', cfg.innerBoundaryRadius_m);
fprintf('Radio/telemetry ranges are provisional assumptions; the stricter configured range is enforced from the control-site origin.\n');
fprintf('Launch-gate response budget: %.1f s reaction/latency + %.1f s flight-response estimate + %.1f s margin = %.1f s total.\n', ...
    cfg.reactionDelay, cfg.gateEstimatedInterceptTime_s, cfg.gateSafetyMargin_s, cfg.gateResponseTimeTotal_s);
fprintf('The 1.5-2.0 km initial-range band is a reported compliance check, not a universal hard gate.\n');
fprintf('Raw guidance demand, controller-limited command and applied acceleration are logged separately; %.1f s velocity-response time is assumed, not measured.\n', cfg.interceptVelocityResponseTime_s);
fprintf('Conservative assumed 30%% forward-flight thrust de-rating margins: %.2fx nominal / %.2fx post-capture.\n', ...
    cfg.thrustMargin30pctNominal, cfg.thrustMargin30pctPostCapture);
fprintf('Mass ledger is a provisional first-pass baseline; the exact 14.78 kg sum does not verify installed hardware mass.\n');
fprintf('Power, battery, drag, thrust de-rating, mass ledger, link range and phase buffers are provisional inputs pending physical verification.\n');
fprintf('Datalink 0.50 kg ledger entry is an unresolved placeholder; bare modem reference is 0.014 kg.\n');
fprintf('Battery product mass, frame/mount masses, and forward-flight thrust loss remain to be confirmed.\n');

end

%% ------------------------------------------------------------------------
function tf = linkRangeExceeded(distance_m, linkLimit_m)
% Shared radio/telemetry limit predicate used by runtime guards and unit tests.
% Equality with the configured maximum is permitted; exceeding it by more than
% 1e-6 m triggers the guard.
tolerance_m = 1e-6;
tf = isfinite(distance_m) && distance_m > linkLimit_m + tolerance_m;
end

function T = runCommunicationGuardUnitTest(cfg)
% Logic-only boundary test for the same predicate used by both flight-loop checks.
% It does not simulate vehicle dynamics or claim any physical link performance.
limit_m = cfg.maxOperationalLinkRange_m;
CaseName = ["Below limit"; "Exactly at limit"; "Predicted next step crosses limit"; "Current position already outside limit"];
CurrentPositionDistance_m = [limit_m-1; limit_m; limit_m-1; limit_m+1];
PredictedNextPositionDistance_m = [limit_m-0.5; limit_m; limit_m+1; limit_m+1];
EffectiveLinkLimit_m = repmat(limit_m,4,1);
ExpectedCurrentPositionGuard = [false; false; false; true];
ExpectedPredictedNextStepGuard = [false; false; true; true];
ActualCurrentPositionGuard = false(4,1);
ActualPredictedNextStepGuard = false(4,1);
for k = 1:4
    ActualCurrentPositionGuard(k) = linkRangeExceeded(CurrentPositionDistance_m(k),limit_m);
    ActualPredictedNextStepGuard(k) = linkRangeExceeded(PredictedNextPositionDistance_m(k),limit_m);
end
TestPassed = ActualCurrentPositionGuard == ExpectedCurrentPositionGuard & ...
    ActualPredictedNextStepGuard == ExpectedPredictedNextStepGuard;
T = table(CaseName,CurrentPositionDistance_m,PredictedNextPositionDistance_m, ...
    EffectiveLinkLimit_m,ExpectedCurrentPositionGuard,ActualCurrentPositionGuard, ...
    ExpectedPredictedNextStepGuard,ActualPredictedNextStepGuard,TestPassed);
assert(all(TestPassed), 'Communication-range guard unit test failed.');
fprintf('\nCommunication-guard logic test: %d/%d cases passed at %.0f m effective limit.\n', ...
    sum(TestPassed),numel(TestPassed),limit_m);
fprintf('This is a predicate/boundary test only; it does not validate radio hardware or flight dynamics.\n');
end

function out = normalizeEnergyReporting(out,cfg)
% Centralize energy accounting for plot labels and summary exports.
% The projected total is always simulated phase energy + estimated remaining
% segment + phase buffer, whenever that projection is meaningful. This is a
% preliminary estimate; it is not physical-performance validation.
out.energyUsedSimulatedPhasesWh = out.energyWh(end);
out.additionalPhaseBufferWh = NaN;
out.completeMissionEnergyEstimateWh = NaN;
out.completeMissionEstimateWithinCeiling = false;
out.energyBudgetStatus = "";

% Do not present a complete-mission projection for diagnostic/gate-only cases.
if out.isGuidanceDiagnostic
    out.estimatedFutureEnergyWh = NaN;
    out.projectedMissionEnergyWithReserveWh = NaN;
    out.missionEnergyCeilingWh = NaN;
    out.requiredPhaseBufferWh = NaN;
    out.energyBudgetStatus = "Not applicable: PN guidance diagnostic, not a mission-energy case";
elseif out.isGateOnlyScenario || ~out.launchAuthorized
    out.estimatedFutureEnergyWh = NaN;
    out.projectedMissionEnergyWithReserveWh = NaN;
    out.missionEnergyCeilingWh = NaN;
    out.requiredPhaseBufferWh = NaN;
    out.energyBudgetStatus = "Not applicable: launch-gate decision only; no mission flight simulated";
elseif out.parachuteRequestFlag
    % Recovery/deployment is outside this model; do not imply a total mission
    % projection for the overweight-target contingency.
    out.estimatedFutureEnergyWh = NaN;
    out.projectedMissionEnergyWithReserveWh = NaN;
    out.missionEnergyCeilingWh = NaN;
    out.requiredPhaseBufferWh = NaN;
    out.energyBudgetStatus = "Phase energy only: overweight recovery/deployment is not simulated; no complete-mission projection";
elseif out.towCompleted
    % The current model endpoint is entry into the designated safe-area radius.
    % No subsequent landing or return-to-base segment is represented.
    out.estimatedFutureEnergyWh = 0;
    out.requiredPhaseBufferWh = cfg.energyBufferTow_Wh;
    out.additionalPhaseBufferWh = cfg.energyBufferTow_Wh;
    out.missionEnergyCeilingWh = min(cfg.energyBudgetWithTow,cfg.batteryUsableEnergyWh);
    out.projectedMissionEnergyWithReserveWh = out.energyUsedSimulatedPhasesWh + ...
        out.estimatedFutureEnergyWh + out.additionalPhaseBufferWh;
    out.completeMissionEnergyEstimateWh = out.projectedMissionEnergyWithReserveWh;
    out.completeMissionEstimateWithinCeiling = out.projectedMissionEnergyWithReserveWh <= out.missionEnergyCeilingWh;
    out.energyBudgetStatus = "Projection to model endpoint (safe-area entry) = simulated tow energy + tow buffer; subsequent landing/return not modelled";
elseif out.energyReserveLimitTriggered || out.telemetryLimitTriggered
    % For a guard stop, retain the guard's final remaining-segment estimate and
    % recompute the total from the same three components used in terminal text.
    if ~isfinite(out.requiredPhaseBufferWh)
        if contains(out.releaseTowDecision,'Tow required') || any(out.phase == "TOW")
            out.requiredPhaseBufferWh = cfg.energyBufferTow_Wh;
        else
            out.requiredPhaseBufferWh = cfg.energyBufferNoTow_Wh;
        end
    end
    if ~isfinite(out.estimatedFutureEnergyWh)
        [~, futureWh, ~, ceilingWh, bufferWh] = evaluateEnergyReserve( ...
            out.pI(:,end),out.energyUsedSimulatedPhasesWh,cfg.massNominal,'INTERCEPT', ...
            [NaN;NaN],out.energyBudget,false,cfg);
        out.estimatedFutureEnergyWh = futureWh;
        if ~isfinite(out.requiredPhaseBufferWh), out.requiredPhaseBufferWh = bufferWh; end
        if ~isfinite(out.missionEnergyCeilingWh), out.missionEnergyCeilingWh = ceilingWh; end
    end
    out.additionalPhaseBufferWh = out.requiredPhaseBufferWh;
    if ~isfinite(out.missionEnergyCeilingWh)
        candidateBudget = out.energyBudget;
        if ~isfinite(candidateBudget) || candidateBudget <= 0
            candidateBudget = cfg.energyBudgetNoTow;
        end
        out.missionEnergyCeilingWh = min(candidateBudget,cfg.batteryUsableEnergyWh);
    end
    out.projectedMissionEnergyWithReserveWh = out.energyUsedSimulatedPhasesWh + ...
        out.estimatedFutureEnergyWh + out.additionalPhaseBufferWh;
    out.completeMissionEnergyEstimateWh = out.projectedMissionEnergyWithReserveWh;
    out.completeMissionEstimateWithinCeiling = out.projectedMissionEnergyWithReserveWh <= out.missionEnergyCeilingWh;
    if out.energyReserveLimitTriggered
        out.energyBudgetStatus = "Projected energy (used + remaining segment + buffer) exceeds ceiling; pursuit stopped by energy guard";
    else
        out.energyBudgetStatus = "Communication guard stopped pursuit; energy projection uses estimated remaining segment and buffer";
    end
elseif contains(out.releaseTowDecision,'Tow required') && ~out.towCompleted
    % Failed/incomplete delivery means the remaining delivery and recovery energy
    % is not known from the simulated phase; do not extrapolate a final total.
    out.estimatedFutureEnergyWh = NaN;
    out.projectedMissionEnergyWithReserveWh = NaN;
    out.missionEnergyCeilingWh = min(cfg.energyBudgetWithTow,cfg.batteryUsableEnergyWh);
    out.requiredPhaseBufferWh = NaN;
    out.energyBudgetStatus = "Tow did not enter safe area within 60 s; simulated-phase energy only, no complete-mission projection";
else
    % For ordinary no-tow cases, preserve the latest direct-return estimate. If the
    % loop ended before one was available, recompute that same simple reserve model
    % from the final state. No physical recovery dynamics are added.
    if ~isfinite(out.requiredPhaseBufferWh)
        out.requiredPhaseBufferWh = cfg.energyBufferNoTow_Wh;
    end
    if ~isfinite(out.estimatedFutureEnergyWh)
        [~,futureWh,~,ceilingWh,bufferWh] = evaluateEnergyReserve( ...
            out.pI(:,end),out.energyUsedSimulatedPhasesWh,cfg.massNominal,'INTERCEPT', ...
            [NaN;NaN],out.energyBudget,false,cfg);
        out.estimatedFutureEnergyWh = futureWh;
        if ~isfinite(out.requiredPhaseBufferWh), out.requiredPhaseBufferWh = bufferWh; end
        if ~isfinite(out.missionEnergyCeilingWh), out.missionEnergyCeilingWh = ceilingWh; end
    end
    out.additionalPhaseBufferWh = out.requiredPhaseBufferWh;
    if ~isfinite(out.missionEnergyCeilingWh)
        candidateBudget = out.energyBudget;
        if ~isfinite(candidateBudget) || candidateBudget <= 0
            candidateBudget = cfg.energyBudgetNoTow;
        end
        out.missionEnergyCeilingWh = min(candidateBudget,cfg.batteryUsableEnergyWh);
    end
    out.projectedMissionEnergyWithReserveWh = out.energyUsedSimulatedPhasesWh + ...
        out.estimatedFutureEnergyWh + out.additionalPhaseBufferWh;
    out.completeMissionEnergyEstimateWh = out.projectedMissionEnergyWithReserveWh;
    out.completeMissionEstimateWithinCeiling = out.projectedMissionEnergyWithReserveWh <= out.missionEnergyCeilingWh;
    if out.completeMissionEstimateWithinCeiling
        out.energyBudgetStatus = "Projected energy = simulated energy + estimated remaining segment + phase buffer; below sizing ceiling; remaining segment not dynamically simulated";
    else
        out.energyBudgetStatus = "Projected energy = simulated energy + estimated remaining segment + phase buffer; exceeds sizing ceiling";
    end
end

% Keep plotted terminal projection checkpoints consistent with the normalized
% summary at the final simulated sample; earlier samples remain time-varying checks.
if isfield(out,'estimatedFutureEnergyWhHistory') && ~isempty(out.estimatedFutureEnergyWhHistory)
    if isfinite(out.estimatedFutureEnergyWh)
        out.estimatedFutureEnergyWhHistory(end) = out.estimatedFutureEnergyWh;
    else
        out.estimatedFutureEnergyWhHistory(end) = NaN;
    end
end
if isfield(out,'projectedMissionEnergyWhHistory') && ~isempty(out.projectedMissionEnergyWhHistory)
    if isfinite(out.projectedMissionEnergyWithReserveWh)
        out.projectedMissionEnergyWhHistory(end) = out.projectedMissionEnergyWithReserveWh;
    else
        out.projectedMissionEnergyWhHistory(end) = NaN;
    end
end
if isfield(out,'missionEnergyCeilingWhHistory') && ~isempty(out.missionEnergyCeilingWhHistory)
    if isfinite(out.missionEnergyCeilingWh)
        out.missionEnergyCeilingWhHistory(end) = out.missionEnergyCeilingWh;
    else
        out.missionEnergyCeilingWhHistory(end) = NaN;
    end
end

% Runtime consistency checks for the normalized energy fields. These assertions
% validate arithmetic/reporting only, not aircraft performance or mission feasibility.
if isfinite(out.projectedMissionEnergyWithReserveWh)
    energyComponents = [out.energyUsedSimulatedPhasesWh, ...
        out.estimatedFutureEnergyWh, out.additionalPhaseBufferWh];
    assert(all(isfinite(energyComponents)), ...
        'Projected energy is defined but one of its energy components is not finite.');
    componentSumWh = sum(energyComponents);
    assert(abs(out.projectedMissionEnergyWithReserveWh-componentSumWh) <= ...
        1e-8*max(1,abs(componentSumWh)), ...
        'Projected mission energy does not equal used + remaining segment + phase buffer.');
end
if isfinite(out.completeMissionEnergyEstimateWh)
    assert(isfinite(out.projectedMissionEnergyWithReserveWh) && ...
        abs(out.completeMissionEnergyEstimateWh-out.projectedMissionEnergyWithReserveWh) <= 1e-8* ...
        max(1,abs(out.projectedMissionEnergyWithReserveWh)), ...
        'CompleteMissionEnergyEstimate_Wh must match ProjectedMissionEnergyWithReserve_Wh.');
end
end

function out = simulateOneScenario(s, cfg)
N = floor(cfg.maxSimTime / cfg.dt) + 1;

pI = s.pI0(:);
pT = s.pT0(:);
vI = s.vI0(:);
vT = s.vT(:);
reactionDelay = s.reactionDelayOverride_s;
reactionStartTime = 0; % mission cases assume the separate gate already authorized launch

% Normalize the optional flag once. In a MATLAB struct array, adding a field
% to only one element creates empty values in the other elements. Evaluating
% an empty value directly with && causes the reported scalar-logical error.
gateOnlyScenario = false;
integratedLaunchGateScenario = false;
extendedPursuitScenario = false;
if isfield(s, 'integratedLaunchGateScenario')
    f = s.integratedLaunchGateScenario;
    if (islogical(f) || isnumeric(f)) && isscalar(f) && ~isempty(f) && isfinite(double(f)), integratedLaunchGateScenario = logical(f); end
end
if isfield(s, 'extendedPursuitScenario')
    f = s.extendedPursuitScenario;
    if (islogical(f) || isnumeric(f)) && isscalar(f) && ~isempty(f) && isfinite(double(f)), extendedPursuitScenario = logical(f); end
end
if isfield(s, 'gateOnlyScenario')
    gateFlag = s.gateOnlyScenario;
    if (islogical(gateFlag) || isnumeric(gateFlag)) && isscalar(gateFlag) && ...
            ~isempty(gateFlag) && isfinite(double(gateFlag))
        gateOnlyScenario = logical(gateFlag);
    end
end
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
requestedAh = zeros(2,N);            % raw guidance demand before command saturation
controllerCommandAh = zeros(2,N);     % bounded command passed to point-mass step
activeLimitHistory = repmat("none",N,1);
pnAh = zeros(2,N);
pnActiveHistory = false(N,1);
rh = nan(N,1);
energy = zeros(N,1);
power = zeros(N,1);
phaseHistory = strings(N,1);
telemetryDistanceHistory = nan(N,1);
estimatedFutureEnergyHistory = nan(N,1);
projectedMissionEnergyHistory = nan(N,1);
missionEnergyCeilingHistory = nan(N,1);

% Main post-launch trajectory cases assume the separate gate authorized launch;
% scenario 8 runs the gate and post-takeoff zone-exit failure in one sequence. The
% gate-only scenarios remain stationary while a target track is assessed.
if s.isGuidanceDiagnostic
    phase = 'INTERCEPT';
    launchAuthorized = true;
    reactionStartTime = 0;
    launchGateDecision = 'Not applicable: in-flight PN diagnostic';
elseif gateOnlyScenario || integratedLaunchGateScenario
    phase = 'TRACKING';
    launchAuthorized = false;
    reactionStartTime = NaN;
    launchGateDecision = 'Pending: outer tracking / inner engagement-zone gate evaluation';
else
    phase = 'REACTION';
    launchAuthorized = true;
    reactionStartTime = 0;
    launchGateDecision = 'Assumed authorized for post-launch trajectory scenario';
end
trackAcquisitionTime = NaN;
predictedTimeToReferenceCircleAtTrack_s = NaN;
if isfield(s,'targetMassEstimate_kg') && ~isempty(s.targetMassEstimate_kg)
    targetMassEstimate_kg = s.targetMassEstimate_kg;
else
    targetMassEstimate_kg = cfg.targetMassLimit_kg;
end
parachuteRequestFlag = false;
towDisabledForOverweight = false;
postCaptureMassEstimate_kg = NaN;
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
geometricCandidateWindowReached = false;
captureCandidateRejectedByEnergyGuard = false;
geometricCandidatePoint = [nan; nan];
closingAtCapture = nan;
captureState = struct('sampleIndex',nan,'time',nan,'pI',[nan;nan],'pT',[nan;nan], ...
    'vI',[nan;nan],'vT',[nan;nan],'los',[nan;nan],'range',nan,'rangeRate',nan, ...
    'closingSpeed',nan,'commandedAccel',[nan;nan],'controllerCommandedAccel',[nan;nan],'appliedAccel',[nan;nan], ...
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
targetEnteredInnerReferenceCircle = norm(pT - s.keepOutCenter(:)) <= s.keepOutRadius;
targetExitedInnerZoneBeforeCandidate = false;
innerZoneExitTime_s = NaN;
innerZoneExitPosition = [NaN; NaN];
telemetryLimitTriggered = false;
energyReserveLimitTriggered = false;
operationalLimitReason = '';
estimatedFutureEnergyWh = NaN;
projectedMissionEnergyWithReserveWh = NaN;
missionEnergyCeilingWh = NaN;
requiredPhaseBufferWh = NaN;
maxTelemetryDistance_m = norm(pI - cfg.telemetryStationPosition_m(:));

lastIndex = N;
for k = 1:N
    t = (k-1) * cfg.dt;
    % Label each integrated sample by the operational phase. The reaction
    % phase ends at the configured delay; subsequent pre-capture motion is
    % logged as INTERCEPT (including relocation to the defense lead point).
    if strcmp(phase,'REACTION') && isfinite(reactionStartTime) && ...
            (t - reactionStartTime) >= reactionDelay
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
    telemetryDistanceNow = norm(pI - cfg.telemetryStationPosition_m(:));
    telemetryDistanceHistory(k) = telemetryDistanceNow;
    maxTelemetryDistance_m = max(maxTelemetryDistance_m, telemetryDistanceNow);

    % Only an authorized airborne pursuit is subject to these operational limits.
    % The inner 1 km circle is NOT a hard stop; communication and return/delivery
    % energy are the hard limits in this revised model.
    if launchAuthorized && ~gateOnlyScenario
        if linkRangeExceeded(telemetryDistanceNow, cfg.maxOperationalLinkRange_m)
            telemetryLimitTriggered = true;
            operationalLimitReason = 'radio/telemetry range limit already exceeded';
            abortFlag = true; abortTime = t; abortReason = operationalLimitReason;
            phase = 'ABORTED_TELEMETRY_RANGE'; phaseHistory(k) = string(phase);
            outcome = ['PURSUIT TERMINATED: ' operationalLimitReason];
            releaseTowDecision = 'Pursuit stopped at communication limit; recovery manoeuvre is not simulated';
            lastIndex = k;
            break;
        end
        [energyGuardTriggered, estimatedFutureEnergyWh, projectedMissionEnergyWithReserveWh, ...
            missionEnergyCeilingWh, requiredPhaseBufferWh] = evaluateEnergyReserve( ...
            pI, energy(k), massCurrent, phase, towGoal, energyBudget, s.isGuidanceDiagnostic, cfg);
        estimatedFutureEnergyHistory(k) = estimatedFutureEnergyWh;
        projectedMissionEnergyHistory(k) = projectedMissionEnergyWithReserveWh;
        missionEnergyCeilingHistory(k) = missionEnergyCeilingWh;
        if energyGuardTriggered
            wasTowPhase = strcmp(phase,'TOW');
            energyReserveLimitTriggered = true;
            operationalLimitReason = sprintf(['energy reserve guard: used %.1f Wh + future-segment estimate %.1f Wh ' ...
                '+ phase buffer %.1f Wh exceeds %.1f Wh mission ceiling'], ...
                energy(k), estimatedFutureEnergyWh, requiredPhaseBufferWh, missionEnergyCeilingWh);
            abortFlag = true; abortTime = t; abortReason = operationalLimitReason;
            phase = 'ABORTED_ENERGY_RESERVE'; phaseHistory(k) = string(phase);
            outcome = ['PURSUIT TERMINATED: ' operationalLimitReason];
            if wasTowPhase
                towDuration = max(0,t - towStartTime);
                towCompleted = false;
                releaseTowDecision = 'Tow terminated by energy-reserve guard; delivery/landing not simulated';
            else
                releaseTowDecision = 'Pursuit terminated to preserve estimated return/recovery energy';
            end
            lastIndex = k;
            break;
        end
    end

    % Record a target crossing of the 1,000 m circle as a reference event only.
    % It is not a pursuit stop: the interceptor may continue outside the circle
    % while radio/telemetry range and energy-reserve checks remain satisfied.
    if ~gateOnlyScenario && launchAuthorized && ~s.isGuidanceDiagnostic && ...
            ~captured && targetEnteredInnerReferenceCircle && ~targetExitedInnerZoneBeforeCandidate && k > 1
        pPrevTarget = pTh(:,k-1);
        pCurrTarget = pT;
        rPrevSite = norm(pPrevTarget - s.keepOutCenter(:));
        rCurrSite = norm(pCurrTarget - s.keepOutCenter(:));
        if rPrevSite <= s.keepOutRadius + 1e-8 && rCurrSite > s.keepOutRadius + 1e-8
            dp = pCurrTarget - pPrevTarget;
            q0 = pPrevTarget - s.keepOutCenter(:);
            qa = dot(dp,dp); qb = 2*dot(q0,dp); qc = dot(q0,q0)-s.keepOutRadius^2;
            disc = qb^2 - 4*qa*qc;
            alpha = 1.0;
            if qa > 1e-12 && disc >= 0
                rootsAlpha = [(-qb-sqrt(disc))/(2*qa), (-qb+sqrt(disc))/(2*qa)];
                rootsAlpha = rootsAlpha(rootsAlpha >= 0 & rootsAlpha <= 1);
                if ~isempty(rootsAlpha), alpha = max(rootsAlpha); end
            end
            innerZoneExitTime_s = th(k-1) + alpha*cfg.dt;
            innerZoneExitPosition = pPrevTarget + alpha*dp;
            targetExitedInnerZoneBeforeCandidate = true;
            fprintf('\n%s\n  Target crossed the %.0f m reference circle at %.2f s; pursuit continues under link/energy guards.\n', ...
                s.name, s.keepOutRadius, innerZoneExitTime_s);
        end
    end

    % The two-boundary logic uses the 2,000 m ring for track acquisition and the
    % 1,000 m circle as a reference boundary. Launch is recommended before the
    % predicted entry if the response-time estimate is met. The integrated failure
    % case continues after launch to demonstrate a target that exits the zone without
    % a candidate; that is a diagnostic case, not an operational policy recommendation.
    if integratedLaunchGateScenario && ~launchAuthorized && ~abortFlag
        dSite = norm(pT - s.keepOutCenter(:));
        if ~isfinite(trackAcquisitionTime) && dSite <= cfg.outerTrackingRadius_m
            trackAcquisitionTime = t;
            [timeToInner, entersInner] = timeToCircleEntry(pT, vT, s.keepOutCenter(:), cfg.innerBoundaryRadius_m);
            predictedTimeToReferenceCircleAtTrack_s = timeToInner;
            phaseHistory(k) = "OUTER_TRACKING";
            if ~entersInner
                launchGateDecision = 'NO TAKEOFF: predicted target track misses 1,000 m engagement zone';
                outcome = launchGateDecision;
                phase = 'TRACKING_NO_LAUNCH';
                lastIndex = k;
                break;
            elseif timeToInner <= cfg.gateResponseTimeTotal_s
                abortFlag = true; abortTime = t; phase = 'ABORTED_BEFORE_TAKEOFF';
                abortReason = sprintf('only %.1f s to engagement-zone entry; %.1f s response estimate required', ...
                    timeToInner, cfg.gateResponseTimeTotal_s);
                launchGateDecision = ['NO TAKEOFF: ' abortReason];
                outcome = ['NO TAKEOFF: ' abortReason];
                phaseHistory(k) = string(phase); lastIndex = k; break;
            else
                launchAuthorized = true;
                reactionStartTime = t;
                phase = 'REACTION';
                launchGateDecision = 'SIMULATED LAUNCH AUTHORIZED: radar-style track predicts inner-zone entry with sufficient lead time';
                outcome = 'Launch authorized by illustrative two-boundary prediction; flight attempt underway';
                fprintf('\n%s\n  Outer-ring track acquired at %.2f s; predicted inner-zone entry in %.2f s; launch authorized in model.\n', ...
                    s.name, t, timeToInner);
            end
        end
        if ~launchAuthorized
            % Do not move the interceptor before launch. Target track advances.
            pT = pT + vT * cfg.dt;
            if k < N, energy(k+1) = energy(k); end
            continue;
        end
    end

    if gateOnlyScenario && ...
            ~launchAuthorized && ~abortFlag
        dSite = norm(pT - s.keepOutCenter);
        if ~isfinite(trackAcquisitionTime) && dSite <= cfg.outerTrackingRadius_m
            trackAcquisitionTime = t;
            [timeToInner, entersInner] = timeToCircleEntry(pT, vT, s.keepOutCenter, s.keepOutRadius);
            predictedTimeToReferenceCircleAtTrack_s = timeToInner;
            phaseHistory(k) = "OUTER_TRACKING";
            % Target mass alone does not veto launch. The launch gate depends on
            % the predicted path and remaining time. If an overweight target later
            % reaches a geometric capture-candidate event, the separate policy
            % below flags immediate simulated parachute request and forbids towing.
            if ~entersInner
                phase = 'TRACKING_NO_LAUNCH';
                launchGateDecision = 'NO TAKEOFF: predicted track does not enter the 1,000 m engagement zone';
                outcome = launchGateDecision;
            else
                requiredTime = cfg.gateResponseTimeTotal_s;
                if timeToInner <= requiredTime
                    phase = 'ABORTED';
                    abortFlag = true;
                    abortTime = t;
                    abortReason = sprintf('insufficient time before engagement-zone entry (%.1f s remaining; %.1f s response estimate required)', ...
                        timeToInner, requiredTime);
                    launchGateDecision = ['NO TAKEOFF: ' abortReason];
                    outcome = ['ABORTED BEFORE TAKEOFF: ' abortReason];
                else
                    launchAuthorized = true;
                    launchGateDecision = 'SIMULATED LAUNCH AUTHORIZED: predicted engagement-zone entry with sufficient time';
                    phase = 'TRACKING_GATE_APPROVED';
                    outcome = 'Launch gate approved (gate demonstration only; no flight command issued)';
                end
            end
            phaseHistory(k) = string(phase);
        end
        if ~launchAuthorized && ~abortFlag
            % Track only; no interceptor motion. Stop once a tracked non-threat
            % exits the outer ring, while preserving its track history.
            if isfinite(trackAcquisitionTime) && t > trackAcquisitionTime && dSite > cfg.outerTrackingRadius_m
                lastIndex = k;
                break;
            end
            pT = pT + vT * cfg.dt;
            if k < N, energy(k+1) = energy(k); end
            continue;
        elseif launchAuthorized
            % The offline gate illustration ends at its decision; do not accidentally
            % run the main intercept law on this special demonstration track.
            lastIndex = k;
            break;
        end
    end

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
    % Capture candidates may occur on either side of the 1 km reference circle.
    % A candidate is only eligible while the link guard and energy-reserve guard pass.
    zoneCondition = true;
    captureGeometryOK = ~captured && launchAuthorized && ...
        isfinite(reactionStartTime) && t >= reactionStartTime + reactionDelay && ...
        R <= cfg.captureRange(2) && R >= cfg.captureRange(1) && zoneCondition;
    if captureGeometryOK && strcmp(mode,'Defense (target faster/equal)') && ...
            ~isempty(s.leadPoint)
        leadPointSettled = norm(s.leadPoint - pI) <= cfg.waypointTolerance && ...
            norm(vI) <= cfg.waypointSpeedTol;
        captureGeometryOK = captureGeometryOK && leadPointSettled;
    end
    if captureGeometryOK
        geometricCandidateWindowReached = true;
        candidatePointPreview = 0.5 * (pI + pT);
        geometricCandidatePoint = candidatePointPreview;
        previewSafeDistances = sqrt(sum((s.safePoints - candidatePointPreview).^2, 1));
        [~, previewSafeIndex] = min(previewSafeDistances);
        previewNeedsTow = ~any(previewSafeDistances <= cfg.safeAreaRadius) && ...
            targetMassEstimate_kg <= cfg.targetMassLimit_kg;
        if previewNeedsTow
            previewPostCaptureMass_kg = cfg.massWithCaptured + ...
                max(0, targetMassEstimate_kg - cfg.targetMassLimit_kg);
            [towEnergyGuardTriggered, towFutureEnergyWh, towProjectedEnergyWh, ...
                towMissionCeilingWh, towPhaseBufferWh] = evaluateEnergyReserve( ...
                candidatePointPreview, energy(k), previewPostCaptureMass_kg, 'TOW', ...
                s.safePoints(:,previewSafeIndex), cfg.energyBudgetWithTow, false, cfg);
            if towEnergyGuardTriggered
                energyReserveLimitTriggered = true;
                estimatedFutureEnergyWh = towFutureEnergyWh;
                projectedMissionEnergyWithReserveWh = towProjectedEnergyWh;
                missionEnergyCeilingWh = towMissionCeilingWh;
                requiredPhaseBufferWh = towPhaseBufferWh;
                estimatedFutureEnergyHistory(k) = towFutureEnergyWh;
                projectedMissionEnergyHistory(k) = towProjectedEnergyWh;
                missionEnergyCeilingHistory(k) = towMissionCeilingWh;
                energyBudget = cfg.energyBudgetWithTow;
                operationalLimitReason = sprintf(['projected mission energy %.2f Wh = %.1f Wh already used + ' ...
                    '%.1f Wh estimated remaining tow segment + %.1f Wh phase buffer; exceeds %.1f Wh tow mission ceiling'], ...
                    towProjectedEnergyWh, energy(k), towFutureEnergyWh, towPhaseBufferWh, towMissionCeilingWh);
                abortFlag = true; abortTime = t; abortReason = operationalLimitReason;
                phase = 'ABORTED_ENERGY_RESERVE'; phaseHistory(k) = string(phase);
                captureCandidateRejectedByEnergyGuard = true;
                outcome = 'Energy guard stop: geometric window reached; candidate not registered';
                releaseTowDecision = ['Geometric candidate window reached, but energy guard rejected the tow requirement; ' ...
                    'candidate not registered'];
                fprintf('\n%s\n  ENERGY GUARD STOP: geometric candidate window reached at %.2f m; candidate not registered.\n  %s\n', ...
                    s.name, R, operationalLimitReason);
                lastIndex = k;
                break;
            end
            % Keep the latest projected tow-energy check for the summary/export.
            estimatedFutureEnergyWh = towFutureEnergyWh;
            projectedMissionEnergyWithReserveWh = towProjectedEnergyWh;
            missionEnergyCeilingWh = towMissionCeilingWh;
            requiredPhaseBufferWh = towPhaseBufferWh;
        end
        captured = true;
        captureTime = t;
        captureSampleIndex = k;
        captureRangeAtEvent = R;
        capturePoint = candidatePointPreview;
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
        candidateRawCmd = [];
        if strcmp(mode,'Defense (target faster/equal)') && ~isempty(s.leadPoint)
            if norm(s.leadPoint-pI) <= cfg.waypointTolerance && norm(vI) <= cfg.waypointSpeedTol
                candidateCmd = -2.0*vI;
            else
                candidateCmd = waypointController(pI,vI,s.leadPoint,cfg.maxSpeed,cfg);
            end
            candidatePN = [0;0];
            candidatePNActive = false;
        else
            [candidateCmd,candidatePN,candidatePNActive,candidateRawCmd] = interceptGuidance(pI,vI,pT,vT,cfg);
        end
        if ~exist('candidateRawCmd','var') || isempty(candidateRawCmd)
            candidateRawCmd = candidateCmd;
        end
        captureState.commandedAccel = candidateRawCmd;
        captureState.controllerCommandedAccel = limitVector(candidateCmd,cfg.accelLimit);
        captureState.appliedAccel = [nan;nan]; % no control interval is integrated at the event boundary
        captureState.activeLimit = 'Capture-event boundary; no integration on this sample';
        captureState.pnAcceleration = candidatePN;
        captureState.pnActive = candidatePNActive;
        requestedAh(:,k) = candidateRawCmd;
        controllerCommandAh(:,k) = limitVector(candidateCmd,cfg.accelLimit);
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
        % The revised 23.0 kg baseline is 18.0 kg nominal plus the 5.0 kg target.
        % The net/tether is already included in nominal mass and is not counted twice.
        % For an overweight target, record the appropriate combined mass but
        % do not integrate loaded flight: the policy immediately requests
        % simulated parachute recovery and explicitly forbids towing.
        postCaptureMassEstimate_kg = cfg.massWithCaptured + ...
            max(0, targetMassEstimate_kg - cfg.targetMassLimit_kg);
        massCurrent = postCaptureMassEstimate_kg;

        if targetMassEstimate_kg > cfg.targetMassLimit_kg
            parachuteRequestFlag = true;
            towDisabledForOverweight = true;
            phase = 'OVERWEIGHT_PARACHUTE_REQUEST';
            phaseHistory(k) = string(phase);
            releaseTowDecision = sprintf(['Overweight target %.1f kg exceeds %.1f kg limit: ' ...
                'immediate simulated parachute-request; towing disabled'], ...
                targetMassEstimate_kg, cfg.targetMassLimit_kg);
            outcome = ['Geometric capture candidate; immediate simulated parachute-request; no tow'];
            if ~s.isGuidanceDiagnostic, energyBudget = cfg.energyBudgetNoTow; end
            fprintf('  Overweight policy: target estimate %.1f kg > %.1f kg. Immediate simulated parachute-request; no tow.\n', ...
                targetMassEstimate_kg, cfg.targetMassLimit_kg);
            fprintf('  Estimated combined mass for reporting: %.2f kg; post-candidate flight/recovery is not simulated.\n', ...
                postCaptureMassEstimate_kg);
            lastIndex = k;
            break;
        elseif captureInsideSafeArea
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

    % Record that the target has entered the 1,000 m reference circle. This event
    % is diagnostic only; candidate geometry may be accepted on either side of it.
    if launchAuthorized && ~gateOnlyScenario && ~s.isGuidanceDiagnostic && ...
            norm(pT - s.keepOutCenter(:)) <= s.keepOutRadius + 1e-8
        targetEnteredInnerReferenceCircle = true;
    end

    % In-flight time-to-boundary aborts are not applied here. A launch-authorized
    % pursuit can cross the 1 km reference circle; the telemetry-range guard and
    % energy-reserve guard above now determine when continued pursuit is disallowed.

    % State transition / guidance. A separate raw-demand vector makes it
    % possible to report controller saturation without confusing demand with
    % the command actually passed to the point-mass integrator.
    aRawCmd = [];
    if strcmp(phase,'TOW')
        dGoal = norm(towGoal - pI);
        % Mission-level tow completion means entering the designated safe-area
        % radius, not reaching the exact centre or reaching zero speed. The
        % simulation stops at the area boundary and does not model release,
        % landing, or a stationary hold. Check arrival before timeout so entry
        % exactly at the 60 s boundary counts as completed.
        if dGoal <= cfg.safeAreaRadius
            towDuration = t - towStartTime;
            towCompleted = true;
            phase = 'SAFE AREA REACHED';
            releaseTowDecision = 'Tow completed: entered designated safe area; release/landing not modelled';
            outcome = 'Geometric capture candidate; designated safe area entered within 60 s';
            phaseHistory(k) = string(phase);
            fprintf('  Tow arrival: entered safe-area radius %.1f m after %.2f s; release/landing not modelled.\n', ...
                cfg.safeAreaRadius, towDuration);
            lastIndex = k;
            break;
        end
        if (t - towStartTime) >= cfg.maxTowDuration
            towDuration = t - towStartTime;
            towCompleted = false;
            outcome = 'Tow failed: designated safe area not entered within 60 s';
            fprintf('  Tow failed: safe-area radius not entered within %.1f s; final centre distance %.2f m (required <= %.2f m).\n', ...
                cfg.maxTowDuration, dGoal, cfg.safeAreaRadius);
            lastIndex = k;
            break;
        end
        aCmd = waypointController(pI, vI, towGoal, cfg.maxTowSpeed, cfg);
        aLimitNow = cfg.accelLimit;
        % cfg.maxTowSpeed is a desired towing cruise speed, not a hard clip.
        % The global speed bound is retained while acceleration-limited braking
        % ramps the interceptor down to the tow setpoint.
        speedLimitNow = cfg.maxSpeed;
    elseif strcmp(phase,'REACTION') && isfinite(reactionStartTime) && ...
            (t - reactionStartTime) < reactionDelay
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
        [aCmd, aPNNow, pnActiveNow, aRawCmd] = interceptGuidance(pI, vI, pT, vT, cfg);
        pnAh(:,k) = aPNNow;
        pnActiveHistory(k) = pnActiveNow;
        aLimitNow = cfg.accelLimit;
        speedLimitNow = cfg.maxSpeed;
    end

    % Preserve the unbounded demand for diagnostics, but enforce the assumed
    % acceleration cap at the controller interface before integrating. The
    % model-step limiter remains as a second safety guard and applies the speed
    % envelope. This separates raw demand, accepted command and applied response.
    if isempty(aRawCmd)
        aRawCmd = aCmd;
    end
    requestedAh(:,k) = aRawCmd;
    controllerCommand = limitVector(aCmd,aLimitNow);
    controllerCommandAh(:,k) = controllerCommand;
    rawCommandExceeded = norm(aRawCmd) > aLimitNow + 1e-10;
    vIBefore = vI;
    [vINext, aApplied, activeLimitNow] = limitedVelocityStep( ...
        vIBefore, controllerCommand, aLimitNow, speedLimitNow, cfg.dt);
    predictedNextPosition = pI + vINext * cfg.dt;
    predictedNextLinkDistance = norm(predictedNextPosition - cfg.telemetryStationPosition_m(:));
    if launchAuthorized && ~gateOnlyScenario && linkRangeExceeded(predictedNextLinkDistance, cfg.maxOperationalLinkRange_m)
        telemetryLimitTriggered = true;
        operationalLimitReason = sprintf('predicted next step would exceed %.0f m radio/telemetry range', ...
            cfg.maxOperationalLinkRange_m);
        abortFlag = true; abortTime = t; abortReason = operationalLimitReason;
        phase = 'ABORTED_TELEMETRY_RANGE'; phaseHistory(k) = string(phase);
        activeLimitHistory(k) = "radio/telemetry range guard; step not integrated";
        ah(:,k) = [0;0];
        outcome = ['PURSUIT TERMINATED: ' operationalLimitReason];
        releaseTowDecision = 'Pursuit stopped before communication-link range was exceeded; recovery not simulated';
        lastIndex = k;
        break;
    end
    vI = vINext;
    if rawCommandExceeded
        if strcmp(activeLimitNow,'none')
            activeLimitNow = 'controller command saturation';
        else
            activeLimitNow = [activeLimitNow ' + controller command saturation'];
        end
    end
    activeLimitHistory(k) = string(activeLimitNow);
    ah(:,k) = aApplied;
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
    flightPowerActive = launchAuthorized && isfinite(reactionStartTime) && ...
        (t - reactionStartTime) >= reactionDelay && ~abortFlag;
    if flightPowerActive
        power(k) = approximatePower(norm(vI), massCurrent, cfg);
        energy(k) = energy(k) + power(k) * cfg.dt / 3600;
    else
        % Prelaunch tracking and the reaction/spin-up interval are not treated
        % as propulsion flight energy. Avionics/launch buffers remain represented
        % separately in the provisional mission-energy estimate.
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
controllerCommandAh = controllerCommandAh(:,1:used);
activeLimitHistory = activeLimitHistory(1:used);
pnAh = pnAh(:,1:used);
pnActiveHistory = pnActiveHistory(1:used);
rh = rh(1:used);
energy = energy(1:used);
power = power(1:used);
phaseHistory = phaseHistory(1:used);
telemetryDistanceHistory = telemetryDistanceHistory(1:used);
estimatedFutureEnergyHistory = estimatedFutureEnergyHistory(1:used);
projectedMissionEnergyHistory = projectedMissionEnergyHistory(1:used);
missionEnergyCeilingHistory = missionEnergyCeilingHistory(1:used);

% Include finite recorded ranges only. Post-capture samples are NaN by design
% because interceptor-to-captured-load separation is not an engagement metric.
finiteRanges = rh(isfinite(rh));
if isempty(finiteRanges)
    minimumRange = nan;
else
    minimumRange = min(minimumRange, min(finiteRanges));
end

% If a capture occurred and tow is still underway at simulation end, report it.
if strcmp(phase,'TOW') && ~towCompleted
    towDuration = min(th(end) - towStartTime, cfg.maxTowDuration);
    outcome = 'Tow incomplete: designated safe area not reached; no complete-mission energy estimate';
end
if ~isfinite(captureTime)
    releaseTowDecision = 'Not applicable: no capture candidate registered';
end
if gateOnlyScenario && ~launchAuthorized && ...
        ~contains(outcome,'ABORTED') && ~isfinite(trackAcquisitionTime)
    launchGateDecision = 'NO TAKEOFF: target never entered the outer tracking ring';
    outcome = launchGateDecision;
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
out.commandedAcceleration = requestedAh; % raw guidance demand before command saturation
out.controllerCommandAcceleration = controllerCommandAh; % bounded command supplied to model
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
out.telemetryDistance_m = telemetryDistanceHistory(:);
out.estimatedFutureEnergyWhHistory = estimatedFutureEnergyHistory(:);
out.projectedMissionEnergyWhHistory = projectedMissionEnergyHistory(:);
out.missionEnergyCeilingWhHistory = missionEnergyCeilingHistory(:);
out.initialRange = norm(s.pT0(:) - s.pI0(:));
out.targetSpeed = norm(s.vT(:));
out.targetVelocity = s.vT(:);
out.captureTime = captureTime;
out.captureSampleIndex = captureSampleIndex;
out.captureState = captureState;
out.capturePoint = capturePoint;
out.captureRangeAtEvent = captureRangeAtEvent;
out.geometricCandidateWindowReached = logical(geometricCandidateWindowReached);
out.captureCandidateRejectedByEnergyGuard = logical(captureCandidateRejectedByEnergyGuard);
out.geometricCandidatePoint = geometricCandidatePoint;
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
out.launchAuthorized = launchAuthorized;
out.trackAcquisitionTime = trackAcquisitionTime;
out.predictedTimeToReferenceCircleAtTrack_s = predictedTimeToReferenceCircleAtTrack_s;
out.targetMassEstimate_kg = targetMassEstimate_kg;
% Export all state flags/values consumed by plotting, summaries and trajectory CSV.
% These were previously maintained as local variables but not copied into OUT,
% causing an "Unrecognized field name" error during plot/export.
out.targetMassLimit_kg = cfg.targetMassLimit_kg;
out.parachuteRequestFlag = logical(parachuteRequestFlag);
out.towDisabledForOverweight = logical(towDisabledForOverweight);
out.postCaptureMassEstimate_kg = postCaptureMassEstimate_kg;
out.scenarioID = NaN; % overwritten by the caller after each scenario returns
out.launchGateDecision = launchGateDecision;
out.isGateOnlyScenario = gateOnlyScenario;
out.integratedLaunchGateScenario = integratedLaunchGateScenario;
out.extendedPursuitScenario = extendedPursuitScenario;
out.targetEnteredInnerReferenceCircle = targetEnteredInnerReferenceCircle;
out.targetExitedInnerZoneBeforeCandidate = targetExitedInnerZoneBeforeCandidate;
out.innerZoneExitTime_s = innerZoneExitTime_s;
out.innerZoneExitPosition = innerZoneExitPosition;
out.radioRangeLimit_m = cfg.radioRangeMax_m;
out.telemetryRangeLimit_m = cfg.telemetryRangeMax_m;
out.effectiveLinkRangeLimit_m = cfg.maxOperationalLinkRange_m;
out.telemetryOriginPosition_m = cfg.telemetryStationPosition_m(:);
out.maxTelemetryDistance_m = maxTelemetryDistance_m;
out.telemetryLimitTriggered = logical(telemetryLimitTriggered);
out.energyReserveLimitTriggered = logical(energyReserveLimitTriggered);
out.operationalLimitReason = operationalLimitReason;
out.estimatedFutureEnergyWh = estimatedFutureEnergyWh;
out.projectedMissionEnergyWithReserveWh = projectedMissionEnergyWithReserveWh;
out.missionEnergyCeilingWh = missionEnergyCeilingWh;
out.requiredPhaseBufferWh = requiredPhaseBufferWh;
out.captureOutsideInnerBoundary = isfinite(captureTime) && ...
    norm(captureState.pT - s.keepOutCenter(:)) > s.keepOutRadius + 1e-8;
end

%% ------------------------------------------------------------------------
function [aCmd, aPN, pnActive, aRawCmd] = interceptGuidance(pI, vI, pT, vT, cfg)
% Point-mass guidance with a provisional response-time velocity servo and a
% terminal PN blend. RAWCMD is logged for diagnosis; ACMD is magnitude-limited
% before it is passed to the point-mass integrator. This is not a flight-control
% law validated against the physical aircraft.
aPN = [0;0];
pnActive = false;
r = pT - pI;
R = max(norm(r), 1e-6);
aimPoint = constantSpeedInterceptPoint(pI, pT, vT, cfg.maxSpeed);
d = aimPoint - pI;
dNorm = max(norm(d), 1e-6);
uAim = d / dNorm;

% Braking profile provides a desired velocity that reduces as the aim point
% approaches. Replace the response-time assumption with measured dynamics when
% such data become available; do not interpret it as a measured aircraft value.
vDesMagnitude = min(cfg.maxSpeed, sqrt(2 * cfg.accelLimit * dNorm));
vDesired = vDesMagnitude * uAim;
velocityError = vDesired - vI;
aVelocityRequest = velocityError / max(cfg.interceptVelocityResponseTime_s,1e-3);
aRawCmd = aVelocityRequest;

if R <= cfg.PNswitchRange && norm(vI) > 0.5
    pnActive = true;
    vRelative = vT - vI;
    closingSpeed = -dot(r, vRelative) / R;
    lambdaDot = (r(1)*vRelative(2) - r(2)*vRelative(1)) / (R^2);
    uVelocity = vI / norm(vI);
    normalToVelocity = [-uVelocity(2); uVelocity(1)];
    aPN = cfg.PNgain * max(closingSpeed, 0) * lambdaDot * normalToVelocity;
    aRawCmd = 0.25 * aVelocityRequest + 0.75 * aPN;
end

% Controller-interface saturation. The raw request remains available in the log,
% while the command sent to the motion model respects the assumed acceleration
% limit. The independent model-step limiter remains in place as a second guard.
aCmd = limitVector(aRawCmd,cfg.accelLimit);
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
% Rate-limited waypoint controller with a braking-speed profile. The tow cruise
% speed is a setpoint; actual velocity transitions smoothly under the common
% acceleration limiter applied by limitedVelocityStep().
e = waypoint - p;
d = norm(e);
if d <= cfg.waypointTolerance
    vDesired = [0;0];
else
    remaining = max(d - cfg.towArrivalTolerance, 0);
    vDesiredMagnitude = min(speedLimit, sqrt(2 * cfg.accelLimit * remaining));
    vDesired = (vDesiredMagnitude / max(d,1e-9)) * e;
end
Kvel = 1.8; % provisional velocity-servo gain; this is a point-mass controller
aCmd = Kvel * (vDesired - v);
end

%% ------------------------------------------------------------------------
function [guardTriggered, futureEnergyWh, projectedEnergyWh, missionCeilingWh, phaseBufferWh] = ...
    evaluateEnergyReserve(pI, energyUsedWh, massCurrent, phase, towGoal, energyBudget, isDiagnostic, cfg)
% Predict the minimum modeled energy to finish the active phase and preserve the
% phase buffer. This is an estimate, not a battery-state estimator or flight test.
if strcmp(phase,'TOW') && all(isfinite(towGoal))
    % The modeled tow ends at the safe-area boundary; loaded descent/avionics are
    % represented by the tow buffer. The later landing/recovery path is not simulated.
    futureDistance_m = norm(towGoal(:) - pI(:));
    reserveSpeed_mps = cfg.maxTowSpeed;
    phaseBufferWh = cfg.energyBufferTow_Wh;
    fallbackBudgetWh = cfg.energyBudgetWithTow;
else
    % For normal pursuit, retain energy for a direct return to the fixed launch pad.
    futureDistance_m = norm(cfg.interceptorStationPosition_m(:) - pI(:));
    reserveSpeed_mps = cfg.returnReserveCruiseSpeed_mps;
    phaseBufferWh = cfg.energyBufferNoTow_Wh;
    fallbackBudgetWh = cfg.energyBudgetNoTow;
end
reserveSpeed_mps = max(reserveSpeed_mps,1.0);
futureTime_s = futureDistance_m / reserveSpeed_mps + cfg.returnReserveTimeMargin_s;
futurePowerW = approximatePower(reserveSpeed_mps, massCurrent, cfg);
futureEnergyWh = futurePowerW * futureTime_s / 3600;
if isfinite(energyBudget) && energyBudget > 0
    sizingCeilingWh = energyBudget;
else
    sizingCeilingWh = fallbackBudgetWh;
end
missionCeilingWh = min(sizingCeilingWh, cfg.batteryUsableEnergyWh);
projectedEnergyWh = max(energyUsedWh,0) + futureEnergyWh + phaseBufferWh;
guardTriggered = projectedEnergyWh > missionCeilingWh + 1e-9;
if isDiagnostic
    % Diagnostic runs use the usable battery as the notional ceiling because they
    % are in-flight controller checks rather than complete mission-energy cases.
    missionCeilingWh = cfg.batteryUsableEnergyWh;
    projectedEnergyWh = max(energyUsedWh,0) + futureEnergyWh + phaseBufferWh;
    guardTriggered = projectedEnergyWh > missionCeilingWh + 1e-9;
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
    % Legacy advisory helper only. Operational flight cases now use radio/telemetry
    % and energy-reserve guards; this helper is not called by the flight loop.
    abortNow = false;
    tBoundary = inf;
    reason = 'target inside reference circle; no geometric abort required';
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
function [vNext, aApplied, activeLimit] = limitedVelocityStep(v, aCmd, aLimit, speedLimit, dt)
% Applies acceleration limiting first, then scales that same acceleration
% increment if needed to avoid exceeding the hard speed envelope. No direct
% velocity assignment is used, so the resulting finite-difference acceleration
% remains bounded by aLimit (up to floating-point tolerance).
active = strings(0,1);
if norm(aCmd) > aLimit + 1e-10
    active(end+1) = "acceleration limit"; %#ok<AGROW>
end
aApplied = limitVector(aCmd, aLimit);
vCandidate = v + aApplied * dt;
if norm(vCandidate) > speedLimit + 1e-9
    lo = 0; hi = 1;
    for j = 1:45
        mid = 0.5 * (lo + hi);
        testV = v + mid * aApplied * dt;
        if norm(testV) <= speedLimit
            lo = mid;
        else
            hi = mid;
        end
    end
    aApplied = lo * aApplied;
    vCandidate = v + aApplied * dt;
    active(end+1) = "speed-envelope governor"; %#ok<AGROW>
end
vNext = vCandidate;
if isempty(active)
    activeLimit = 'none';
else
    activeLimit = strjoin(cellstr(active), ' + ');
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
xl = out.telemetryOriginPosition_m(1) + out.effectiveLinkRangeLimit_m*cos(ang);
yl = out.telemetryOriginPosition_m(2) + out.effectiveLinkRangeLimit_m*sin(ang);
hLink = plot(ax1,xl,yl,'m-.','LineWidth',1.0);
hInterceptor = plot(ax1,out.pI(1,:),out.pI(2,:),'b-','LineWidth',1.7);
hTarget = plot(ax1,out.pT(1,:),out.pT(2,:),'Color',[0.85 0.35 0.1],'LineStyle','-','LineWidth',1.7);
hIstart = plot(ax1,out.pI(1,1),out.pI(2,1),'bo','MarkerFaceColor','b');
hTstart = plot(ax1,out.pT(1,1),out.pT(2,1),'o','Color',[0.85 0.35 0.1],'MarkerFaceColor',[0.85 0.35 0.1]);
hSafe = plot(ax1,out.safePoints(1,:),out.safePoints(2,:),'g^','MarkerFaceColor','g','MarkerSize',8);

% Explicit circles show the same geometric test used by simulateOneScenario.
% A capture candidate inside a designated safe area gets immediate release; otherwise the
% simulation records a tow until the designated safe-area radius is entered.
hSafeArea = gobjects(1,1);
for q = 1:size(out.safePoints,2)
    xs = out.safePoints(1,q) + out.safeAreaRadius*cos(ang);
    ys = out.safePoints(2,q) + out.safeAreaRadius*sin(ang);
    hTmp = plot(ax1,xs,ys,'g:','LineWidth',1.1);
    if q == 1
        hSafeArea = hTmp;
    end
end
hLegend = [hKeep, hLink, hInterceptor, hTarget, hIstart, hTstart, hSafe, hSafeArea];
legendText = {'1,000 m reference circle','Effective radio/telemetry envelope', ...
    'Interceptor','Target / captured load','Interceptor start','Target start', ...
    'Safe-point centres','Designated safe-area boundary'};
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
if out.captureCandidateRejectedByEnergyGuard && all(isfinite(out.geometricCandidatePoint))
    hRejected = plot(ax1,out.geometricCandidatePoint(1),out.geometricCandidatePoint(2), ...
        'mx','MarkerSize',12,'LineWidth',2.0);
    hLegend(end+1) = hRejected;
    legendText{end+1} = 'Geometric window reached; rejected by energy guard';
end
if isfield(out,'targetExitedInnerZoneBeforeCandidate') && ...
        out.targetExitedInnerZoneBeforeCandidate && all(isfinite(out.innerZoneExitPosition))
    hExit = plot(ax1,out.innerZoneExitPosition(1),out.innerZoneExitPosition(2),'rx', ...
        'MarkerSize',10,'LineWidth',1.8);
    hLegend(end+1) = hExit;
    legendText{end+1} = 'Target crossed 1,000 m reference circle';
end
xlabel(ax1,'x (m)'); ylabel(ax1,'y (m)');
if out.isGateOnlyScenario
    rangeTitleNote = 'gate-only track; interceptor working-range check not applicable';
elseif out.isGuidanceDiagnostic
    rangeTitleNote = 'diagnostic; nominal mission-range check not applied';
elseif out.initialRange >= cfg.workingRangeMin && out.initialRange <= cfg.workingRangeMax
    rangeTitleNote = sprintf('initial range %.0f m: within 1.5-2.0 km band',out.initialRange);
else
    rangeTitleNote = sprintf('initial range %.0f m: outside 1.5-2.0 km band',out.initialRange);
end
title(ax1,sprintf('2-D trajectories (%.0f m reference-circle radius / %.0f m safe-area radius)\n%s', ...
    out.keepOutRadius, cfg.safeAreaRadius,rangeTitleNote),'Color','k','FontSize',9,'Interpreter','none');
lg = legend(ax1,hLegend,legendText,'Location','bestoutside');
set(lg,'Color','w','TextColor','k','EdgeColor',[0.65 0.65 0.65]);
styleAxes(ax1);

% Show the geometric release/tow decision explicitly on the trajectory panel.
if isfinite(out.captureTime)
    hasParachuteRequest = isfield(out,'parachuteRequestFlag') && ...
        ~isempty(out.parachuteRequestFlag) && isscalar(out.parachuteRequestFlag) && ...
        logical(out.parachuteRequestFlag);
    if hasParachuteRequest
        safeMsg = sprintf('Overweight target %.1f kg: immediate simulated parachute-request; NO TOW', ...
            out.targetMassEstimate_kg);
    elseif out.captureInsideSafeArea
        safeMsg = sprintf('Capture candidate inside safe area (nearest centre %.1f m)', ...
            out.nearestSafePointDistanceAtCapture);
    else
        safeMsg = sprintf('Capture candidate outside safe areas (nearest centre %.1f m): TOW', ...
            out.nearestSafePointDistanceAtCapture);
    end
    text(ax1,0.02,0.025,safeMsg,'Units','normalized','FontSize',7.5, ...
        'Color',[0.08 0.08 0.08],'BackgroundColor','w','EdgeColor',[0.55 0.55 0.55], ...
        'Interpreter','none','Margin',5);
    closingNote = sprintf(['Relative closing speed at candidate: %.2f m/s (kinematic only).\n' ...
        'Net deployment, tether loads and physical capture success are NOT modelled.'],out.closingAtCapture);
    text(ax1,0.02,0.19,closingNote,'Units','normalized','FontSize',7.0, ...
        'Color',[0.25 0.08 0.08],'BackgroundColor',[1.0 0.97 0.92], ...
        'EdgeColor',[0.75 0.40 0.25],'Interpreter','none','Margin',4);
elseif out.captureCandidateRejectedByEnergyGuard && out.geometricCandidateWindowReached
    limitMsg = sprintf(['ENERGY GUARD STOP\n' ...
        'Geometric window reached: %.2f m separation\n' ...
        'Candidate not registered: %.1f Wh projected > %.0f Wh ceiling'], ...
        out.minimumRange, out.projectedMissionEnergyWithReserveWh, out.missionEnergyCeilingWh);
    text(ax1,0.02,0.02,limitMsg,'Units','normalized','FontSize',7.0, ...
        'Color',[0.35 0.05 0.05],'BackgroundColor',[1.0 0.96 0.91], ...
        'EdgeColor',[0.7 0.3 0.2],'Interpreter','none','Margin',4, ...
        'VerticalAlignment','bottom');
elseif out.telemetryLimitTriggered || out.energyReserveLimitTriggered
    limitMsg = sprintf('Pursuit stopped: %s', out.operationalLimitReason);
    text(ax1,0.02,0.02,limitMsg,'Units','normalized','FontSize',7.2, ...
        'Color',[0.35 0.05 0.05],'BackgroundColor',[1.0 0.96 0.91], ...
        'EdgeColor',[0.7 0.3 0.2],'Interpreter','none','Margin',4);
elseif isfield(out,'targetExitedInnerZoneBeforeCandidate') && out.targetExitedInnerZoneBeforeCandidate
    refMsg = sprintf('Target crossed the %.0f m reference circle at %.2f s; pursuit continues subject to link and energy limits.', ...
        out.keepOutRadius,out.innerZoneExitTime_s);
    text(ax1,0.02,0.02,refMsg,'Units','normalized','FontSize',8, ...
        'Color',[0.15 0.15 0.3],'BackgroundColor',[0.95 0.96 1.0], ...
        'EdgeColor',[0.4 0.45 0.65],'Interpreter','none','Margin',5);
elseif out.isGateOnlyScenario || ~out.launchAuthorized
    text(ax1,0.02,0.02,char(out.launchGateDecision),'Units','normalized','FontSize',8, ...
        'Color','k','BackgroundColor','w','EdgeColor',[0.7 0.7 0.7], 'Interpreter','none');
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
elseif isfield(out,'targetExitedInnerZoneBeforeCandidate') && out.targetExitedInnerZoneBeforeCandidate
    xline(ax2,out.innerZoneExitTime_s,'r:','1 km reference crossing');
end
xlabel(ax2,'Time (s)'); ylabel(ax2,'Interceptor-target range (m)');
if isfinite(out.closingAtCapture)
    rangeTitle = sprintf('Range before candidate (V_{close}=%.2f m/s)',out.closingAtCapture);
else
    rangeTitle = 'Range while vehicles remain separate';
end
title(ax2,rangeTitle,'Color','k','FontSize',10,'Interpreter','tex');
styleAxes(ax2);

% Near-range inset: the 5-15 m interval is the assumed geometric engagement
% window, not a model of net deployment or proof of capture success.
if isfinite(out.captureTime) && any(validRange)
    pos = ax2.Position;
    insetPos = [pos(1)+0.61*pos(3), pos(2)+0.10*pos(4), ...
                0.34*pos(3), 0.27*pos(4)];
    axInset = axes('Parent',fig,'Position',insetPos,'Color','w');
    hold(axInset,'on'); grid(axInset,'on');
    plot(axInset,out.time(validRange),out.range(validRange),'k-','LineWidth',1.0);
    yline(axInset,cfg.captureRange(1),'r--');
    yline(axInset,cfg.captureRange(2),'r--');
    xlim(axInset,[max(0,out.captureTime-2.0), out.captureTime+0.1]);
    ylim(axInset,[0,60]);
    title(axInset,'Near-candidate range (m)','FontSize',7,'Color','k','Interpreter','none');
    set(axInset,'FontSize',7,'XColor','k','YColor','k','GridColor',[0.8 0.8 0.8]);
    axInset.Box = 'on';
end

% 3) Speed check
ax3 = subplot(3,2,4,'Parent',fig);
plot(ax3,out.time,out.interceptorSpeed,'b-','LineWidth',1.5); hold(ax3,'on'); grid(ax3,'on');
yline(ax3,cfg.maxSpeed,'r--','25 m/s limit');
if any(out.phase == "TOW")
    yline(ax3,cfg.maxTowSpeed,'g:','Tow cruise setpoint (ramp-limited)');
end
xlabel(ax3,'Time (s)'); ylabel(ax3,'Interceptor speed (m/s)');
title(ax3,sprintf('Speed check (max observed %.2f m/s)',out.maxSpeedObserved),'Color','k');
styleAxes(ax3);

% 4) Acceleration check. This checks the model's assumed cap only; it does
% not establish actual aircraft thrust, power or controller capability.
ax4 = subplot(3,2,5,'Parent',fig);
requestedAccelerationMagnitude = sqrt(sum(out.commandedAcceleration.^2,1))';
controllerCommandMagnitude = sqrt(sum(out.controllerCommandAcceleration.^2,1))';
% Candidate sample is a geometric event boundary; no control interval is integrated there.
requestedAccelerationMagnitude(out.phase(:) == "CAPTURE_CANDIDATE") = NaN;
controllerCommandMagnitude(out.phase(:) == "CAPTURE_CANDIDATE") = NaN;
plot(ax4,out.time,requestedAccelerationMagnitude,'--','Color',[0.85 0.35 0.1],'LineWidth',1.2);
hold(ax4,'on'); grid(ax4,'on');
plot(ax4,out.time,controllerCommandMagnitude,'Color',[0.05 0.55 0.2],'LineWidth',1.15);
plot(ax4,out.time,out.accelerationMagnitude,'Color',[0.45 0.15 0.65],'LineWidth',1.4);
yline(ax4,cfg.accelLimit,'r--',sprintf('Applied cap %.1f m/s^2',cfg.accelLimit));
finiteRequested = requestedAccelerationMagnitude(isfinite(requestedAccelerationMagnitude));
if isempty(finiteRequested), peakRequestedForTitle = 0; else, peakRequestedForTitle = max(finiteRequested); end
xlabel(ax4,'Time (s)'); ylabel(ax4,'Acceleration magnitude (m/s^2)');
title(ax4,sprintf('Raw demand %.2f; bounded command %.2f; applied max %.2f m/s^2', ...
    peakRequestedForTitle,max(controllerCommandMagnitude,[],'omitnan'),out.maxAccelerationObserved),'Color','k','FontSize',8.5);
legend(ax4,{'Raw guidance demand','Command after controller saturation','Applied model acceleration','Assumed acceleration cap'}, ...
    'Location','best','FontSize',7);
styleAxes(ax4);

% 5) Simulated-phase energy, corrected sizing ceiling, and usable battery energy.
ax5 = subplot(3,2,6,'Parent',fig);
plot(ax5,out.time,out.energyWh,'Color',[0.1 0.55 0.2],'LineWidth',1.5);
hold(ax5,'on'); grid(ax5,'on');
projectedEnergyPlot = out.projectedMissionEnergyWhHistory(:);
projectedEnergyValid = isfinite(projectedEnergyPlot);
if any(projectedEnergyValid)
    plot(ax5,out.time(projectedEnergyValid),projectedEnergyPlot(projectedEnergyValid), ...
        'm-.','LineWidth',1.1);
end
if isfinite(out.energyBudget)
    hCeiling = yline(ax5,out.energyBudget,'r--','LineWidth',1.0); %#ok<NASGU>
end
hBattery = yline(ax5,cfg.batteryUsableEnergyWh,'b:','LineWidth',1.0); %#ok<NASGU>
finiteProjectionForScale = projectedEnergyPlot(isfinite(projectedEnergyPlot));
if isempty(finiteProjectionForScale)
    maxProjectionForScale = 0;
else
    maxProjectionForScale = max(finiteProjectionForScale);
end
finiteEnergyBudgetForScale = [];
if isfinite(out.energyBudget), finiteEnergyBudgetForScale = out.energyBudget; end
yPlotMax = 1.10 * max([cfg.batteryUsableEnergyWh, finiteEnergyBudgetForScale, ...
    max(out.energyWh), maxProjectionForScale, 1]);
ylim(ax5,[0, yPlotMax]);
% Put short threshold labels inside the axes instead of using yline labels at
% the far-right edge, where the two labels can collide in report-sized figures.
text(ax5,0.985,cfg.batteryUsableEnergyWh/yPlotMax, ...
    sprintf('Usable battery %.0f Wh',cfg.batteryUsableEnergyWh), ...
    'Units','normalized','HorizontalAlignment','right','VerticalAlignment','bottom', ...
    'FontSize',7,'Color',[0.05 0.20 0.65],'BackgroundColor','w','Margin',1);
if isfinite(out.energyBudget)
    text(ax5,0.985,out.energyBudget/yPlotMax, ...
        sprintf('Mission ceiling %.0f Wh',out.energyBudget), ...
        'Units','normalized','HorizontalAlignment','right','VerticalAlignment','top', ...
        'FontSize',7,'Color',[0.7 0.05 0.05],'BackgroundColor','w','Margin',1);
end
xlabel(ax5,'Time (s)'); ylabel(ax5,'Estimated energy (Wh)');
title(ax5,'Simulated energy and projected requirement checkpoints','Color','k','FontSize',10);
if out.isGuidanceDiagnostic
    msg = 'PN diagnostic only; no mission-energy projection';
elseif out.isGateOnlyScenario || ~out.launchAuthorized
    msg = 'Not applicable: no mission flight simulated';
elseif out.energyReserveLimitTriggered && out.captureCandidateRejectedByEnergyGuard
    msg = sprintf(['ENERGY GUARD: %.1f Wh projected > %.0f Wh ceiling\n' ...
        '%.1f used + %.1f remaining + %.1f buffer\n' ...
        'Candidate window reached at %.2f m; not registered'], ...
        out.projectedMissionEnergyWithReserveWh, out.missionEnergyCeilingWh, ...
        out.energyUsedSimulatedPhasesWh, out.estimatedFutureEnergyWh, ...
        out.additionalPhaseBufferWh, out.minimumRange);
elseif out.energyReserveLimitTriggered
    msg = sprintf(['Energy guard: %.1f Wh projected > %.0f Wh ceiling\n' ...
        '%.1f used + %.1f remaining + %.1f buffer'], ...
        out.projectedMissionEnergyWithReserveWh, out.missionEnergyCeilingWh, ...
        out.energyUsedSimulatedPhasesWh, out.estimatedFutureEnergyWh, out.additionalPhaseBufferWh);
elseif out.telemetryLimitTriggered
    msg = sprintf('Link guard stopped pursuit at %.0f m limit; projection %.1f Wh (estimate only)', ...
        out.effectiveLinkRangeLimit_m, out.projectedMissionEnergyWithReserveWh);
elseif out.towCompleted
    msg = sprintf('Tow endpoint projection: %.1f Wh = %.1f used + %.1f buffer; landing/return not modelled', ...
        out.projectedMissionEnergyWithReserveWh, out.energyUsedSimulatedPhasesWh, out.additionalPhaseBufferWh);
elseif contains(out.releaseTowDecision,'Tow required') && ~out.towCompleted
    msg = sprintf('Tow incomplete: %.2f Wh simulated-phase energy only; no mission-total projection',out.energyWh(end));
elseif isfield(out,'parachuteRequestFlag') && out.parachuteRequestFlag
    msg = sprintf('Overweight recovery not simulated: %.2f Wh phase energy only; no mission-total projection',out.energyWh(end));
elseif isfinite(out.projectedMissionEnergyWithReserveWh)
    msg = sprintf('Projected requirement %.1f Wh = %.1f used + %.1f remaining + %.1f buffer; ceiling %.0f Wh', ...
        out.projectedMissionEnergyWithReserveWh, out.energyUsedSimulatedPhasesWh, ...
        out.estimatedFutureEnergyWh, out.additionalPhaseBufferWh, out.missionEnergyCeilingWh);
else
    msg = sprintf('Simulated-phase energy: %.2f Wh; no mission-total projection available',out.energyWh(end));
end
text(ax5,0.025,0.98,msg,'Units','normalized', ...
    'Color',[0.08 0.08 0.08],'BackgroundColor','w','EdgeColor',[0.55 0.55 0.55], ...
    'FontSize',6.4,'Interpreter','none','Margin',3,'VerticalAlignment','top');
text(ax5,0.025,0.80, ...
    sprintf('Projection = used + estimated remaining segment + phase buffer.\nValues are assumption-based, not physically validated.'), ...
    'Units','normalized','Color',[0.10 0.20 0.48],'BackgroundColor','w', ...
    'EdgeColor',[0.55 0.55 0.55],'FontSize',6.4,'Interpreter','none','Margin',3, ...
    'VerticalAlignment','top');
styleAxes(ax5);

if out.captureCandidateRejectedByEnergyGuard && out.geometricCandidateWindowReached
    titleOutcome = 'ENERGY GUARD STOP: window reached; candidate not registered';
    titleFontSize = 10.5;
else
    titleOutcome = out.outcome;
    titleFontSize = 13;
end
sg = sgtitle(sprintf('%s\nMode: %s | Outcome: %s', out.name, out.mode, titleOutcome), ...
    'Interpreter','none','FontSize',titleFontSize);
sg.Color = 'k';
annotation(fig,'textbox',[0.02 0.005 0.96 0.028], ...
    'String','PRELIMINARY 2-D POINT-MASS MODEL | Mass, propulsion and energy inputs include unverified estimates | 5-15 m marks are geometric candidates only', ...
    'Units','normalized','FitBoxToText','off','Color',[0.18 0.18 0.18], ...
    'BackgroundColor',[0.96 0.96 0.96],'EdgeColor',[0.65 0.65 0.65], ...
    'FontSize',8,'HorizontalAlignment','center','VerticalAlignment','middle', ...
    'Interpreter','none','Margin',2);
end



function plotTerminalApproachSummary(results, cfg)
% Four local views in a 2x2 grid avoid cramped titles and duplicate legends.
% Each point is a geometric candidate only; relative closing speed is kinematic.
capturedIdx = [];
for k = 1:numel(results)
    if isfinite(results{k}.captureTime) && ~results{k}.isGuidanceDiagnostic
        capturedIdx(end+1) = k; %#ok<AGROW>
    end
end
if isempty(capturedIdx), return; end

fig = figure('Name','SKYFORCE terminal approach details','Color','w', ...
    'Position',[60 45 1500 930]);
shortLabels = strings(numel(capturedIdx),1);
for j = 1:numel(capturedIdx)
    out = results{capturedIdx(j)};
    ax = subplot(2,2,j,'Parent',fig);
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    preCapture = out.time <= out.captureTime + 1e-9;
    plot(ax,out.pI(1,preCapture),out.pI(2,preCapture),'b-','LineWidth',1.7);
    plot(ax,out.pT(1,preCapture),out.pT(2,preCapture), ...
        'Color',[0.85 0.35 0.1],'LineWidth',1.7);
    pIfinal = out.pI(:,find(preCapture,1,'last'));
    pTfinal = out.pT(:,find(preCapture,1,'last'));
    plot(ax,pIfinal(1),pIfinal(2),'bo','MarkerFaceColor','b','MarkerSize',6);
    plot(ax,pTfinal(1),pTfinal(2),'o','Color',[0.85 0.35 0.1], ...
        'MarkerFaceColor',[0.85 0.35 0.1],'MarkerSize',6);
    plot(ax,[pIfinal(1),pTfinal(1)],[pIfinal(2),pTfinal(2)],'k--','LineWidth',1.2);
    plot(ax,out.capturePoint(1),out.capturePoint(2),'mp','MarkerFaceColor','m','MarkerSize',11);
    xlim(ax,[out.capturePoint(1)-250, out.capturePoint(1)+250]);
    ylim(ax,[out.capturePoint(2)-250, out.capturePoint(2)+250]);
    xlabel(ax,'x (m)'); ylabel(ax,'y (m)');
    if contains(out.name,'Attack mode')
        shortName = 'Attack';
    elseif contains(out.name,'Defense mode')
        shortName = 'Defense';
    elseif contains(out.name,'Tow required')
        shortName = 'Tow';
    elseif contains(out.name,'Overweight')
        shortName = 'Overweight (no tow)';
    else
        shortName = char(out.name);
    end
    shortLabels(j) = string(shortName);
    title(ax,sprintf('%s | candidate range %.2f m | V_{close} %.2f m/s', ...
        shortName,out.captureRangeAtEvent,out.closingAtCapture), ...
        'Interpreter','tex','FontSize',10,'Color','k');
    styleAxes(ax);
end
sg = sgtitle('Terminal approach comparison — pre-capture kinematics only','FontSize',14);
sg.Color = 'k';
annotation(fig,'textbox',[0.06 0.015 0.88 0.045], ...
    'String',['Blue = interceptor; orange = target; dashed black = final separation; magenta = geometric midpoint. ' ...
    'V_{close} is a relative-kinematic value only; net deployment, tether loads and capture reliability are not modelled.'], ...
    'Units','normalized','FitBoxToText','off','Color',[0.10 0.10 0.10], ...
    'BackgroundColor',[0.96 0.96 0.96],'EdgeColor',[0.6 0.6 0.6], ...
    'FontSize',8.5,'HorizontalAlignment','center','VerticalAlignment','middle', ...
    'Interpreter','tex','Margin',4);
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

function tf = PhaseIsNotCaptureCandidate(phase)
% True for samples with a control interval; the geometric event sample itself
% does not advance the point-mass integrator.
tf = string(phase(:)) ~= "CAPTURE_CANDIDATE";
end

function T = buildTrajectoryExport(results,cfg)
% One row per integration sample for independent kinematic/control auditing.
parts = cell(numel(results),1);
for ir = 1:numel(results)
    out = results{ir};
    ns = numel(out.time);
    sampleIndex = (1:ns)';
    ScenarioID = repmat(out.scenarioID,ns,1);
    Scenario = repmat(string(out.name),ns,1);
    ScenarioPurpose = repmat(string(ifelseLocal(out.isGuidanceDiagnostic, ...
        'PN guidance diagnostic only; in-flight initial condition', 'Mission/trajectory scenario')),ns,1);
    LaunchGateDecision = repmat(string(out.launchGateDecision),ns,1);
    LaunchAuthorizedInScenario = repmat(out.launchAuthorized,ns,1);
    TargetMassEstimate_kg = repmat(out.targetMassEstimate_kg,ns,1);
    TargetMassLimit_kg = repmat(out.targetMassLimit_kg,ns,1);
    OverweightTargetAboveLimit = repmat(out.targetMassEstimate_kg > out.targetMassLimit_kg,ns,1);
    % Event flags turn on only at the geometric candidate sample, not during
    % the preceding flight. No post-candidate dynamics are simulated for overload.
    ParachuteRequestFlag_SimOnly = strcmp(out.phase(:), 'OVERWEIGHT_PARACHUTE_REQUEST');
    TowDisabledForOverweight = false(ns,1);
    EstimatedPostCaptureMass_kg = nan(ns,1);
    if isfinite(out.captureSampleIndex) && out.captureSampleIndex >= 1 && out.captureSampleIndex <= ns
        if out.towDisabledForOverweight
            TowDisabledForOverweight(out.captureSampleIndex:end) = true;
        end
        EstimatedPostCaptureMass_kg(out.captureSampleIndex:end) = out.postCaptureMassEstimate_kg;
    end
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
    ControllerLimitedCommandX_mps2 = out.controllerCommandAcceleration(1,:)';
    ControllerLimitedCommandY_mps2 = out.controllerCommandAcceleration(2,:)';
    ControllerLimitedCommandMagnitude_mps2 = sqrt(ControllerLimitedCommandX_mps2.^2 + ControllerLimitedCommandY_mps2.^2);
    IsIntegratedControlSample = PhaseIsNotCaptureCandidate(out.phase(:));
    CommandIsAboveModelCap = IsIntegratedControlSample & (CommandedAccelerationMagnitude_mps2 > cfg.accelLimit + 1e-6);
    RequestedAccelerationOverCap_mps2 = max(CommandedAccelerationMagnitude_mps2 - cfg.accelLimit,0);
    RequestedAccelerationOverCap_mps2(~IsIntegratedControlSample) = NaN;
    ModelAccelerationCap_mps2 = repmat(cfg.accelLimit,ns,1);
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
    TelemetryDistance_m = out.telemetryDistance_m(:);
    EffectiveLinkRangeLimit_m = repmat(out.effectiveLinkRangeLimit_m,ns,1);
    EstimatedFutureEnergyReserve_Wh = out.estimatedFutureEnergyWhHistory(:);
    ProjectedMissionEnergyWithReserve_Wh = out.projectedMissionEnergyWhHistory(:);
    EnergyGuardMissionCeiling_Wh = out.missionEnergyCeilingWhHistory(:);
    parts{ir} = table(ScenarioID,Scenario,ScenarioPurpose,LaunchGateDecision,LaunchAuthorizedInScenario, ...
        TargetMassEstimate_kg,TargetMassLimit_kg,OverweightTargetAboveLimit,ParachuteRequestFlag_SimOnly, ...
        TowDisabledForOverweight,EstimatedPostCaptureMass_kg,sampleIndex,Time_s, ...
        CaptureEventSampleIndex,EventIndex,IsCaptureCandidateSample, ...
        InterceptorX_m,InterceptorY_m,InterceptorVx_mps,InterceptorVy_mps, ...
        TargetX_m,TargetY_m,TargetVx_mps,TargetVy_mps,Range_m,LOSUnitX,LOSUnitY, ...
        RangeRate_mps,ClosingSpeed_mps,CommandedAccelerationX_mps2,CommandedAccelerationY_mps2, ...
        CommandedAccelerationMagnitude_mps2,ControllerLimitedCommandX_mps2,ControllerLimitedCommandY_mps2, ...
        ControllerLimitedCommandMagnitude_mps2,IsIntegratedControlSample,ModelAccelerationCap_mps2, ...
        CommandIsAboveModelCap,RequestedAccelerationOverCap_mps2,AppliedAccelerationX_mps2,AppliedAccelerationY_mps2, ...
        AppliedAccelerationMagnitude_mps2,ActiveLimit,PNAccelerationX_mps2,PNAccelerationY_mps2, ...
        PNAccelerationMagnitude_mps2,PNGuidanceActive,Phase,ElectricalPowerEstimate_W, ...
        IncrementalEnergy_Wh,CumulativeEnergy_Wh,TelemetryDistance_m,EffectiveLinkRangeLimit_m, ...
        EstimatedFutureEnergyReserve_Wh,ProjectedMissionEnergyWithReserve_Wh,EnergyGuardMissionCeiling_Wh);
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

function T = runBoundaryGateDemo(cfg)
% Offline, synthetic demonstration of two concentric decision boundaries.
% It issues no hardware command. Outer entry starts track assessment; launch is
% recommended only if the predicted straight-line track enters the inner
% engagement zone with enough lead time for the illustrative response interval.
site = [0;0]; outerR = cfg.outerTrackingRadius_m; innerR = cfg.innerBoundaryRadius_m;
tracks(1).name = 'Threat track: engagement-zone entry predicted';
tracks(1).p0 = [-3000;0]; tracks(1).v = [20;0];
tracks(2).name = 'Outer ring only: engagement zone missed';
tracks(2).p0 = [-3000;1800]; tracks(2).v = [20;0];
tracks(3).name = 'Threat detected too late';
tracks(3).p0 = [-2500;0]; tracks(3).v = [40;0]; % 25 s from outer ring to 1 km boundary, less than response estimate
nt = numel(tracks);
Scenario = strings(nt,1); TargetSpeed_mps = nan(nt,1); TargetHeading_deg = nan(nt,1);
InitialRangeToSite_m = nan(nt,1); ClosestApproach_m = nan(nt,1);
OuterTrackingRadius_m = repmat(outerR,nt,1); InnerReferenceCircleRadius_m = repmat(innerR,nt,1);
TimeToOuterEntry_s = nan(nt,1); TimeToReferenceCircleEntryFromTrack_s = nan(nt,1);
PredictedReferenceCircleEntry = false(nt,1); EstimatedResponseTime_s = repmat(cfg.gateResponseTimeTotal_s,nt,1);
DecisionTime_s = nan(nt,1); LaunchRecommendedBeforeInner = false(nt,1); GateDecision = strings(nt,1); Rationale = strings(nt,1);
for i=1:nt
    p0=tracks(i).p0; v=tracks(i).v;
    Scenario(i)=string(tracks(i).name);
    TargetSpeed_mps(i)=norm(v); TargetHeading_deg(i)=atan2d(v(2),v(1));
    InitialRangeToSite_m(i)=norm(p0-site);
    [~,ClosestApproach_m(i)] = closestApproach(p0,v,site);
    [tOuter, outerHit] = timeToCircleEntry(p0,v,site,outerR);
    if ~outerHit
        TimeToOuterEntry_s(i)=NaN;
        GateDecision(i)="NO TAKEOFF: target never enters outer tracking ring";
        Rationale(i)="No track acquisition event in the illustrative horizon.";
        continue;
    end
    TimeToOuterEntry_s(i)=tOuter;
    pAtTrack=p0+v*tOuter;
    [tInnerFromTrack, innerHit] = timeToCircleEntry(pAtTrack,v,site,innerR);
    PredictedReferenceCircleEntry(i)=innerHit;
    if innerHit, TimeToReferenceCircleEntryFromTrack_s(i)=tInnerFromTrack; end
    if ~innerHit
        GateDecision(i)="NO TAKEOFF IN THIS GATE-ONLY DEMO: track misses the 1,000 m reference circle";
        Rationale(i)=sprintf('Gate-only scenario retains the inner-circle launch policy; mission trajectories separately enforce radio/telemetry and energy limits. Closest approach %.0f m.',ClosestApproach_m(i));
    elseif tInnerFromTrack <= EstimatedResponseTime_s(i)
        GateDecision(i)="NO TAKEOFF: insufficient time before 1,000 m engagement-zone entry";
        Rationale(i)=sprintf('Only %.1f s to engagement-zone entry; %.1f s response estimate required.',tInnerFromTrack,EstimatedResponseTime_s(i));
    else
        DecisionTime_s(i)=tOuter;
        LaunchRecommendedBeforeInner(i)=true;
        GateDecision(i)="SIMULATED LAUNCH RECOMMENDATION before engagement-zone entry";
        Rationale(i)=sprintf('%.1f s available after outer-ring acquisition; %.1f s response estimate.',tInnerFromTrack,EstimatedResponseTime_s(i));
    end
end
T=table(Scenario,TargetSpeed_mps,TargetHeading_deg,InitialRangeToSite_m,ClosestApproach_m, ...
    OuterTrackingRadius_m,InnerReferenceCircleRadius_m,TimeToOuterEntry_s,TimeToReferenceCircleEntryFromTrack_s, ...
    PredictedReferenceCircleEntry,EstimatedResponseTime_s,DecisionTime_s,LaunchRecommendedBeforeInner,GateDecision,Rationale);
plotBoundaryGateDemo(tracks,site,outerR,innerR,T,cfg);
end

function plotBoundaryGateDemo(tracks,site,outerR,innerR,T,cfg)
fig=figure('Name','SKYFORCE two-boundary launch-gate demo','Color','w','Position',[80 70 1350 760]);
ax1=subplot(1,2,1,'Parent',fig); hold(ax1,'on'); grid(ax1,'on'); axis(ax1,'equal');
a=linspace(0,2*pi,400);
plot(ax1,site(1)+outerR*cos(a),site(2)+outerR*sin(a),'b--','LineWidth',1.6);
plot(ax1,site(1)+innerR*cos(a),site(2)+innerR*sin(a),'r--','LineWidth',1.6);
plot(ax1,site(1),site(2),'kp','MarkerFaceColor','k','MarkerSize',10);
plot(ax1,cfg.interceptorStationPosition_m(1),cfg.interceptorStationPosition_m(2),'kd', ...
    'MarkerFaceColor',[0.2 0.7 0.85],'MarkerSize',8);
cols={[0.1 0.55 0.2],[0.8 0.4 0.1],[0.55 0.2 0.65]};
labels=cell(1,numel(tracks)+4);
labels{1}='Outer tracking radius'; labels{2}='Inner reference circle';
labels{3}='Protected site centre'; labels{4}='Fixed interceptor station (illustrative)';
for i=1:numel(tracks)
    tMax=200;
    [tOuter,outerHit]=timeToCircleEntry(tracks(i).p0,tracks(i).v,site,outerR);
    if outerHit && T.PredictedReferenceCircleEntry(i) && isfinite(T.TimeToReferenceCircleEntryFromTrack_s(i))
        tMax=min(tMax,tOuter+T.TimeToReferenceCircleEntryFromTrack_s(i)+8);
    elseif outerHit
        tMax=min(tMax,max(tOuter+45,60));
    end
    tt=linspace(0,tMax,300);
    xy=tracks(i).p0+tracks(i).v*tt;
    plot(ax1,xy(1,:),xy(2,:),'Color',cols{i},'LineWidth',1.6);
    plot(ax1,tracks(i).p0(1),tracks(i).p0(2),'o','Color',cols{i},'MarkerFaceColor',cols{i});
    if outerHit
        pe=tracks(i).p0+tracks(i).v*tOuter;
        plot(ax1,pe(1),pe(2),'s','Color',cols{i},'MarkerFaceColor',cols{i});
    end
    labels{i+4}=char(tracks(i).name);
end
legend(ax1,labels,'Location','best','Interpreter','none','FontSize',8);
xlabel(ax1,'x relative to protected site (m)'); ylabel(ax1,'y relative to protected site (m)');
title(ax1,sprintf('Two-boundary concept: fixed interceptor station at (%.0f, %.0f) m', ...
    cfg.interceptorStationPosition_m(1),cfg.interceptorStationPosition_m(2)),'Color','k'); styleAxes(ax1);

% NOTE: annotation positions are normalized figure coordinates [0..1].
% The previous version mistakenly used data-like coordinates (e.g. y=3.2),
% which caused MATLAB R2025b to throw: 'Position values must be between 0 and 1'.
ax2=subplot(1,2,2,'Parent',fig); axis(ax2,[0.5 3.5 0 4]); axis(ax2,'off'); hold(ax2,'on');
annotation(fig,'textbox',[0.53 0.80 0.43 0.12], ...
    'String',sprintf('Outer tracking radius: %.0f m\nTrack position, speed, heading and predicted path (simulation only)',outerR), ...
    'Units','normalized','FitBoxToText','off','BackgroundColor',[0.90 0.95 1.0], ...
    'EdgeColor',[0.2 0.4 0.7],'Color',[0.08 0.08 0.08],'FontSize',10, ...
    'FontWeight','normal','VerticalAlignment','middle','Margin',8,'Interpreter','none');
annotation(fig,'textbox',[0.53 0.64 0.43 0.12], ...
    'String',sprintf('Inner reference-circle radius: %.0f m\nPursuit may continue outside subject to link and energy limits',innerR), ...
    'Units','normalized','FitBoxToText','off','BackgroundColor',[1.0 0.93 0.93], ...
    'EdgeColor',[0.75 0.2 0.2],'Color',[0.08 0.08 0.08],'FontSize',10, ...
    'FontWeight','normal','VerticalAlignment','middle','Margin',8,'Interpreter','none');
rowY=[0.45 0.29 0.13];
for i=1:min(3,height(T))
    annotation(fig,'textbox',[0.53 rowY(i) 0.43 0.12], ...
        'String',sprintf('%s\n%s',char(T.Scenario(i)),char(T.GateDecision(i))), ...
        'Units','normalized','FitBoxToText','off','BackgroundColor',[0.97 0.97 0.97], ...
        'EdgeColor',cols{i},'Color',[0.08 0.08 0.08],'FontSize',9, ...
        'FontWeight','normal','VerticalAlignment','middle','Margin',7,'Interpreter','none');
end
sg=sgtitle('Offline gate logic only — no autopilot or hardware commands','Color','k','FontSize',14); %#ok<NASGU>
annotation(fig,'textbox',[0.08 0.005 0.84 0.035], ...
    'String','Synthetic constant-velocity tracks; response-time threshold and radii are model assumptions; decisions are not real launch commands.', ...
    'Units','normalized','FitBoxToText','off','Color',[0.12 0.12 0.12], ...
    'BackgroundColor',[0.96 0.96 0.96],'EdgeColor',[0.6 0.6 0.6], ...
    'FontSize',8,'HorizontalAlignment','center','VerticalAlignment','middle', ...
    'Interpreter','none','Margin',3);
end

function T = runOverweightContingencyDemo(cfg)
% Logic-only demonstration. The target-mass estimate is an assumed input, not
% inferred from radar alone. A parachute-request flag is NOT a hardware command
% and does not validate parachute rating, opening shock, or landing safety.
masses=[cfg.overweightDemoMass_kg; cfg.overweightDemoMass_kg];
DetectionStage=["Mass known during tracking, before launch"; "After geometric capture-candidate event"];
LaunchDecision=["No mass-only abort; use trajectory/time launch gate"; ...
    "Continue to candidate; immediate simulated parachute request"];
ParachuteRequest=[false; true]; % only set at/after the geometric candidate event
TowAllowed=[false; false];
TargetMassLimit_kg=repmat(cfg.targetMassLimit_kg,2,1);
LimitExceeded_kg=masses-TargetMassLimit_kg;
DeploymentDynamicsModelled=false(2,1);
RecoveryValidated=false(2,1);
Reason=["Mass alone does not reject launch; if the trajectory/time gate authorizes launch and a geometric candidate occurs, request parachute recovery and do not tow."; ...
    "Immediate simulated parachute-request flag after candidate; no towing; actual deployment, load rating, opening shock and safe landing remain unvalidated."];
T=table(DetectionStage,masses,TargetMassLimit_kg,LimitExceeded_kg,LaunchDecision,ParachuteRequest, ...
    TowAllowed,DeploymentDynamicsModelled,RecoveryValidated,Reason, ...
    'VariableNames',{'DetectionStage','TargetMass_kg','TargetMassLimit_kg','LimitExceeded_kg', ...
    'Decision','ParachuteRequestFlag_SimOnly','TowAllowed','DeploymentDynamicsModelled','RecoveryValidated','Rationale'});
plotOverweightContingencyDemo(cfg,T);
end

function plotOverweightContingencyDemo(cfg,T)
fig=figure('Name','SKYFORCE target mass limit / recovery contingency','Color','w','Position',[100 80 1250 650]);
ax=axes('Parent',fig); axis(ax,[0 10 0 6]); axis(ax,'off'); hold(ax,'on');
annotation(fig,'textbox',[0.12 0.70 0.76 0.16],'String', ...
    sprintf('Target mass estimate = %.1f kg; nominal target limit = %.1f kg',T.TargetMass_kg(1),cfg.targetMassLimit_kg), ...
    'FitBoxToText','off','BackgroundColor',[1.0 0.94 0.88],'EdgeColor',[0.7 0.25 0.1], ...
    'Color',[0.08 0.08 0.08],'FontSize',13,'FontWeight','bold', ...
    'HorizontalAlignment','center','VerticalAlignment','middle','Margin',8,'Interpreter','none');
annotation(fig,'textbox',[0.12 0.43 0.34 0.18],'String', ...
    sprintf('Known during tracking\nNo mass-only launch abort\nApply trajectory/time gate'), ...
    'FitBoxToText','off','BackgroundColor',[0.90 0.96 0.90],'EdgeColor',[0.1 0.5 0.2], ...
    'Color',[0.08 0.08 0.08],'FontSize',11,'HorizontalAlignment','center', ...
    'VerticalAlignment','middle','Margin',8,'Interpreter','none');
annotation(fig,'textbox',[0.54 0.43 0.34 0.18],'String', ...
    sprintf('After geometric candidate\nImmediate simulated parachute-request\nNo towing; deployment not modelled'), ...
    'FitBoxToText','off','BackgroundColor',[1.0 0.93 0.93],'EdgeColor',[0.7 0.2 0.2], ...
    'Color',[0.08 0.08 0.08],'FontSize',11,'HorizontalAlignment','center', ...
    'VerticalAlignment','middle','Margin',8,'Interpreter','none');
annotation(fig,'textbox',[0.12 0.12 0.76 0.17],'String', ...
    sprintf('Safety note: this is a state-logic flag only, not a real deployment command. The revised 2.5 m canopy sizing reference uses a %.2f kg suspended load (5 kg target + 0.30 kg net/tether) and an estimated %.2f m/s descent rate. A heavier load is outside that sizing assumption; deployment dynamics, structural loads and safe recovery are not validated.', cfg.parachuteSuspendedMassNominal_kg, cfg.parachuteDescentRateEstimate_mps), ...
    'FitBoxToText','off','BackgroundColor',[1 1 0.84],'EdgeColor',[0.6 0.5 0.1], ...
    'Color',[0.08 0.08 0.08],'FontSize',9.5,'HorizontalAlignment','center', ...
    'VerticalAlignment','middle','Margin',7,'Interpreter','none');
sg=sgtitle('Overweight-target policy — simulated parachute request only','Color','k','FontSize',14); %#ok<NASGU>
end

function T = buildModelAssumptionsTable(cfg)
% Machine-readable register explicitly separates assumptions from verified data.
Category = ["Mass";"Mass";"Mass";"Mass";"Propulsion";"Propulsion";"Propulsion"; ...
    "Control";"Control";"Control";"Energy";"Energy";"Energy";"Aerodynamics"; ...
    "Capture";"Capture";"Launch gate";"Launch gate";"Interceptor station";"Operating range"; ...
    "Recovery";"Recovery";"Communications";"Communications";"Energy reserve";"Model scope"];
Parameter = ["Nominal mass";"Post-capture mass (5 kg target)";"16-line component subtotal"; ...
    "Integration/growth allowance";"Maximum static thrust";"Forward-flight thrust derating"; ...
    "Drag-balance speed";"Maximum model speed";"Applied acceleration cap"; ...
    "Requested-command saturation";"Battery product and usable energy";"Hover/sprint power"; ...
    "Additional phase energy buffers";"Drag coefficient and frontal area"; ...
    "5-15 m engagement window";"Candidate closing speed";"Outer tracking / inner engagement-zone radii"; ...
    "Gate response-time calculation";"Fixed interceptor-station position on inner boundary";"1.5-2.0 km initial-range requirement"; ...
    "Parachute descent estimate";"Overweight parachute request";"Radio range";"Telemetry range";"Return/delivery energy guard";"Dynamics coverage"];
Value = [string(sprintf('%.2f kg',cfg.massNominal)); ...
    string(sprintf('%.2f kg',cfg.massWithCaptured)); ...
    string(sprintf('%.2f kg',cfg.massLedgerSubtotal)); ...
    string(sprintf('%.2f kg',cfg.massGrowthAllowance)); ...
    string(sprintf('%.1f N total, static estimate',cfg.totalMaxThrustN)); ...
    string('15%, 20%, 30% sensitivity cases'); ...
    string(sprintf('%.1f / %.1f m/s idealized',cfg.staticDragBalanceSpeedNominal_mps,cfg.staticDragBalanceSpeedPostCapture_mps)); ...
    string(sprintf('%.1f m/s',cfg.maxSpeed)); ...
    string(sprintf('%.1f m/s^2',cfg.accelLimit)); ...
    string('Raw requested commands are compared against cap; applied acceleration is rate-limited'); ...
    string(sprintf('%.1f Ah; %.1f Wh nominal; %.1f Wh counted usable',cfg.batteryCapacity_Ah,cfg.batteryNominalEnergyWh,cfg.batteryUsableEnergyWh)); ...
    string(sprintf('%.0f W hover; %.0f W sprint',cfg.hoverPower,cfg.sprintPower)); ...
    string(sprintf('%.1f Wh no-tow; %.1f Wh tow',cfg.energyBufferNoTow_Wh,cfg.energyBufferTow_Wh)); ...
    string(sprintf('Cd=%.2f; area=%.2f m^2',cfg.dragCoefficient,cfg.frontalArea_m2)); ...
    string(sprintf('%.1f to %.1f m',cfg.captureRange(1),cfg.captureRange(2))); ...
    string('Computed from relative velocity and line-of-sight unit vector'); ...
    string(sprintf('%.0f m outer; %.0f m inner',cfg.outerTrackingRadius_m,cfg.innerBoundaryRadius_m)); ...
    string(sprintf('%.1f s reaction/latency + %.1f s flight-response estimate + %.1f s margin = %.1f s total', ...
        cfg.reactionDelay,cfg.gateEstimatedInterceptTime_s,cfg.gateSafetyMargin_s,cfg.gateResponseTimeTotal_s)); ...
    string(sprintf('(%.0f, %.0f) m; station lies on %.0f m inner boundary', ...
        cfg.interceptorStationPosition_m(1),cfg.interceptorStationPosition_m(2),cfg.innerBoundaryRadius_m)); ...
    string(sprintf('%.0f-%.0f m band; assessment is not a hard gate',cfg.workingRangeMin,cfg.workingRangeMax)); ...
    string(sprintf('%.2f m/s for %.2f m canopy assumption',cfg.parachuteDescentRateEstimate_mps,2.5)); ...
    string(sprintf('%.1f kg threshold; request flag only',cfg.targetMassLimit_kg)); ...
    string(sprintf('%.0f m assumed max radio range; effective link uses the smaller limit',cfg.radioRangeMax_m)); ...
    string(sprintf('%.0f m assumed max telemetry range from (%.0f, %.0f) m',cfg.telemetryRangeMax_m, ...
        cfg.telemetryStationPosition_m(1),cfg.telemetryStationPosition_m(2))); ...
    string(sprintf('Return speed %.1f m/s + %.1f s route margin + phase buffer; approximate only', ...
        cfg.returnReserveCruiseSpeed_mps,cfg.returnReserveTimeMargin_s)); ...
    string('2-D point mass; no wind, obstacles, 3-D dynamics, net deployment or parachute deployment')];
EvidenceStatus = ["Provisional target; not weighed aircraft"; ...
    "Derived from nominal mass + target; assumes net/tether already included"; ...
    "Arithmetic sum reconciled; most line items are estimates"; ...
    "Arithmetic headroom, not separately installed mass"; ...
    "Static rating-based estimate; verify exact motor/prop/ESC on test stand"; ...
    "Assumed sensitivity only; not measured"; ...
    "Static idealization; not a verified top speed"; ...
    "Simulation constraint; not demonstrated aircraft capability"; ...
    "Simulation constraint; not measured controller/aircraft acceleration"; ...
    "Saturation is observed in the model only; no actuator/hardware response model"; ...
    "Battery product unselected; 80% usable fraction assumed"; ...
    "Sizing inputs; not power telemetry or a continuous flight integration"; ...
    "Assumed allowances; full launch-to-landing integration not performed"; ...
    "Provisional coefficients; no flight/CFD identification"; ...
    "Geometric event only; net/target capture physics absent"; ...
    "Kinematic value only; no acceptable physical capture-speed limit defined"; ...
    "Outer/inner radii are scenario values; launch decision is synthetic"; ...
    "Synthetic constant-velocity tracks; 1.8 s reaction/latency is a stacked assumption"; ...
    "Illustrative common site-centred coordinate; exactly 1,000 m from protected-site centre in every scenario initial condition"; ...
    "Scenario-by-scenario diagnostic; universal applicability needs guide confirmation"; ...
    "Calculated sizing estimate; no drop test or validated canopy model"; ...
    "Software state flag only; no deployment command or recovery simulation"; ...
    "Assumed value, not RF/link-budget validated; set from the selected radio and antenna test data"; ...
    "Assumed value, not RF/link-budget validated; use measured control-link range under intended conditions"; ...
    "Approximate energy estimate; validate with measured power, route geometry, wind and battery telemetry"; ...
    "Preliminary model; not flight-control software or physical validation"];
VerificationNeeded = ["Measure completed aircraft mass"; ...
    "Confirm actual carried target and capture-hardware mass"; ...
    "Close BOM against CAD and measured masses"; ...
    "Replace with margin policy after weighed design"; ...
    "Bench-test exact propulsion configuration and electrical limits"; ...
    "Measure thrust loss under relevant forward-flight conditions"; ...
    "Do not cite as top speed; validate through approved engineering analysis/test"; ...
    "Validate through controlled performance testing"; ...
    "Validate control authority and structural/propulsion limits"; ...
    "Review peak command and over-cap duration; model does not simulate actual actuator clipping"; ...
    "Select exact battery; check mass, C-rating, voltage sag and usable energy"; ...
    "Replace with measured/profile-based electrical power model"; ...
    "Integrate each mission phase and recovery/landing energy"; ...
    "Confirm with geometry/CFD/test data"; ...
    "Model capture geometry and net/tether dynamics; no physical success claim"; ...
    "Capture loads and mechanism must be qualified independently"; ...
    "Confirm requirements and operating rules with project guide"; ...
    "Keep formula consistent; replace synthetic timing with sensor/control latency model"; ...
    "Confirm actual site geography, common coordinate frame and fixed station location; validate the 2-D assumption"; ...
    "Ask guide whether all main cases must start within 1.5-2.0 km or only nominal design case"; ...
    "Validate canopy, load, opening shock and descent by qualified analysis/test"; ...
    "Do not implement as a real deployment without independent safety validation"; ...
    "Replace with link-budget range including antenna, orientation, interference and margin"; ...
    "Replace with tested telemetry/control-link envelope and specify whether radius is site- or station-centred"; ...
    "Integrate the actual route, loaded mass, voltage sag, wind, takeoff/landing and recovery energy"; ...
    "3-D, wind, obstacles, structural dynamics, hardware limits and recovery remain outside scope"];
T = table(Category,Parameter,Value,EvidenceStatus,VerificationNeeded);
end

function T = buildWorkingRangeAssessment(results,cfg)
% The provided 1.5-2.0 km band is reported transparently without silently
% turning it into a universal mission gate before the guide confirms scope.
n = numel(results);
Scenario = strings(n,1); InitialRange_m = nan(n,1);
MinimumRequiredRange_m = repmat(cfg.workingRangeMin,n,1);
MaximumRequiredRange_m = repmat(cfg.workingRangeMax,n,1);
WithinBand = false(n,1); AppliesAsHardGate = repmat(cfg.workingRangeIsHardGate,n,1);
Assessment = strings(n,1); Interpretation = repmat("Diagnostic only pending guide confirmation of universal vs nominal-case applicability",n,1);
for ii=1:n
    out=results{ii}; Scenario(ii)=string(out.name); InitialRange_m(ii)=out.initialRange;
    WithinBand(ii)=InitialRange_m(ii)>=cfg.workingRangeMin && InitialRange_m(ii)<=cfg.workingRangeMax;
    if out.isGuidanceDiagnostic
        Assessment(ii)="Not applicable: in-flight PN diagnostic";
    elseif out.isGateOnlyScenario
        Assessment(ii)="Not applicable: gate-only demonstration; no interceptor launch";
    elseif WithinBand(ii)
        Assessment(ii)="Within nominal 1.5-2.0 km initial-range band";
    else
        Assessment(ii)="Outside 1.5-2.0 km band; confirm whether scenario exception is allowed";
    end
end
T=table(Scenario,InitialRange_m,MinimumRequiredRange_m,MaximumRequiredRange_m, ...
    WithinBand,AppliesAsHardGate,Assessment,Interpretation);
end


function T = buildMassLedgerTable(cfg)
% The approved 16-line first-pass ledger is the simulation mass baseline.
% Bare-component references are exported in a separate table and do not replace
% these ledger values unless the team explicitly revises the baseline.
Component = ["Airframe (frame, plates, arms, fasteners)"; ...
    "Six motors, U8 Lite KV100"; ...
    "Six ESCs, Alpha 60A"; ...
    "Six propellers, G28x9.2 CF"; ...
    "Landing gear"; ...
    "Battery, 12S 8000 mAh"; ...
    "Power board, wiring, connectors"; ...
    "Flight controller, GPS, IMU"; ...
    "Companion computer"; ...
    "Camera and gimbal"; ...
    "Datalink radio and antenna"; ...
    "Net launcher frame and mechanism"; ...
    "Net and tether"; ...
    "CO2 cartridge"; ...
    "Parachute and release"; ...
    "Miscellaneous"];
Quantity = ["As designed"; "6"; "6"; "6"; "As designed"; "1"; ...
    "As installed"; "1 set"; "1"; "1"; "1"; "As designed"; ...
    "As designed"; "As applicable"; "As designed"; "As installed"];
Mass_kg = [cfg.airframeLedgerMass_kg; cfg.motorSetLedgerMass_kg; ...
    cfg.escSetLedgerMass_kg; cfg.propellerSetLedgerMass_kg; ...
    cfg.landingGearLedgerMass_kg; cfg.batteryMass_kg; ...
    cfg.powerWiringLedgerMass_kg; cfg.flightControlLedgerMass_kg; ...
    cfg.companionComputerLedgerMass_kg; cfg.cameraGimbalLedgerMass_kg; ...
    cfg.datalinkLedgerMass_kg; cfg.netLauncherLedgerMass_kg; ...
    cfg.netTetherMass_kg; cfg.co2HardwareLedgerMass_kg; ...
    cfg.parachuteReleaseLedgerMass_kg; cfg.miscLedgerMass_kg];
Status = ["First-pass estimate"; ...
    "Ledger value 1.43 kg; bare spec 6 x 0.238 kg = 1.428 kg"; ...
    "First-pass ledger placeholder; bare spec 6 x 0.073 kg = 0.438 kg"; ...
    "First-pass ledger placeholder; bare spec 6 x 0.059 kg = 0.354 kg"; ...
    "Estimated"; "Estimated; exact product not selected"; ...
    "Estimated"; "First-pass estimate; exact Standard Set not confirmed"; ...
    "First-pass estimate; actual carrier assembly not finalized"; ...
    "First-pass estimate; exact camera/gimbal mass unknown"; ...
    "First-pass estimate; bare modem reference 0.014 kg"; ...
    "Estimated fabrication placeholder"; "Estimated; included in nominal mass"; ...
    "Estimated"; "Estimated; itemized breakdown pending"; "Estimated"];
IncludedInSubtotal = true(16,1);
T = table(Component,Quantity,Mass_kg,IncludedInSubtotal,Status);
% Verify ledger arithmetic at runtime so later edits do not silently drift.
ledgerSum = sum(T.Mass_kg);
if abs(ledgerSum-cfg.massLedgerSubtotal) > 1e-8
    warning('SKYFORCE:MassLedgerMismatch', ...
        'Mass ledger sums to %.3f kg, but cfg.massLedgerSubtotal is %.3f kg.', ...
        ledgerSum,cfg.massLedgerSubtotal);
end
end

function T = buildComponentMassReferenceTable(cfg)
% Bare/specification reference masses are explicitly separate from the 16-line
% design ledger; comparison does not imply equivalent inclusion scope.
Component = ["Motor set (6 motors)"; ...
    "Propeller set (6 propellers)"; ...
    "ESC set (6 ESCs)"; ...
    "Cube Orange+ bare flight-controller module"; ...
    "Jetson Orin Nano full dev-kit assembly"; ...
    "RFD900x-class bare modem"];
ReferenceMass_kg = [6*cfg.motorMassEach_kg; 6*0.059; 6*0.073; 0.053; 0.174; 0.014];
LedgerMass_kg = [cfg.motorSetLedgerMass_kg; cfg.propellerSetLedgerMass_kg; ...
    cfg.escSetLedgerMass_kg; cfg.flightControlLedgerMass_kg; ...
    cfg.companionComputerLedgerMass_kg; cfg.datalinkLedgerMass_kg];
DifferenceLedgerMinusReference_kg = LedgerMass_kg-ReferenceMass_kg;
ReferenceScope = ["Six motor bodies as supplied; rounds to 1.43 kg ledger entry"; ...
    "Bare propeller reference; ledger value may include undocumented allowance, not verified"; ...
    "Bare ESC reference; ledger value may include undocumented allowance, not verified"; ...
    "Bare module only; excludes carrier board and GPS/compass; not like-for-like"; ...
    "Full dev-kit bound supplied; selected installed assembly not confirmed"; ...
    "Bare modem only; ledger's 0.50 kg entry includes nominal antenna but is an unresolved outlier"];
T = table(Component,ReferenceMass_kg,LedgerMass_kg,DifferenceLedgerMinusReference_kg,ReferenceScope);
end

function [tEntry, doesEnter] = timeToCircleEntry(p0,v,center,radius)
% First non-negative time at which constant-velocity track enters a circle.
r0=p0-center; d0=norm(r0); a=dot(v,v);
if d0 <= radius
    tEntry=0; doesEnter=true; return;
end
if a < 1e-12
    tEntry=inf; doesEnter=false; return;
end
b=2*dot(r0,v); c=dot(r0,r0)-radius^2; disc=b^2-4*a*c;
if disc < 0
    tEntry=inf; doesEnter=false; return;
end
rootsT=sort([(-b-sqrt(max(disc,0)))/(2*a),(-b+sqrt(max(disc,0)))/(2*a)]);
tEntry=inf; doesEnter=false;
for q=1:numel(rootsT)
    tq=rootsT(q);
    if tq >= 0
        rq=r0+v*tq;
        if dot(rq,v) <= 1e-8 % crossing inward or tangent
            tEntry=tq; doesEnter=true; return;
        end
    end
end
end

function [tClosest,dClosest] = closestApproach(p0,v,center)
r=p0-center; vv=dot(v,v);
if vv < 1e-12, tClosest=0; else, tClosest=max(0,-dot(r,v)/vv); end
dClosest=norm(r+v*tClosest);
end

function manifest = exportAllFiguresToFolder(outputFolder)
% Export all figures made during this run to the current MATLAB folder.
% PNG files are convenient for reports; FIG files remain editable in MATLAB.
% A timestamped current-folder name and then TEMPDir are used if a file is locked.
if nargin < 1 || isempty(outputFolder)
    outputFolder = pwd;
end
if ~exist(outputFolder,'dir')
    mkdir(outputFolder);
end
figs = findall(groot,'Type','figure');
if isempty(figs)
    manifest = table(strings(0,1),strings(0,1),strings(0,1));
    return;
end
try
    figNums = arrayfun(@(h) double(h.Number), figs);
    [~,ord] = sort(figNums);
    figs = figs(ord);
catch
    figs = flipud(figs(:));
end
nFig = numel(figs);
FigureNumber = (1:nFig)';
FigureName = strings(nFig,1);
PNGPath = strings(nFig,1);
FIGPath = strings(nFig,1);
for ii = 1:nFig
    fig = figs(ii);
    figName = get(fig,'Name');
    if isempty(figName)
        figName = sprintf('Figure_%02d',ii);
    end
    safeName = regexprep(strtrim(char(figName)),'[^A-Za-z0-9_-]+','_');
    safeName = regexprep(safeName,'_+','_');
    safeName = regexprep(safeName,'^_+|_+$','');
    if isempty(safeName), safeName = sprintf('Figure_%02d',ii); end
    if numel(safeName) > 70, safeName = safeName(1:70); end
    base = sprintf('skyforce_figure_%02d_%s',ii,safeName);
    stamp = datestr(now,'yyyymmdd_HHMMSS');
    candidates = {outputFolder, fullfile(outputFolder,[base '_' stamp]), tempdir};
    savedPNG = '';
    savedFIG = '';
    lastErr = '';
    for cc = 1:numel(candidates)
        if cc == 2
            candidateFolder = outputFolder;
            candidateBase = [base '_' stamp];
        else
            candidateFolder = candidates{cc};
            candidateBase = base;
        end
        pngPath = fullfile(candidateFolder,[candidateBase '.png']);
        figPath = fullfile(candidateFolder,[candidateBase '.fig']);
        try
            drawnow;
            print(fig,pngPath,'-dpng','-r200');
            savefig(fig,figPath);
            savedPNG = pngPath;
            savedFIG = figPath;
            break;
        catch ME
            lastErr = ME.message;
        end
    end
    FigureName(ii) = string(figName);
    PNGPath(ii) = string(savedPNG);
    FIGPath(ii) = string(savedFIG);
    if isempty(savedPNG) || isempty(savedFIG)
        warning('SKYFORCE:FigureExportFailed', ...
            'Could not export figure %d (%s). Last error: %s',ii,char(figName),lastErr);
    else
        fprintf('  Figure %02d saved: %s and %s\n',ii,savedPNG,savedFIG);
    end
end
manifest = table(FigureNumber,FigureName,PNGPath,FIGPath);
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
requestedPNDiagnostic = sqrt(sum(out.commandedAcceleration.^2,1))';
limitedPNDiagnostic = sqrt(sum(out.controllerCommandAcceleration.^2,1))';
requestedPNDiagnostic(out.phase(:) == "CAPTURE_CANDIDATE") = NaN;
limitedPNDiagnostic(out.phase(:) == "CAPTURE_CANDIDATE") = NaN;
plot(ax2,out.time,out.pnAccelerationMagnitude,'m-','LineWidth',1.35);
hold(ax2,'on');
plot(ax2,out.time,requestedPNDiagnostic,'--','Color',[0.85 0.35 0.1],'LineWidth',1.1);
plot(ax2,out.time,limitedPNDiagnostic,'Color',[0.05 0.55 0.2],'LineWidth',1.25);
plot(ax2,out.time,out.accelerationMagnitude,'b-','LineWidth',1.1);
yline(ax2,cfg.accelLimit,'r--',sprintf('%.1f m/s^2 model cap',cfg.accelLimit));
xlabel(ax2,'Time (s)'); ylabel(ax2,'Acceleration magnitude (m/s^2)');
validRequest = requestedPNDiagnostic(isfinite(requestedPNDiagnostic));
if isempty(validRequest), peakTotalRequest = 0; else, peakTotalRequest = max(validRequest); end
validLimited = limitedPNDiagnostic(isfinite(limitedPNDiagnostic));
if isempty(validLimited), peakLimitedCommand = 0; else, peakLimitedCommand = max(validLimited); end
title(ax2,sprintf('PN peak %.2f; raw demand peak %.2f; bounded command peak %.2f m/s^2', ...
    max(out.pnAccelerationMagnitude),peakTotalRequest,peakLimitedCommand),'Color','k','FontSize',8.5);
legend(ax2,{'PN component','Raw guidance demand','Controller-limited command','Applied model acceleration','Assumed acceleration cap'}, ...
    'Location','best','FontSize',7);
styleAxes(ax2);

sg = sgtitle(sprintf('PN guidance diagnostic only — %d active samples, %.2f s', ...
    sum(out.pnActive),sum(out.pnActive)*cfg.dt),'FontSize',14);
sg.Color = 'k';
annotation(fig,'textbox',[0.04 0.015 0.92 0.04], ...
    'String',sprintf('In-flight diagnostic only; velocity-servo response time %.1f s is an assumption, not hardware data. Candidate V_close %.2f m/s is kinematic only. Command is capped at %.1f m/s^2; capture physics and actuator response are not modelled.',cfg.interceptVelocityResponseTime_s,out.closingAtCapture,cfg.accelLimit), ...
    'Units','normalized','FitBoxToText','off','Color',[0.12 0.12 0.12], ...
    'BackgroundColor',[0.96 0.96 0.96],'EdgeColor',[0.6 0.6 0.6], ...
    'FontSize',8.5,'HorizontalAlignment','center','VerticalAlignment','middle', ...
    'Interpreter','none','Margin',3);
end

