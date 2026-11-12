function p = params()
%PARAMS Single source of truth for Ankel-Bot physical and control parameters.
%   Exported to config/ankelbot_params.json by export_reference.m and read by
%   the onboard Python software, so simulation and vessel use the same numbers.
%
%   Frames: world x (north), y (east), heading psi clockwise from x.
%   Body: forward, lateral positive to starboard. Port thruster at -b.
%
%   Hydrodynamic values are engineering ESTIMATES for the 1.62 m catamaran
%   and must be replaced by tow-test / bollard-pull measurements.

p.dt = 0.1;                      % control period [s]
p.dt_plant = 0.02;               % plant integration step [s]

% --- geometry ---
p.L = 1.62;  p.B = 1.07;
p.b = 0.40;                      % thruster lateral offset from centreline [m]
p.mouth_w = 0.52;                % tunnel mouth width [m]
p.pass_w = 0.42;                 % widest object that physically passes gate + net mouth [m]
p.hinge_x = 0.30;                % wing hinge lateral offset [m]
p.hinge_fwd = 0.81;              % wing hinge forward of origin [m]
p.wing_len = 0.80;
p.wing_min = deg2rad(20); p.wing_max = deg2rad(60); p.wing_nom = deg2rad(40);
p.wing_rate = deg2rad(15);       % actuator slew [rad/s]
p.gate_time = 0.8;               % reject rake open<->closed [s]
p.V_net = 0.070;                 % rear net volume [m^3]
p.pack = 4.0;                    % loose-packing factor for net fill

% --- rigid body + added mass ---
p.m = 75;  p.Iz = 22;
p.Xud = 8;  p.Yvd = 55;  p.Nrd = 6;

% --- damping ---
p.Xu = 10;  p.Xuu = 30;          % bare hulls
p.Xuu_wing = 10;                 % both wings at nominal sweep
p.Xuu_net = 15;                  % empty net
p.net_fill_drag = 2.0;           % net drag multiplier slope with fill
p.Xuu_nom = p.Xuu + p.Xuu_wing + p.Xuu_net;
p.Yv = 40;  p.Yvv = 200;
p.Nr = 8;   p.Nrr = 12;

% --- thrusters / power ---
p.Tmax = 50;                     % forward bollard thrust per thruster [N]
p.rev = 0.8;                     % reverse thrust fraction
p.tau_thr = 0.25;                % ESC + motor lag [s]
p.cmd_deadband = 0.05;
p.Pmax = 350;                    % electrical power per thruster at full command [W]
p.P_hotel = 25;                  % computer, sensors, radios [W]
p.E_batt = 2048;                 % usable energy [Wh]

% --- camera (pinhole) ---
p.cam.h = 0.55;                  % lens height above water [m]
p.cam.fwd = 0.60;                % lens forward of origin [m]
p.cam.pitch = deg2rad(13);       % optical axis below horizontal [rad]
p.cam.W = 1280;  p.cam.H = 720;
p.cam.hfov = deg2rad(79);
p.cam.fx = (p.cam.W/2) / tan(p.cam.hfov/2);
p.cam.fy = p.cam.fx;
p.cam.cx = p.cam.W/2;  p.cam.cy = p.cam.H/2;

% --- detector model used in simulation ---
p.det.min_px = 14;               % smallest usable box width [px]
p.det.max_range = 8;             % [m]
p.det.sigma_px = 2.0;            % box corner noise [px]
p.det.p_detect = 0.9;            % per-frame detection probability
p.det.p_class = 0.85;            % per-frame prob. an oversize class is recognised
p.det.sigma_att = deg2rad(0.7);  % residual roll/pitch error after IMU compensation

% --- decision ---
p.dec.w_max = 0.35;              % widest object accepted by policy [m]
p.dec.min_obs = 3;               % observations before a track is classified
p.dec.buf = 9;                   % width median window
p.dec.t_keep = 8.0;              % track memory without detections [s]
p.dec.drift_min_age = 2.0;       % track age before it contributes to the drift estimate [s]
p.dec.drift_alpha = 0.05;        % low-pass factor of the common drift estimate
p.dec.drift_max = 0.5;           % clamp on estimated surface drift [m/s]

% --- supervisor ---
q.u_search = 0.5;  q.u_approach = 0.5;  q.u_slow = 0.3;  q.u_avoid = 0.35;  q.u_back = -0.3;
q.bearing_slow = 0.5;
q.r_avoid = 4.0;  q.r_gate = 2.5;
q.corridor_margin = 0.30;  q.hold_margin = 0.6;  q.avoid_extra = 0.2;
q.funnel_fwd = 1.5;  q.funnel_lat = 0.45;
q.t_back = 3.0;  q.t_turn = 3.0;  q.back_turn = deg2rad(40);
q.near_fwd = 2.0;  q.t_over = 3.0;
q.t_clear = 2.0;
q.t_block = 2.0;                  % mouth blocked this long with the rake open -> back off
q.d_stop = 0.8;
q.soc_return = 20;  q.soc_crit = 8;
q.dock = [-2 0];  q.dock_radius = 1.5;  q.t_swap = 20;
q.wp_radius = 1.0;
q.area = [0 16 -5 5];            % search rectangle [xmin xmax ymin ymax]
half = p.hinge_x + p.wing_len*sin(p.wing_nom);
q.lane = 2*half*0.85;            % lane spacing with 15 % overlap
q.wps = ankel.lawnmower(q.area, q.lane);
p.sup = q;

p.gains = ankel.designGains(p);
end
