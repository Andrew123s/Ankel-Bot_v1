function [out, m] = supOutputs(ns, tState, m, c, in, p)
%SUPOUTPUTS References and actuator commands for the active mission state.
%   out.state, out.psi_ref, out.u_ref, out.gate (1 = rake closed),
%   out.wing [port stbd] rad, out.enable (0 = thrusters must be zero).
%   m is the supervisor memory (wp, hold_psi, back_psi, side, t_clear).
q = p.sup;  S = ankel.states();  dt = p.dt;
corridor = p.hinge_x + p.wing_len*sin(p.wing_nom) + q.corridor_margin;
out.state = ns;  out.enable = 1;  out.gate = 0;
out.wing = [p.wing_nom p.wing_nom];
out.psi_ref = in.psi;  out.u_ref = 0;
switch ns
    case S.FAILSAFE
        out.enable = 0;  out.gate = 1;
    case S.BACKOFF
        out.gate = 1;
        if tState < q.t_back
            out.u_ref = q.u_back;  out.psi_ref = m.hold_psi;     % reverse straight
        else
            out.psi_ref = m.back_psi;                            % then turn away
        end
    case S.DOCKED
        out.gate = 1;  out.wing = [p.wing_min p.wing_min];
    case S.RETURN
        out.gate = 1;  out.wing = [p.wing_min p.wing_min];
        out.psi_ref = atan2(q.dock(2) - in.y, q.dock(1) - in.x);
        out.u_ref = q.u_search;
    case S.AVOID
        out.gate = 1;  out.u_ref = q.u_avoid;
        if c.threat
            rng = max(hypot(c.thr(1), c.thr(2)), 0.01);
            cl = corridor + c.thr(3)/2 + q.avoid_extra;
            off = asin(min(1, cl/rng));
            m.hold_psi = ankel.wrapPi(in.psi + atan2(c.thr(1), c.thr(2)) + m.side*off);
            m.t_clear = 0;
        else
            m.t_clear = m.t_clear + dt;
        end
        out.psi_ref = m.hold_psi;
        if m.side < 0, out.wing(2) = p.wing_min; else, out.wing(1) = p.wing_min; end
    case S.APPROACH
        brg = atan2(c.tgt(1), c.tgt(2));
        out.psi_ref = ankel.wrapPi(in.psi + brg);
        if abs(brg) < q.bearing_slow, out.u_ref = q.u_approach; else, out.u_ref = q.u_slow; end
    otherwise  % SEARCH
        wp = q.wps(m.wp, :) + in.area_shift;
        if hypot(wp(1) - in.x, wp(2) - in.y) < q.wp_radius
            m.wp = mod(m.wp, size(q.wps, 1)) + 1;
            wp = q.wps(m.wp, :) + in.area_shift;
        end
        out.psi_ref = atan2(wp(2) - in.y, wp(1) - in.x);
        out.u_ref = q.u_search;
end
end
