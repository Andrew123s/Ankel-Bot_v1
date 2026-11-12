classdef AnkelBotTest < matlab.unittest.TestCase
    %ANKELBOTTEST Requirement-level tests on the Ankel-Bot engineering model.
    %   Run with:  results = runtests('matlab/tests')

    properties
        p
    end

    methods (TestClassSetup)
        function setup(tc)
            tc.p = ankel.params();
        end
    end

    methods (Test)
        function thrustMapIsInvertible(tc)
            for T = [-40 -12 -0.5 0 0.5 7 25 50]
                c = ankel.thrustToCmd(T, tc.p);
                if abs(c) >= tc.p.cmd_deadband
                    tc.verifyEqual(ankel.cmdToThrust(c, tc.p), T, 'AbsTol', 1e-9);
                end
            end
        end

        function straightLineSpeedReached(tc)
            r = ankel.headingStep(tc.p, 0, 0.5, 40);
            tc.verifyEqual(r.u(end), 0.5, 'AbsTol', 0.02);
            tc.verifyLessThan(max(abs(r.psi)), 1e-6);
        end

        function headingStepMeetsSpec(tc)
            r = ankel.headingStep(tc.p, deg2rad(30), 0.5);
            tc.verifyLessThan(r.overshoot_pct, 15);
            tc.verifyLessThan(r.settle_s, 8);
            tc.verifyLessThan(r.rise_s, 5);
        end

        function topSpeedIsPlausible(tc)
            x = [0;0;0;0;0;0];  T = [0 0];
            for k = 1:600
                [x, T] = ankel.plantStep(x, T, [1 1], tc.p, 0, [tc.p.wing_min tc.p.wing_min], [0 0]);
            end
            tc.verifyGreaterThan(x(4), 1.2);
            tc.verifyLessThan(x(4), 2.2);
        end

        function sizeEstimateIsExactWithoutNoise(tc)
            for c = [0 2 0.20; 0.6 4 0.45; -1.0 6 0.9; 0.2 1.6 0.1]'
                [bb, ok] = ankel.cameraProject(c(1), c(2), c(3), 0.08, tc.p, 0, -0.01);
                tc.assertTrue(ok);
                [lat, fwd, w, valid] = ankel.estimateFromBBox(bb, tc.p, 0, -0.01);
                tc.verifyTrue(valid);
                tc.verifyEqual([lat fwd], [c(1) c(2)], 'AbsTol', 0.02);
                % the top of the object is nearer the lens than its waterline, so the
                % box is a little wider than the waterline width: bounded overestimate
                tc.verifyGreaterThanOrEqual(w, c(3) - 1e-9);
                tc.verifyEqual(w, c(3), 'RelTol', 0.06);
            end
        end

        function sizeEstimateUnderRollStaysWithinTenPercent(tc)
            % An axis-aligned box around a rolled object is slightly too big:
            % a systematic error of the box-based method, bounded here.
            for c = [0 2 0.20; 0.6 4 0.45; -1.0 6 0.9]'
                bb = ankel.cameraProject(c(1), c(2), c(3), 0.08, tc.p, 0.03, 0);
                [~, ~, w] = ankel.estimateFromBBox(bb, tc.p, 0.03, 0);
                tc.verifyEqual(w, c(3), 'RelTol', 0.10);
            end
        end

        function classifierUsesMedianAndClassVotes(tc)
            tc.verifyEqual(ankel.classifyTrack([0.2 0.9 0.2 NaN], 3, 0, tc.p), 1);
            tc.verifyEqual(ankel.classifyTrack([0.5 0.6 0.2], 3, 0, tc.p), 2);
            tc.verifyEqual(ankel.classifyTrack([0.2 0.2 0.2], 3, 2, tc.p), 2);
            tc.verifyEqual(ankel.classifyTrack([0.9 0.9], 2, 2, tc.p), 0);
        end

        function calmWaterCollectsMostWaste(tc)
            m = ankel.simulate(ankel.scenario('calm'), tc.p).metrics;
            tc.verifyGreaterThan(m.collected_pct, 85);
            tc.verifyEqual(m.jams, 0);
        end

        function rejectionPreventsJams(tc)
            m = ankel.simulate(ankel.scenario('mixed'), tc.p).metrics;
            tc.verifyEqual(m.jams, 0);
            tc.verifyGreaterThan(m.collected_pct, 60);
        end

        function withoutRejectionOversizeJams(tc)
            m = ankel.simulate(ankel.scenario('mixed_no_reject'), tc.p).metrics;
            tc.verifyGreaterThan(m.jams, 0);
        end

        function lowBatteryReturnsToDock(tc)
            lg = ankel.simulate(ankel.scenario('low_battery'), tc.p);
            tc.verifyEqual(lg.metrics.final_state, 'DOCKED');
            tc.verifyLessThan(hypot(lg.x(end,1) - tc.p.sup.dock(1), lg.x(end,2) - tc.p.sup.dock(2)), 3);
        end

        function healthLossStopsThrusters(tc)
            lg = ankel.simulate(ankel.scenario('link_loss'), tc.p);
            tc.verifyEqual(lg.metrics.final_state, 'FAILSAFE');
            after = lg.t > 121;
            tc.verifyEqual(max(abs(lg.cmd(after,:)), [], 'all'), 0);
            tc.verifyEqual(min(lg.gate(lg.t > 122)), 1);
        end

        function systemBlocksMatchSimulate(tc)
            % The four model blocks (ankel.BlockCore, wrapped for Simulink by
            % +ankel/+sys) wired with the Simulink model's one-step delays must
            % reproduce ankel.simulate exactly.
            sc = ankel.scenario('mixed');  sc.T = 120;  pp = tc.p;
            ref = ankel.simulate(sc, pp);
            W = ankel.BlockCore('world', 'mixed');  Sv = ankel.BlockCore('supervisor');
            C = ankel.BlockCore('controller');  Pl = ankel.BlockCore('plant', sc.current);
            x = zeros(6,1);  ug = 0;  P = pp.P_hotel;  gate = 0;  wing = [pp.wing_nom pp.wing_nom];  info = [1 0];
            n = round(sc.T / pp.dt);  X = zeros(n, 6);
            for k = 1:n
                [tracks, flags, shift, status] = W.worldStep(x, gate, wing, info, P);
                [refs, gate, wing, info] = Sv.supervisor(x, tracks, flags, shift);
                cmd = C.controller(refs, x, ug);
                [x, ug, P] = Pl.plant(cmd, status(1), status(2:3));
                X(k,:) = x';
            end
            tc.verifyEqual(X, ref.x, 'AbsTol', 1e-9);
        end

        function driftIsEstimatedFromTracks(tc)
            m = ankel.simulate(ankel.scenario('current'), tc.p).metrics;
            tc.verifyEqual(m.drift_est, [0.02 0.06], 'AbsTol', 0.015);
        end

        function crossCurrentStillCollects(tc)
            m = ankel.simulate(ankel.scenario('current'), tc.p).metrics;
            tc.verifyEqual(m.jams, 0);
            tc.verifyGreaterThan(m.collected_pct, 60);
        end
    end
end
