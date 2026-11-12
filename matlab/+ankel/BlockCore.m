classdef BlockCore < handle
    %BLOCKCORE Signal-level (vector in, vector out) versions of the four model
    %   blocks. Plain MATLAB, so they are testable without Simulink; the MATLAB
    %   System blocks in +ankel/+sys are thin wrappers around these methods.
    %
    %   kind = 'plant' | 'controller' | 'supervisor' | 'world'

    properties
        kind; p
        x; T; current = [0 0]            % plant
        cst                              % controller
        sup; rejectEnabled = true        % supervisor
        world                            % world
    end

    methods
        function o = BlockCore(kind, varargin)
            o.kind = kind;  o.p = ankel.params();
            switch kind
                case 'plant'
                    o.x = zeros(6,1);  o.T = [0 0];
                    if ~isempty(varargin), o.current = varargin{1}; end
                case 'controller'
                    o.cst = struct('iu', 0);
                case 'supervisor'
                    o.sup = ankel.supervisorInit();
                    if ~isempty(varargin), o.rejectEnabled = varargin{1}; end
                case 'world'
                    o.world = ankel.World(ankel.scenario(varargin{1}), o.p);
                otherwise
                    error('ankel:BlockCore', 'Unknown block kind %s', kind);
            end
        end

        function [x, ug, P] = plant(o, cmd, fill, wing)
            [o.x, o.T, P] = ankel.plantStep(o.x, o.T, cmd(:)', o.p, min(fill, 1), wing(:)', o.current);
            x = o.x;
            ug = x(4) + o.current(1)*cos(x(3)) + o.current(2)*sin(x(3));
        end

        function cmd = controller(o, refs, x, ug)
            [cmd, o.cst] = ankel.controlLaw(o.cst, refs(1), refs(2), x(3), ug, x(6), o.p);
            if refs(3) < 0.5, cmd = [0 0]; end
        end

        function [refs, gate, wing, info] = supervisor(o, x, tracks, flags, areaShift)
            in.x = x(1);  in.y = x(2);  in.psi = x(3);  in.tracks = tracks;
            in.area_shift = areaShift(:)';
            in.net_full = flags(1) > 0.5;  in.soc = flags(2);  in.obst_range = flags(3);
            in.mouth_blocked = flags(4) > 0.5;  in.healthy = flags(5) > 0.5;
            [o.sup, out] = ankel.supervisorStep(o.sup, in, o.p);
            if ~o.rejectEnabled, out.gate = 0; end
            refs = [out.psi_ref out.u_ref out.enable];
            gate = out.gate;  wing = out.wing;
            info = [o.sup.state o.sup.t_state];
        end

        function [tracks, flags, areaShift, status] = worldStep(o, x, gateCmd, wingCmd, info, P)
            r = o.world.step(x(:), gateCmd, wingCmd(:)', info(1), info(2), P);
            tracks = r.tracks;
            flags = [double(r.net_full) r.soc r.obst_range double(r.mouth_blocked) double(r.healthy)];
            areaShift = r.area_shift;
            status = [r.fill r.wing r.gate sum(o.world.D.st == 1) o.world.jams];
        end
    end
end
