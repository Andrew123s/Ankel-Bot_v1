classdef SupervisorBlock < matlab.System
    %SUPERVISORBLOCK Mission state machine (ankel.supervisorStep).
    %   Inputs : x [6], tracks [8x5], flags [net_full soc obst_range mouth_blocked healthy],
    %            area_shift [1x2]
    %   Outputs: refs [psi_ref u_ref enable], gate, wing [1x2], info [state t_state]
    properties (Nontunable)
        RejectEnabled (1,1) logical = true
        SampleTime = 0.1
    end
    properties (Access = private)
        core
    end
    methods (Access = protected)
        function setupImpl(o)
            o.core = ankel.BlockCore('supervisor', o.RejectEnabled);
        end
        function resetImpl(o)
            o.core = ankel.BlockCore('supervisor', o.RejectEnabled);
        end
        function [refs, gate, wing, info] = stepImpl(o, x, tracks, flags, areaShift)
            [refs, gate, wing, info] = o.core.supervisor(x, tracks, flags, areaShift);
        end
        function sts = getSampleTimeImpl(o)
            sts = createSampleTime(o, 'Type', 'Discrete', 'SampleTime', o.SampleTime);
        end
        function [a, b, c, d] = getOutputSizeImpl(~),      a = [1 3]; b = [1 1]; c = [1 2]; d = [1 2]; end
        function [a, b, c, d] = getOutputDataTypeImpl(~),  a = 'double'; b = 'double'; c = 'double'; d = 'double'; end
        function [a, b, c, d] = isOutputComplexImpl(~),    a = false; b = false; c = false; d = false; end
        function [a, b, c, d] = isOutputFixedSizeImpl(~),  a = true; b = true; c = true; d = true; end
    end
end
