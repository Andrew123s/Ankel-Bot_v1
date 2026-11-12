classdef WorldBlock < matlab.System
    %WORLDBLOCK Debris field, camera/detector/tracker, actuators, collection
    %   physics and battery (ankel.World) as a MATLAB System block.
    %   Inputs : x [6], gateCmd, wingCmd [2], info [state t_state], P [W]
    %   Outputs: tracks [8x5], flags [net_full soc obst_range mouth_blocked healthy],
    %            area_shift [1x2], status [fill wingP wingS gate collected jams]
    properties (Nontunable)
        Scenario = 'mixed'
        SampleTime = 0.1
    end
    properties (Access = private)
        core
    end
    methods (Access = protected)
        function setupImpl(o)
            o.core = ankel.BlockCore('world', o.Scenario);
        end
        function resetImpl(o)
            o.core = ankel.BlockCore('world', o.Scenario);
        end
        function [tracks, flags, areaShift, status] = stepImpl(o, x, gateCmd, wingCmd, info, P)
            [tracks, flags, areaShift, status] = o.core.worldStep(x, gateCmd, wingCmd, info, P);
        end
        function sts = getSampleTimeImpl(o)
            sts = createSampleTime(o, 'Type', 'Discrete', 'SampleTime', o.SampleTime);
        end
        function [a, b, c, d] = getOutputSizeImpl(~),      a = [8 5]; b = [1 5]; c = [1 2]; d = [1 6]; end
        function [a, b, c, d] = getOutputDataTypeImpl(~),  a = 'double'; b = 'double'; c = 'double'; d = 'double'; end
        function [a, b, c, d] = isOutputComplexImpl(~),    a = false; b = false; c = false; d = false; end
        function [a, b, c, d] = isOutputFixedSizeImpl(~),  a = true; b = true; c = true; d = true; end
    end
end
