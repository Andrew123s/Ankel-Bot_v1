classdef PlantBlock < matlab.System
    %PLANTBLOCK Vessel + thruster dynamics as a MATLAB System block.
    %   Inputs : cmd [2] ESC commands, fill (0..1), wing [2] sweep angles
    %   Outputs: x [6x1] state, ug ground speed along heading, P power [W]
    properties (Nontunable)
        Current = [0 0]      % water current, world frame [m/s]
        SampleTime = 0.1
    end
    properties (Access = private)
        core
    end
    methods (Access = protected)
        function setupImpl(o)
            o.core = ankel.BlockCore('plant', o.Current);
        end
        function resetImpl(o)
            o.core = ankel.BlockCore('plant', o.Current);
        end
        function [x, ug, P] = stepImpl(o, cmd, fill, wing)
            [x, ug, P] = o.core.plant(cmd, fill, wing);
        end
        function sts = getSampleTimeImpl(o)
            sts = createSampleTime(o, 'Type', 'Discrete', 'SampleTime', o.SampleTime);
        end
        function [a, b, c] = getOutputSizeImpl(~),      a = [6 1]; b = [1 1]; c = [1 1]; end
        function [a, b, c] = getOutputDataTypeImpl(~),  a = 'double'; b = 'double'; c = 'double'; end
        function [a, b, c] = isOutputComplexImpl(~),    a = false; b = false; c = false; end
        function [a, b, c] = isOutputFixedSizeImpl(~),  a = true; b = true; c = true; end
    end
end
