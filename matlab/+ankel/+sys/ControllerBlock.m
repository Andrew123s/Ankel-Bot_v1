classdef ControllerBlock < matlab.System
    %CONTROLLERBLOCK Heading/speed controller + thrust allocation (ankel.controlLaw).
    %   Inputs : refs [psi_ref u_ref enable], x [6], ug ground speed
    %   Output : cmd [1x2] ESC commands
    properties (Nontunable)
        SampleTime = 0.1
    end
    properties (Access = private)
        core
    end
    methods (Access = protected)
        function setupImpl(o)
            o.core = ankel.BlockCore('controller');
        end
        function resetImpl(o)
            o.core = ankel.BlockCore('controller');
        end
        function cmd = stepImpl(o, refs, x, ug)
            cmd = o.core.controller(refs, x, ug);
        end
        function sts = getSampleTimeImpl(o)
            sts = createSampleTime(o, 'Type', 'Discrete', 'SampleTime', o.SampleTime);
        end
        function a = getOutputSizeImpl(~),      a = [1 2]; end
        function a = getOutputDataTypeImpl(~),  a = 'double'; end
        function a = isOutputComplexImpl(~),    a = false; end
        function a = isOutputFixedSizeImpl(~),  a = true; end
    end
end
