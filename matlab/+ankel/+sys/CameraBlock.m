classdef CameraBlock < matlab.System
    %CAMERABLOCK Synthetic camera: renders floating objects as bright boxes.
    %   A stand-in for the real camera so the perception chain can be simulated
    %   with images. Objects are drawn where ankel.cameraProject puts them, so
    %   the truth (position, width) of everything in the picture is known.
    %   Output: image [Rows x Cols x 3] uint8.
    properties (Nontunable)
        Objects = [0 1.6 0.30 0.12; 0.7 2.5 0.80 0.20]   % rows [lat fwd width height] at t = 0
        Speed = 0.05          % objects approach at this speed [m/s]
        Rows = 180
        Cols = 320
        SampleTime = 0.1
    end
    properties (Access = private)
        p; k
    end
    methods (Static)
        function I = render(objs, p, rows, cols)
            I = uint8(55 * ones(rows, cols, 3));
            I(1:round(rows*0.3), :, :) = 80;                    % sky band above the horizon
            sx = cols / p.cam.W;  sy = rows / p.cam.H;
            for i = 1:size(objs, 1)
                [b, ok] = ankel.cameraProject(objs(i,1), objs(i,2), objs(i,3), objs(i,4), p, 0, 0);
                if ~ok, continue; end
                c1 = max(1, round(b(1)*sx) + 1);  c2 = min(cols, round(b(3)*sx));
                r1 = max(1, round(b(2)*sy) + 1);  r2 = min(rows, round(b(4)*sy));
                I(r1:r2, c1:c2, :) = 215;
            end
        end
    end
    methods (Access = protected)
        function setupImpl(o)
            o.p = ankel.params();  o.k = 0;
        end
        function resetImpl(o)
            o.k = 0;
        end
        function I = stepImpl(o)
            objs = o.Objects;
            objs(:,2) = objs(:,2) - o.Speed * o.k * o.SampleTime;
            I = ankel.sys.CameraBlock.render(objs, o.p, o.Rows, o.Cols);
            o.k = o.k + 1;
        end
        function n = getNumInputsImpl(~), n = 0; end
        function sts = getSampleTimeImpl(o)
            sts = createSampleTime(o, 'Type', 'Discrete', 'SampleTime', o.SampleTime);
        end
        function a = getOutputSizeImpl(o),      a = [o.Rows o.Cols 3]; end
        function a = getOutputDataTypeImpl(~),  a = 'uint8'; end
        function a = isOutputComplexImpl(~),    a = false; end
        function a = isOutputFixedSizeImpl(~),  a = true; end
    end
end
