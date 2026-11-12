classdef SizeBlock < matlab.System
    %SIZEBLOCK Detector boxes -> position, width and accept/reject verdict.
    %   Inputs : bboxes [N x 4] (variable size, [x y w h] frame pixels), scores [N x 1]
    %   Output : meas [8 x 6] rows [valid lat fwd width verdict score],
    %            verdict 1 = collect, 2 = reject (too wide), strongest first.
    %   Uses ankel.measureBoxes -> ankel.estimateFromBBox, the same code as the
    %   simulation and the onboard software.
    properties (Nontunable)
        Rows = 180
        Cols = 320
        SampleTime = 0.1
    end
    properties (Access = private)
        p
    end
    methods (Access = protected)
        function setupImpl(o)
            o.p = ankel.params();
        end
        function meas = stepImpl(o, bboxes, scores)
            meas = zeros(8, 6);
            bb = double(bboxes);  sc = double(scores(:));
            keep = all(bb(:, 3:4) > 0, 2);                       % unused rows are zero
            bb = bb(keep, :);  sc = sc(keep);
            [~, order] = sort(sc, 'descend');
            n = min(8, numel(order));
            M = ankel.measureBoxes(bb(order(1:n), :), [o.Rows o.Cols], o.p);
            for i = 1:n
                verdict = 1 + (M(i,3) > o.p.dec.w_max);
                meas(i,:) = [M(i,4) M(i,1) M(i,2) M(i,3) verdict sc(order(i))];
            end
        end
        function sts = getSampleTimeImpl(o)
            sts = createSampleTime(o, 'Type', 'Discrete', 'SampleTime', o.SampleTime);
        end
        function flag = isInputSizeMutableImpl(~, ~), flag = true; end
        function a = getOutputSizeImpl(~),      a = [8 6]; end
        function a = getOutputDataTypeImpl(~),  a = 'double'; end
        function a = isOutputComplexImpl(~),    a = false; end
        function a = isOutputFixedSizeImpl(~),  a = true; end
    end
end
