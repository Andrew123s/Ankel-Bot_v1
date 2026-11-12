function T = detectOnVideo(videoFile, det, varargin)
%DETECTONVIDEO Detector -> size estimation -> accept/reject on a video, in MATLAB.
%
%   T = detectOnVideo('clip.mp4', detector)                 detector object with detect()
%   T = detectOnVideo('clip.mp4', @(frame) myDetect(frame)) function returning
%                                                           [bboxes(x y w h), scores, labels]
%   Options: 'MaxFrames', 'Threshold' (0.35), 'OutputVideo' (annotated .mp4/.avi),
%            'RejectClasses' (string array), 'Params' (ankel.params())
%
%   Each detection box is passed to ankel.estimateFromBBox (the same function
%   the simulation and the onboard Python port use) to get its position on the
%   water and its width, then labelled with the accept/reject policy:
%       collect   width <= p.dec.w_max and class is not a reject class
%       REJECT    otherwise
%   The verdict here is per frame; the vessel additionally tracks objects
%   over time (ankel.classifyTrack) before acting.
%
%   T: frame, label, score, u1 v1 u2 v2 (calibration pixels), lat, fwd, width,
%      valid, truncated, verdict
o = inputParser;
o.addParameter('MaxFrames', inf);
o.addParameter('Threshold', 0.35);
o.addParameter('OutputVideo', '');
o.addParameter('RejectClasses', ["branch" "log" "tyre" "vegetation_mat" "person" "boat" "bird" "animal"]);
o.addParameter('Params', []);
o.parse(varargin{:});  o = o.Results;
p = o.Params;  if isempty(p), p = ankel.params(); end

vr = VideoReader(videoFile);
vw = [];
if ~isempty(o.OutputVideo)
    [~, ~, ext] = fileparts(o.OutputVideo);
    if strcmpi(ext, '.mp4'), vw = VideoWriter(o.OutputVideo, 'MPEG-4'); else, vw = VideoWriter(o.OutputVideo); end
    vw.FrameRate = vr.FrameRate;  open(vw);
end

rows = cell(0, 13);  k = 0;
while hasFrame(vr) && k < o.MaxFrames
    frame = readFrame(vr);  k = k + 1;
    if isa(det, 'function_handle')
        [bb, sc, lb] = det(frame);
    else
        [bb, sc, lb] = detect(det, frame, 'Threshold', o.Threshold);
    end
    lb = string(lb);
    M = ankel.measureBoxes(bb, [size(frame, 1) size(frame, 2)], p);
    ann = strings(size(bb, 1), 1);  col = strings(size(bb, 1), 1);
    for i = 1:size(bb, 1)
        lat = M(i,1);  fwd = M(i,2);  w = M(i,3);  valid = M(i,4) > 0.5;  trunc = M(i,5) > 0.5;  box = M(i,6:9);
        if ~valid
            verdict = "above horizon";
        elseif any(lb(i) == o.RejectClasses) || w > p.dec.w_max
            verdict = "REJECT";
        else
            verdict = "collect";
        end
        rows(end+1, :) = {k, lb(i), sc(i), box(1), box(2), box(3), box(4), lat, fwd, w, valid, trunc, verdict}; %#ok<AGROW>
        ann(i) = sprintf('%s %.2f | %.2f m wide @ %.1f m | %s', lb(i), sc(i), w, hypot(lat, fwd), verdict);
        if verdict == "collect", col(i) = "green"; else, col(i) = "red"; end
    end
    if ~isempty(vw)
        if ~isempty(bb)
            frame = insertObjectAnnotation(frame, 'rectangle', bb, cellstr(ann), 'Color', cellstr(col), 'LineWidth', 2);
        end
        writeVideo(vw, frame);
    end
end
if ~isempty(vw), close(vw); end
T = cell2table(rows, 'VariableNames', {'frame', 'label', 'score', 'u1', 'v1', 'u2', 'v2', 'lat', 'fwd', 'width', 'valid', 'truncated', 'verdict'});
end
