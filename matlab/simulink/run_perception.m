function r = run_perception(detectorFile)
%RUN_PERCEPTION Build and run the Simulink perception model AnkelPerception.slx.
%
%       Camera (synthetic) -> Deep Learning Object Detector -> Size estimation
%
%   detectorFile  MAT-file holding a variable "detector". Defaults to the toy
%                 fixture matlab/detector/toyDetector.mat (created on demand),
%                 which only finds bright rectangles. Pass the trained
%                 wasteDetector.mat to run the real model through the same wiring.
%
%   The synthetic camera draws two objects whose true position and width are
%   known, so the result is checked: r.width_err is the relative width error
%   per object, r.verdict_ok the fraction of frames with the right verdict.
here = fileparts(mfilename('fullpath'));  root = fileparts(here);
addpath(root, fullfile(root, 'detector'));
if nargin < 1
    detectorFile = fullfile(root, 'detector', 'toyDetector.mat');
    if ~isfile(detectorFile), makeToyDetector(detectorFile); end
end
p = ankel.params();
mdl = 'AnkelPerception';  T = 3;
if bdIsLoaded(mdl), close_system(mdl, 0); end
if isfile(fullfile(here, [mdl '.slx'])), delete(fullfile(here, [mdl '.slx'])); end
new_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.1', 'StopTime', num2str(T));

msys = 'simulink/User-Defined Functions/MATLAB System';
add_block(msys, [mdl '/Camera'], 'System', 'ankel.sys.CameraBlock', 'SimulateUsing', 'Interpreted execution', 'Position', [60 100 200 180]);
load_system('visionanalysis');
add_block('visionanalysis/Deep Learning Object Detector', [mdl '/Detector'], 'Position', [290 90 470 190], ...
    'Detector', 'Detector from MAT-file', 'DetectorFilePath', detectorFile, ...
    'Bboxes', 'on', 'Scores', 'on', 'Labels', 'off', 'Threshold', '0.4', 'MaxDetections', '8', ...
    'SimulateUsing', 'Interpreted execution');
add_block(msys, [mdl '/SizeEstimate'], 'System', 'ankel.sys.SizeBlock', 'SimulateUsing', 'Interpreted execution', 'Position', [560 100 720 180]);
add_block('simulink/Sinks/To Workspace', [mdl '/log_meas'], 'VariableName', 'pc_meas', 'SaveFormat', 'Array', 'Position', [800 125 880 155]);
add_line(mdl, 'Camera/1', 'Detector/1', 'autorouting', 'smart');
add_line(mdl, 'Detector/1', 'SizeEstimate/1', 'autorouting', 'smart');
add_line(mdl, 'Detector/2', 'SizeEstimate/2', 'autorouting', 'smart');
add_line(mdl, 'SizeEstimate/1', 'log_meas/1', 'autorouting', 'smart');
save_system(mdl, fullfile(here, [mdl '.slx']));

out = sim(mdl);
M = out.pc_meas;                                   % 8 x 6 x steps
if size(M, 1) ~= 8, M = permute(M, [2 3 1]); end
nSteps = size(M, 3);

% ---- compare with the truth the camera drew ----
cam = ankel.sys.CameraBlock;  objs0 = cam.Objects;  speed = cam.Speed;
nObj = size(objs0, 1);  werr = nan(nSteps, nObj);  vok = false(nSteps, nObj);  found = false(nSteps, nObj);
for k = 1:nSteps
    objs = objs0;  objs(:,2) = objs(:,2) - speed*(k-1)*0.1;
    m = M(:,:,k);  m = m(m(:,1) > 0.5, :);
    for i = 1:nObj
        if isempty(m), continue; end
        [d, j] = min(hypot(m(:,2) - objs(i,1), m(:,3) - objs(i,2)));
        if d > 0.5, continue; end
        found(k,i) = true;
        werr(k,i) = abs(m(j,4) - objs(i,3)) / objs(i,3);
        vok(k,i) = m(j,5) == 1 + (objs(i,3) > p.dec.w_max);
    end
end
r.detector = detectorFile;
r.steps = nSteps;
r.found = mean(found, 1);                          % fraction of frames each object was detected
r.width_err = mean(werr, 1, 'omitnan');            % mean relative width error per object
r.verdict_ok = sum(vok, 1) ./ max(1, sum(found, 1));
r.true_width = objs0(:,3)';
fprintf('Perception model: %d frames. Detected %s of frames, width error %s, verdict correct %s\n', ...
    nSteps, mat2str(round(r.found, 2)), mat2str(round(r.width_err, 3)), mat2str(round(r.verdict_ok, 2)));

img = fullfile(fileparts(root), 'docs', 'img');
if isfolder(img)
    print(['-s' mdl], '-dpng', '-r110', fullfile(img, 'simulink_perception.png'));
end
close_system(mdl, 0);
end
