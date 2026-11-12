classdef DetectorPathTest < matlab.unittest.TestCase
    %DETECTORPATHTEST Tests of the MATLAB detector data path and video pipeline.
    %   These use a small synthetic dataset and a stand-in detector function.
    %   They do NOT train or run a neural network.

    properties
        root
    end

    methods (TestClassSetup)
        function makeDataset(tc)
            tc.assumeTrue(exist('boxLabelDatastore') > 0, 'Computer Vision Toolbox not installed'); %#ok<EXIST>
            addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'detector'));
            tc.root = tempname;  rng(3);
            for split = ["train" "val"]
                mkdir(fullfile(tc.root, 'images', split));  mkdir(fullfile(tc.root, 'labels', split));
                for i = 1:8
                    I = uint8(60 + 20*rand(240, 320, 3));
                    n = randi(3);  L = zeros(n, 5);
                    for j = 1:n
                        w = 20 + randi(60);  h = 10 + randi(30);  x = randi(320 - w);  y = randi(240 - h);
                        I(y:y+h-1, x:x+w-1, :) = 200;
                        L(j, :) = [randi(2) - 1, (x - 1 + w/2)/320, (y - 1 + h/2)/240, w/320, h/240];
                    end
                    imwrite(I, fullfile(tc.root, 'images', split, sprintf('im%02d.png', i)));
                    writematrix(L, fullfile(tc.root, 'labels', split, sprintf('im%02d.txt', i)), 'Delimiter', ' ');
                end
            end
            fid = fopen(fullfile(tc.root, 'data.yaml'), 'w');
            fprintf(fid, 'path: x\ntrain: images/train\nval: images/val\nnames:\n  0: bottle\n  1: branch\n');
            fclose(fid);
        end
    end

    methods (TestClassTeardown)
        function cleanup(tc)
            if ~isempty(tc.root) && isfolder(tc.root), rmdir(tc.root, 's'); end
        end
    end

    methods (Test)
        function loaderConvertsYoloLabelsExactly(tc)
            [ds, classes, T] = loadYoloDataset(tc.root, 'train');
            tc.verifyEqual(classes, ["bottle"; "branch"]);
            tc.verifyEqual(height(T), 8);
            L = readmatrix(fullfile(tc.root, 'labels', 'train', 'im01.txt'), 'FileType', 'text');
            L = reshape(L, [], 5);
            b = T.boxes{1};
            tc.verifyEqual(b(:,3), L(:,4)*320, 'AbsTol', 1e-6);
            tc.verifyEqual(b(:,1) - 1 + b(:,3)/2, L(:,2)*320, 'AbsTol', 1e-6);
            tc.verifyEqual(b(:,2) - 1 + b(:,4)/2, L(:,3)*240, 'AbsTol', 1e-6);
            d = read(ds);
            tc.verifySize(d{1}, [240 320 3]);
            tc.verifyEqual(d{2}, b);
            tc.verifyEqual(string(d{3}), classes(L(:,1) + 1));
            % the labelled pixels really are the bright rectangle
            I = d{1};  r = round(b(1,:));
            tc.verifyEqual(I(r(2) + 1, r(1) + 1, 1), uint8(200));
        end

        function anchorsAreScaledAndSorted(tc)
            ds = loadYoloDataset(tc.root, 'train');
            a = wasteAnchors(ds, [416 416 3], 6);
            tc.verifyEqual(size(a, 2), 2);
            tc.verifyLessThanOrEqual(size(a, 1), 6);
            tc.verifyTrue(issorted(prod(a, 2), 'descend'));
            tc.verifyLessThanOrEqual(max(a(:)), 416);
        end

        function augmentKeepsBoxesOnObjects(tc)
            ds = loadYoloDataset(tc.root, 'train');
            d = read(ds);
            rng(1);
            for k = 1:6
                out = wasteAugment(d);
                b = round(out{2}(1,:));  I = out{1};
                inside = mean(I(b(2)+1:b(2)+b(4)-2, b(1)+1:b(1)+b(3)-2, 1), 'all');
                outside = mean(I(:,:,1), 'all');
                tc.verifyGreaterThan(inside, outside + 30);      % box still covers the bright object
                tc.verifyEqual(out{3}, d{3});
            end
        end

        function simulinkPerceptionModelRuns(tc)
            % Camera -> Deep Learning Object Detector block -> size estimation, in Simulink,
            % with the toy fixture detector (bright rectangles only).
            tc.assumeTrue(exist('new_system') > 0, 'Simulink not installed'); %#ok<EXIST>
            addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'simulink'));
            r = run_perception();
            tc.verifyGreaterThan(r.found, [0.9 0.9]);
            tc.verifyLessThan(r.width_err, [0.2 0.1]);
            tc.verifyEqual(r.verdict_ok(2), 1);          % the 0.8 m object is always rejected
        end

        function videoPipelineEstimatesSizeAndVerdict(tc)
            % Synthetic clip + stand-in detector returning boxes projected from known objects.
            p = ankel.params();
            objs = [0.0 3.0 0.20 0.08; -0.8 5.0 0.90 0.20];      % [lat fwd width height]
            vf = [tempname '.avi'];
            vw = VideoWriter(vf);  open(vw);
            for k = 1:5, writeVideo(vw, uint8(80*ones(360, 640, 3))); end
            close(vw);
            stub = @(frame) projectObjects(objs, p, size(frame));
            T = detectOnVideo(vf, stub, 'Params', p, 'OutputVideo', [tempname '.avi']);
            delete(vf);
            tc.verifyEqual(height(T), 10);
            a = T(T.label == "waste", :);  b = T(T.label == "other_waste", :);
            tc.verifyEqual(a.width, repmat(0.20, 5, 1), 'RelTol', 0.06);
            tc.verifyEqual(a.fwd, repmat(3.0, 5, 1), 'AbsTol', 0.05);
            tc.verifyEqual(a.verdict, repmat("collect", 5, 1));
            tc.verifyEqual(b.width, repmat(0.90, 5, 1), 'RelTol', 0.06);
            tc.verifyEqual(b.verdict, repmat("REJECT", 5, 1));   % too wide, whatever its class
        end
    end
end

function [bb, sc, lb] = projectObjects(objs, p, frameSize)
% Boxes a perfect detector would return for known objects, in frame pixels [x y w h].
n = size(objs, 1);  bb = zeros(n, 4);  sc = 0.9*ones(n, 1);  lb = ["waste"; "other_waste"];
sx = frameSize(2) / p.cam.W;  sy = frameSize(1) / p.cam.H;
for i = 1:n
    b = ankel.cameraProject(objs(i,1), objs(i,2), objs(i,3), objs(i,4), p, 0, 0);
    bb(i,:) = [b(1)*sx + 1, b(2)*sy + 1, (b(3) - b(1))*sx, (b(4) - b(2))*sy];
end
end
