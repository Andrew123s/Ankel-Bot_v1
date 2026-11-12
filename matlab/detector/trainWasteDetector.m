function [detector, metrics] = trainWasteDetector(dataRoot, varargin)
%TRAINWASTEDETECTOR Transfer-learn a YOLO v4 tiny detector on floating waste.
%
%   [detector, metrics] = trainWasteDetector('onboard/datasets/floating_waste')
%   ... = trainWasteDetector(root, 'Epochs', 80, 'InputSize', [416 416 3], 'MiniBatchSize', 8)
%
%   Starts from the COCO-pretrained 'tiny-yolov4-coco' network and retrains it
%   on the classes in data.yaml. Saves matlab/detector/wasteDetector.mat, which
%   detectOnVideo and the Simulink Deep Learning Object Detector block load.
%
%   REQUIRES
%     - Computer Vision Toolbox, Deep Learning Toolbox
%     - Computer Vision Toolbox Model for YOLO v4 Object Detection (support package)
%     - Parallel Computing Toolbox + a CUDA GPU is strongly advised; on a CPU
%       a few thousand images take many hours per run.
%
%   STATUS: the data path (loader, anchors, augmentation, preprocessing) is
%   tested in MATLAB. The training and evaluation calls below have NOT been
%   run: the YOLO v4 support package was not installed on the development
%   machine and no floating-waste dataset has been assembled.
o = inputParser;
o.addParameter('Epochs', 80);
o.addParameter('InputSize', [416 416 3]);
o.addParameter('MiniBatchSize', 8);
o.addParameter('LearnRate', 1e-3);
o.parse(varargin{:});  o = o.Results;

[dsTrain, classes] = loadYoloDataset(dataRoot, 'train');
[dsVal, ~, Tval]   = loadYoloDataset(dataRoot, 'val');

% Anchor boxes from the training labels at network resolution. Tiny YOLO v4
% has two detection heads: the first takes the three largest anchors.
anchors = wasteAnchors(dsTrain, o.InputSize, 6);
anchorBoxes = {anchors(1:3, :); anchors(4:6, :)};

detector = yolov4ObjectDetector('tiny-yolov4-coco', classes, anchorBoxes, 'InputSize', o.InputSize);

opts = trainingOptions('adam', ...
    'InitialLearnRate', o.LearnRate, 'MiniBatchSize', o.MiniBatchSize, 'MaxEpochs', o.Epochs, ...
    'Shuffle', 'every-epoch', 'ValidationData', dsVal, 'VerboseFrequency', 20, ...
    'L2Regularization', 5e-4, 'ResetInputNormalization', false, ...
    'OutputNetwork', 'best-validation-loss', 'Plots', 'none');
detector = trainYOLOv4ObjectDetector(transform(dsTrain, @wasteAugment), detector, opts);

% Evaluate on the validation split
results = detect(detector, imageDatastore(Tval.imageFilename), 'Threshold', 0.01, 'MiniBatchSize', o.MiniBatchSize);
metrics = evaluateObjectDetection(results, dsVal);

here = fileparts(mfilename('fullpath'));
save(fullfile(here, 'wasteDetector.mat'), 'detector', 'classes');
fprintf('Saved %s\n', fullfile(here, 'wasteDetector.mat'));
disp(metrics.ClassMetrics);
end
