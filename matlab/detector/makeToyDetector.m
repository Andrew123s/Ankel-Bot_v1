function detector = makeToyDetector(outFile)
%MAKETOYDETECTOR Train a tiny detector on synthetic bright rectangles.
%
%   This is a WIRING FIXTURE, not a waste detector. It exists so that the
%   Simulink perception model (Deep Learning Object Detector block ->
%   size estimation -> accept/reject) can be built and run before a real
%   model has been trained. It learns to find light rectangles on a dark
%   background, which is all the synthetic test video contains.
%   Trains on the CPU in about a minute. Saves variable "detector".
if nargin < 1, outFile = fullfile(fileparts(mfilename('fullpath')), 'toyDetector.mat'); end
rng(11);
N = 240;  S = 128;
d = tempname;  mkdir(d);
files = strings(N, 1);  boxes = cell(N, 1);  labels = cell(N, 1);
for i = 1:N
    I = uint8(50 + 25*rand(S, S, 3));
    w = 14 + randi(40);  h = 8 + randi(20);  x = randi(S - w);  y = randi(S - h);
    I(y:y+h-1, x:x+w-1, :) = uint8(200 + 40*rand);
    files(i) = fullfile(d, sprintf('t%03d.png', i));  imwrite(I, files(i));
    boxes{i} = [x y w h];  labels{i} = categorical("waste", "waste");
end
T = table(boxes, labels);
ds = combine(imageDatastore(files), boxLabelDatastore(T));
anchors = estimateAnchorBoxes(boxLabelDatastore(T), 3);

layers = [
    imageInputLayer([S S 3], 'Name', 'in', 'Normalization', 'rescale-zero-one')
    convolution2dLayer(3, 16, 'Padding', 'same', 'Name', 'c1'); batchNormalizationLayer('Name', 'b1'); reluLayer('Name', 'r1')
    maxPooling2dLayer(2, 'Stride', 2, 'Name', 'p1')
    convolution2dLayer(3, 32, 'Padding', 'same', 'Name', 'c2'); batchNormalizationLayer('Name', 'b2'); reluLayer('Name', 'r2')
    maxPooling2dLayer(2, 'Stride', 2, 'Name', 'p2')
    convolution2dLayer(3, 64, 'Padding', 'same', 'Name', 'c3'); batchNormalizationLayer('Name', 'b3'); reluLayer('Name', 'feat')
    maxPooling2dLayer(2, 'Stride', 2, 'Name', 'p3')
    convolution2dLayer(3, 64, 'Padding', 'same', 'Name', 'c4'); batchNormalizationLayer('Name', 'b4'); reluLayer('Name', 'feat2')];
net = dlnetwork(layers);
detector = yolov2ObjectDetector(net, "waste", anchors, 'DetectionNetworkSource', 'feat2');

opts = trainingOptions('adam', 'InitialLearnRate', 2e-3, 'MiniBatchSize', 16, 'MaxEpochs', 25, ...
    'Shuffle', 'every-epoch', 'Verbose', false, 'Plots', 'none', 'ExecutionEnvironment', 'cpu');
detector = trainYOLOv2ObjectDetector(ds, detector, opts);
save(outFile, 'detector');
rmdir(d, 's');
end
