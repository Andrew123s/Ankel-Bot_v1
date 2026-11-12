function [ds, classes, T] = loadYoloDataset(root, split)
%LOADYOLODATASET Read an Ultralytics-format detection dataset for MATLAB training.
%
%   [ds, classes, T] = loadYoloDataset('onboard/datasets/floating_waste', 'train')
%
%   Layout (the same folder the YOLO26 training in onboard/tools uses, so one
%   labelled dataset serves both detectors):
%       root/data.yaml                 class names ("  0: bottle" lines)
%       root/images/<split>/*.jpg|png
%       root/labels/<split>/*.txt      "class cx cy w h", normalised 0..1
%
%   ds       combined datastore {image, boxes [x y w h] px, labels} for
%            trainYOLOv4ObjectDetector / evaluateObjectDetection
%   classes  class names in index order (string array)
%   T        table: imageFilename, boxes, labels
%   Images without any label are skipped (the trainer needs at least one box).
classes = readClassNames(fullfile(root, 'data.yaml'));
imgDir = fullfile(root, 'images', split);  lblDir = fullfile(root, 'labels', split);
files = [dir(fullfile(imgDir, '*.jpg')); dir(fullfile(imgDir, '*.jpeg')); dir(fullfile(imgDir, '*.png'))];
if isempty(files), error('ankel:dataset', 'No images in %s', imgDir); end

names = strings(0, 1);  boxes = {};  labels = {};
for i = 1:numel(files)
    [~, stem] = fileparts(files(i).name);
    lf = fullfile(lblDir, stem + ".txt");
    if ~isfile(lf), continue; end
    L = readmatrix(lf, 'FileType', 'text');
    if isempty(L), continue; end
    L = reshape(L, [], 5);
    info = imfinfo(fullfile(imgDir, files(i).name));  W = info(1).Width;  H = info(1).Height;
    w = L(:,4) * W;  h = L(:,5) * H;
    x = L(:,2) * W - w/2 + 1;  y = L(:,3) * H - h/2 + 1;       % MATLAB boxes are 1-based [x y w h]
    b = [x y w h];
    b(:,1) = max(b(:,1), 1);  b(:,2) = max(b(:,2), 1);
    b(:,3) = min(b(:,3), W - b(:,1) + 1);  b(:,4) = min(b(:,4), H - b(:,2) + 1);
    keep = b(:,3) >= 2 & b(:,4) >= 2 & L(:,1) >= 0 & L(:,1) < numel(classes);
    if ~any(keep), continue; end
    names(end+1, 1) = string(fullfile(imgDir, files(i).name)); %#ok<AGROW>
    boxes{end+1, 1} = b(keep, :); %#ok<AGROW>
    labels{end+1, 1} = categorical(classes(L(keep,1) + 1), classes); %#ok<AGROW>
end
if isempty(names), error('ankel:dataset', 'No labelled images in %s', imgDir); end
T = table(names, boxes, labels, 'VariableNames', {'imageFilename', 'boxes', 'labels'});
ds = combine(imageDatastore(T.imageFilename), boxLabelDatastore(T(:, {'boxes', 'labels'})));
end

function classes = readClassNames(yamlFile)
txt = string(splitlines(fileread(yamlFile)));
classes = strings(0, 1);
for i = 1:numel(txt)
    tok = regexp(txt(i), '^\s+(\d+):\s*(\S.*?)\s*$', 'tokens', 'once');
    if ~isempty(tok), classes(str2double(tok(1)) + 1, 1) = tok(2); end
end
if isempty(classes), error('ankel:dataset', 'No class names found in %s', yamlFile); end
end
