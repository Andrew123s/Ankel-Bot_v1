function out = wasteAugment(data)
%WASTEAUGMENT Training augmentation for water scenes: {image, boxes, labels}.
%   Horizontal flip and brightness/contrast jitter (glare, overcast, dusk).
%   No vertical flip or large rotation: floating objects are never upside-down
%   and the horizon is roughly level from the vessel.
I = data{1};  boxes = data{2};  labels = data{3};
if rand < 0.5
    I = fliplr(I);
    W = size(I, 2);
    boxes(:, 1) = W - boxes(:, 1) - boxes(:, 3) + 2;       % 1-based mirror of [x y w h]
end
gain = 0.7 + 0.6*rand;  bias = 30*(rand - 0.5);
I = uint8(min(255, max(0, double(I)*gain + bias)));
out = {I, boxes, labels};
end
