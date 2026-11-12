function anchors = wasteAnchors(ds, inputSize, n)
%WASTEANCHORS Anchor boxes [h w] for a dataset at network input resolution.
%   Boxes are rescaled to inputSize first, then clustered; rows are sorted
%   from largest to smallest area (largest go to the coarsest detection head).
reset(ds);
B = zeros(0, 4);
while hasdata(ds)
    d = read(ds);
    sz = size(d{1});
    b = bboxresize(d{2}, inputSize(1:2) ./ sz(1:2));
    B = [B; b]; %#ok<AGROW>
end
reset(ds);
n = min(n, size(unique(round(B(:, 3:4)), 'rows'), 1));
anchors = estimateAnchorBoxes(boxLabelDatastore(table({B}, 'VariableNames', {'boxes'})), n);
[~, order] = sort(prod(anchors, 2), 'descend');
anchors = anchors(order, :);
end
