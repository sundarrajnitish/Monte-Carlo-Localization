function map = loadOccupancyMap(file, resolution)
%LOADOCCUPANCYMAP Load a 0/1 grid (top row first) into a map struct.
%   map.occ(iy, ix) is true for occupied cells, with iy = 1 the BOTTOM row,
%   so world coordinates are x right / y up, in cm, origin bottom-left.
%   Cell (ix, iy) covers [(ix-1)*res, ix*res) x [(iy-1)*res, iy*res).
if nargin < 2, resolution = 4; end
raw = load(file);
map.occ = flipud(raw > 0.5);
map.res = resolution;
[map.H, map.W] = size(map.occ);
map.width  = map.W * resolution;
map.height = map.H * resolution;
map.dist = euclideanDistanceTransform(map.occ) * resolution;  % cm to nearest wall
end
