function tf = isFree(map, x, y)
%ISFREE True where (x, y) lies inside the map in a free cell.
tf = x >= 0 & y >= 0 & x < map.width & y < map.height;
ix = floor(x(tf) / map.res) + 1;
iy = floor(y(tf) / map.res) + 1;
tf(tf) = ~map.occ(sub2ind(size(map.occ), iy, ix));
end
