function recs = loadRobotLog(file)
%LOADROBOTLOG Parse the O/L robot log.
%   recs(k).kind  'O' or 'L'
%   recs(k).odom  [x y theta] robot pose in the ODOMETRY frame (cm, rad)
%   recs(k).ranges 1x180 laser ranges (cm), beam j at theta - 90deg + (j-1) deg
%   recs(k).ts    timestamp (s)
fid = fopen(file, 'r');
assert(fid > 0, 'cannot open %s', file);
cleaner = onCleanup(@() fclose(fid));
recs = struct('kind', {}, 'odom', {}, 'ranges', {}, 'ts', {});
line = fgetl(fid);
while ischar(line)
    line = strtrim(line);
    if ~isempty(line)
        vals = sscanf(line(2:end), '%f').';
        r.kind = line(1);
        r.odom = vals(1:3);
        r.ts = vals(end);
        if r.kind == 'L'
            r.ranges = vals(7:186);
        else
            r.ranges = [];
        end
        recs(end+1) = r; %#ok<AGROW>
    end
    line = fgetl(fid);
end
[~, order] = sort([recs.ts]);
recs = recs(order);
end
