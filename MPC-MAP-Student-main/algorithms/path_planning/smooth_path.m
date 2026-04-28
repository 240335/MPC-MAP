function [new_path] = smooth_path(old_path, read_only_vars)
%SMOOTH_PATH Summary of this function goes here

new_path = old_path;
if size(old_path, 1) < 3, return; end

grid    = read_only_vars.discrete_map.map;
res     = read_only_vars.map.discretization_step;
bounds  = read_only_vars.map.limits;

radius  = ceil(0.25 / res);
blocked = imdilate(grid > 0, strel('disk', radius));
[R, C]  = size(blocked);

snap = @(pt) [round((pt(2)-bounds(2))/res)+1, round((pt(1)-bounds(1))/res)+1];
passable = @(pt) is_clear(snap(pt), R, C, blocked);

w_data   = 0.15;
w_smooth = 0.35;
epsilon  = 1e-5;

for iter = 1:3000
    total_shift = 0;
    for i = 2 : size(new_path,1)-1
        delta = w_data   * (raw_path(i,:) - new_path(i,:)) + ...
                w_smooth * (new_path(i-1,:) + new_path(i+1,:) - 2*new_path(i,:));
        moved = new_path(i,:) + delta;
        if passable(moved)
            new_path(i,:) = moved;
            total_shift  = total_shift + norm(delta);
        end
    end
    if total_shift < epsilon, break; end
end

new_path = old_path;

end

function ok = is_clear(cell, R, C, blocked)
r  = cell(1);  c = cell(2);
ok = r>=1 && r<=R && c>=1 && c<=C && ~blocked(r,c);
end