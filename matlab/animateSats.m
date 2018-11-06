function animate(pos, speed)

figure(5); clf;
set(gcf, 'name', 'Animation', 'NumberTitle', 'off');

a = 6378137.0; % semi-major axis
f = (1.0/298.257223563); % flattening factor
b = a * (1-f);

[x, y, z] = ellipsoid(0, 0, 0, a, a, b, 100);
surf(x, y, z)
axis equal
hold on;
view(150, 23);

handles = [];

[niter, longest_idx] = max(cellfun(@(x) length(x), pos));

time = pos{longest_idx}(2,:);
for i = 1:speed:niter
    t = time(i);
    handles = draw_sats(pos, t, handles);
    drawnow
end
end



function handles = draw_sats(pos, t, handles)

first_time = isempty(handles);
for i = 1:length(pos)
    idx = find(pos{i}(2,:) == t);    
    if first_time
        if isempty(idx)
            handles = [handles, scatter3(0, 0, 0)];
        else
            handles = [handles, scatter3(pos{i}(3, idx), pos{i}(4, idx), pos{i}(5, idx))];
        end
    else
        if isempty(idx)
            set(handles(i), 'XData', 0, 'YData', 0, 'ZData', 0);
        else
            set(handles(i), 'XData', pos{i}(3, idx), 'YData', pos{i}(4, idx), 'ZData', pos{i}(5, idx));
        end
        
    end
end
end