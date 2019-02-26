format compact
set(0,'DefaultFigureWindowStyle','docked')

file = fopen('../logs/sat.log', 'r');
data = fread(file, 'double');

rec_pos = [-1798904.13, -4532227.1 ,  4099781.95]';
names = ["t", "N", "px", "py", "pz", "vx", "vy", "vz", "qw", "qx", "qy", "qz", "ax", "ay", "az", "bx", "by", "bz"];

i = 1;
pos = cell(255, 1);
while i < length(data)
    N = data(i);
    t = data(i+1);
    n = (N-2)/11;
    chunk = data(i+2:i+N-1);
    chunk = reshape(chunk, 11, []);
    for j = 1:n
        sat = chunk(1,j);
        pos{sat} = [pos{sat}, [sat; t; chunk(2:end, j)]];
    end
    i = i + N;
end

pos = pos(~cellfun('isempty',pos)); 
no_meas = @(x)sum(x(3,:)) == 0;
pos = pos(~cellfun(no_meas, pos));

nsat = length(pos);

% Remove missing measurements
for i = 1:nsat
    pos{i}(:, pos{i}(4,:) == 0) = [];
end


%% Plot Position
figure(1); clf;
set(gcf, 'name', 'ECEF Position', 'NumberTitle', 'off');
positions = [];
labels = zeros(nsat,1);
titles = ['x', 'y', 'z'];
for i = 1:nsat
    for j = 1:3
        subplot(3,1,j)
        plot(pos{i}(2,:), pos{i}(2+j,:))
        hold on;
        title(titles(j))
        ylabel('m')
    end
    labels(i) = pos{i}(1,1);
    xlabel('s')
    positions = [positions, pos{i}(3:5, 545)];
end
legend(num2str(labels))
positions'

%% Plot Velocity
figure(2); clf;
set(gcf, 'name', 'ECEF Velocity', 'NumberTitle', 'off');

labels = zeros(nsat,1);
titles = ['x', 'y', 'z'];
for i = 1:nsat
    for j = 1:3
        subplot(3,1,j)
        plot(pos{i}(2,:), pos{i}(5+j,:))
        hold on;
        title(titles(j))
        ylabel('m/s')        
    end
    xlabel('s')
    labels(i) = pos{i}(1,1);
end
legend(num2str(labels))

%% Plot Sat clock
figure(3); clf;
set(gcf, 'name', 'Clock', 'NumberTitle', 'off');

labels = zeros(nsat,1);
titles = ["bias", "drift"];
ylabels = ["s", "s/s"];
for i = 1:nsat
    for j = 1:2
        subplot(2,1,j)
        plot(pos{i}(2,:), pos{i}(7+j,:))
        hold on;
        title(titles(j))
        ylabel(ylabels(j))        
    end
    xlabel('s')
    labels(i) = pos{i}(1,1);
end
legend(num2str(labels))

%% Plot variance and Health
figure(4); clf;
set(gcf, 'name', 'Misc', 'NumberTitle', 'off');

labels = zeros(nsat,1);
titles = ["variance", "health"];
ylabels = ["m^2", ""];
for i = 1:nsat
    for j = 1:2
        subplot(2,1,j)
        plot(pos{i}(2,:), pos{i}(9+j,:))
        hold on;
        title(titles(j))
        ylabel(ylabels(j))        
    end
    xlabel('s')
    labels(i) = pos{i}(1,1);
end
legend(num2str(labels))

%% Animation
animateSats(rec_pos, pos, 10);

