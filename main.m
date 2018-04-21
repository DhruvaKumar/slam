
% ============================= Description ==============================
%
% Author: Dhruva Kumar
%
%   - Initialize parameters:
%       - Initialize 1500x1500 log odds grid map
%       - Grid resolution is kept at 5cm per cell
%       - Odometry pose {x,y,theta} is computed from lidar.pose[1-2] & rp[y]
%       - head pitch, neck yaw + torso yaw (didn't matter)
%   - Initialize particles (n=100)
%       - position (0,0) | orientation with a variance of +-20 degrees
%       along 0 degrees
%       - get the lidar hits and update the log odds grid map for the 1st iteration
%       - particle weights initialzied to 1
%   - i=2:end
%       - dynamic update of the particles done according to difference in
%       odometry {dX, dY, dTheta}. random normal noise with variance 0.3
%       added to the motion update.
%       - the weights of the particles are updated and resampled if
%       required
%       - the log odds map is updated based on the lidar scan of the best
%       particle (occupied+10*log_r | free-log_r) (log_r = 0.02)
%
%   Parameters  - head_pitch > 0.25 (radians). hard threshold. worked better
%               than (lidar_scan > r/sin(pitch))
%               - n=100
%               - variance of particles during dynamic update =0.3. matters. 
%               the larger the better. but not too large for it to diverge
%               and not capture the scans properly. higher variance is
%               better with more particles
%
%
% ========================================================================

% add path
addpath(genpath('/home/dhruva/Documents/Learning in Robotics/4-SLAM/data'));
addpath(genpath('/home/dhruva/Documents/Learning in Robotics/4-SLAM/src'));

% load data
load lidar0
load joints0

%% init parameters

% init occupancy grid map in terms of log odds ratio to 0(p()=0.5)
bound = 1500;
grid_log_odds = zeros(bound);
path = zeros(bound);

% grid cell resolution
gridCell_res = 5; % cm
dist_scale = 100/gridCell_res;

% offline computation of lidar data: ts_lidar, lidar_scan, pose(x,y,theta)
ts_lidar = zeros(numel(lidar),1);
x_c = zeros(numel(lidar),1); y_c = zeros(numel(lidar),1); 
scann = zeros(numel(lidar), length(lidar{1}.scan));
theta_c = zeros(numel(lidar),1);
for i=1:numel(lidar)
    ts_lidar(i) = lidar{i}.t;
    scann(i,:) = lidar{i}.scan;
    x_c(i) = lidar{i}.pose(1) * dist_scale;
    y_c(i) = lidar{i}.pose(2) * dist_scale;
    theta_c(i) = lidar{i}.rpy(3);
end

% compute head pitch: skip (logical array, when to skip)
% pos = pos';
head_pitch = spline(ts, pos(:,2), ts_lidar);
% lidar_r = mean(scann(:,535:545),2);
% h = 141 * 10^-2; % m;
% thresh = repmat(h,numel(lidar),1) ./ sin(head_pitch);
% skip = lidar_r > thresh;
skip = head_pitch > 0.25;

% compute neck yaw
neck_yaw = spline(ts, pos(:,get_joint_index('Neck')), ts_lidar);
torso_yaw = spline(ts, pos(:,get_joint_index('TorsoYaw')), ts_lidar);
neck_yaw = neck_yaw + torso_yaw;

% pose_all: {x,y,theta} - pose of the bot according to odometry
pose_all = zeros(numel(lidar), 3);

% new dPose_all
dX = [0; diff(x_c)];
dY = [0; diff(y_c)];
dtheta = [0; diff(theta_c)]; %dyaw_gyro

%% initial point

% get the lidar scan of the first iteration
[pose_prev, x_prev, y_prev] = getLidarScan2(pose_all(1,:), lidar{1}.scan,...
                                            dist_scale);
% update grid map (log odds ratio)
scan = [x_prev; y_prev];
[grid_log_odds, path, ind_occ_prev] = updateGridMap(pose_prev, scan, grid_log_odds, bound, path);
    
% jitter the point - create n particles at 0,0. randomize the orientation
% around +- 20 degrees
n = 100;
theta_p = normrnd(0, 20*pi/180, [n,1]); % variance of 20 degrees
pose_p_prev = repmat(pose_prev,n,1) + [zeros(n,2), theta_p];

% pose_p_all = cell(numel(lidar),1);
% pose_p_all{1} = pose_p_prev;
% pose_b_all = cell(numel(lidar),1);
% pose_b_all{1} = pose_prev;

w_prev = ones(n,1);

% video
h_avi = VideoWriter('slam0_current9.avi');
h_avi.FrameRate = 10;
h_avi.Quality = 100;
h_avi.open();

pose_p = zeros(size(pose_p_prev));

%%
for i = 2:numel(lidar)
    %%
    fprintf('Iteration %d',i);
    
    % dynamic/motion update: shift all the particles by the dynamics 
    % specified by odometry (BUT in the local frame of reference)
    for j = 1:n
        pose_p(j,1:2) = (pose_p_prev(j,1:2)' + rot(pose_p_prev(j,3) - theta_c(i-1))*[dX(i); dY(i)])';
    end
    % add noise
    pose_p(:,1) = pose_p(:,1) + normrnd(0,0.3,[n,1]);
    pose_p(:,2) = pose_p(:,2) + normrnd(0,0.3,[n,1]);
    pose_p(:,3) = pose_p_prev(:,3) + dtheta(i);
    
%     pose_p_all{i} = pose_p; % pose of the particles
    
    % update weights and resample (if required)
    [pose_p, pose, scan, w, ind] = updateWeights2(grid_log_odds, w_prev, ...
        lidar{i}.scan, pose_p, bound, dist_scale, skip(i), neck_yaw(i));

    % update grid map (log odds ratio) based on the highest weighted
    % particle pose
    [grid_log_odds, path, ~] = updateGridMap(pose, scan, grid_log_odds, ...
        bound, path);
   
%     pose_b_all{i} = pose; % pose of the bot
    
    % plot
    if (mod(i,50) == 0 && i>200)
        h = figure(1); 
        % plot grid map 
        imagesc(grid_log_odds), title(sprintf('Iteration %d',i)), colormap(gray)
        hold on,
        % plot path (green)
        [PX,PY] = ind2sub([bound,bound],find(path));
        plot(PY, PX, '.g', 'MarkerSize', 2);
        % plot occupied cells/lidar hits: plot every 5th hit (blue)
        x_temp = scan(1,:); y_temp = scan(2,:);
        indexVec = 1:1081;
        x_temp(indexVec(mod(indexVec,5) ~= 0)) = nan;
        y_temp(indexVec(mod(indexVec,5) ~= 0)) = nan;
        plot(floor(y_temp)+(bound/2), floor(x_temp)+(bound/2), '.b', 'MarkerSize', 4.5),hold on,
        % plot particles (yellow)
        plot(floor(pose_p(:,2))+(bound/2), floor(pose_p(:,1))+(bound/2), '.y');
        % plot current position of bot (red)
        plot(floor(pose(2))+(bound/2), floor(pose(1))+(bound/2), '.r');
        set(gca,'position',[0 0 1 1], 'units', 'normalized')

%         set(gca,'position',[0 0 1 1],'units','normalized')
        h_avi.writeVideo(getframe(h));
         
    end
   
    % update for next iteration
    pose_p_prev = pose_p;
    pose_prev = pose;
    w_prev = w;
    fprintf('\n');
end
% toc;
h_avi.close();

%% write map to file

% print(h, '-dpng','plots/slam0_result1.png');

%% debug 2: plot only particles
% j=2000;
% j=200;
% % h_avi = VideoWriter('slam00000.avi');
% % h_avi.Quality = 100;
% % h_avi.FrameRate = 120;
% % h_avi.open();
% 
% while (~isempty(pose_p_all{j}) && ~isempty(pose_b_all{j}))
% % for j = 2000:10:numel(lidar) 
%     h = figure(2);
%     posee_pp = pose_p_all{j}; % particles
%     posee_bb = pose_b_all{j}; % bot
%     posee_odom = pose_all(j,:); % lidar odom
% %     plot((pose_pp(:,1)), -(pose_pp(:,2)), '.b'), hold on,
%     plot((posee_pp(:,2)), (posee_pp(:,1)), '.b'), hold on, % particles
%     title(sprintf('Iteration %d',j)),
%     hold on;
% %     plot((posee_bb(1)), -(posee_bb(2)), '*r'); % bot
% %     plot((posee_odom(1)), (posee_odom(2)), 'dr'); % odom
%     plot((posee_bb(2)), (posee_bb(1)), '*r'); % bot
% %     plot((posee_odom(2)), (posee_odom(1)), 'dr'); % odom
%     
% %     hold off;
% %       axis([-100 100 -10 100])
%     axis([-100 220 -200 350])
% %     axis([550 750 550 900])
% %     set(gca,'position',[0 0 1 1])
% %     h_avi.writeVideo(getframe(h));
%     j=j+10;
% end
% hold off;
% % 
% % h_avi.close();




%% test dynamic update
% 
% pose_prev = [0,0,0];
% pose = [10,10,45*pi/180];
% n = 100;
% % TODO: rand for theta?
% % [x_p, y_p] = meshgrid(-4:5, -4:5);
% % x_p = reshape(x_p, [], 1);
% % y_p = reshape(y_p, [], 1);
% % theta_p = randn(n,1);
% theta_p = normrnd(0,30*pi/180, [n,1]);
% 
% pose_p_prev = repmat(pose_prev,n,1) + [zeros(n,2), theta_p];
% pose_p_prev(:,1:2) = ceil(pose_p_prev(:,1:2));
% 
% r = norm(pose(1:2) - pose_prev(1:2));
% phi = atan2(pose(2)-pose_prev(2), pose(1)-pose_prev(1));
% pose_p(:,1) = pose_p_prev(:,1) + r*cos(pose_p_prev(:,3));
% pose_p(:,2) = pose_p_prev(:,2) + r*sin(pose_p_prev(:,3));
% pose_p(:,3) = pose_p_prev(:,3);
% % pose_p(:,3) = pose_p_prev(:,3) + (pose(3)-pose_prev(3));
% pose_p(:,1:2) = ceil(pose_p(:,1:2));
% 
% plot(pose_p_prev(:,1),pose_p_prev(:,2),'.');
% hold on,
% plot(pose_p(:,1),pose_p(:,2),'g*');
% arrow(pose_p_prev(:,1:2), pose_p(:,1:2));
% hold off;











