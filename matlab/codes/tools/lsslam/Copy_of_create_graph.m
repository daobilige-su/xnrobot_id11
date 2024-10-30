% This file create graph structure "g" for lsSLAM.m




%% refresh
clear all;
close all;

%% Parameters

% define speed of sound
cs = 340;

M_y = 3;
M_x = 3;

% number of mics
M = M_y*M_x;

% distance between each mic
mic_dis = 0.5;

% maximum starting time delays
max_delay = 100e-3;

% microphone sampling frequency
Fs = 44100;

% max clock difference of sound card (per sec)
max_drift = (5/Fs);

% ground truth of sound src odometry, distance betwean each step per sec
src_step = 0.05;

% observation noise
TDOA_observation_noise = 0.001/3; % within 3 sigma (within 1ms)

TDOA_observation_noise_amp_rate = 2;

% sigma of random walker
random_walker_noise = 1/3; % within 3 sigma (within 1m)

% noise used for microphone initialization
mic_pos_init_noise = mic_dis/1;

% noise used for src position initialization
src_pos_init_noise = mic_dis*3/3; % within 3 sigma (within 3*mic_dis)

%% microphone ground truth

% ground truth of mic position.
mic_pos = zeros(M,2);
       
for m=1:M_y
    for n=1:M_x
        mic_pos(M_x*(m-1)+n,1) = (n-1)*mic_dis;
        mic_pos(M_x*(m-1)+n,2) = (m-1)*mic_dis;
    end
end

% ground truth of mic starting delay.
mic_delay = [0;
             rand(M-1,1)*max_delay];

% ground truth of mic clock difference.         
mic_drift = [0;
             rand(M-1,1)*max_drift];

%% construction of source odometry (ground truth)

% starting position
src_pos = [-mic_dis/2 -mic_dis/2];

for i = 1:floor(M_x/2)
    % trajectory 1.
    x_const = src_pos(end,1);
    y_const = src_pos(end,2);
    for n = 1:M_y*(mic_dis/src_step)
        src_pos = [src_pos; x_const y_const+src_step*n];
    end

    % trajectory 2.
    x_const = src_pos(end,1);
    y_const = src_pos(end,2);

    for n = 1:1*(mic_dis/src_step)
        src_pos = [src_pos; x_const+src_step*n y_const];
    end

    % trajectory 3.
    x_const = src_pos(end,1);
    y_const = src_pos(end,2);

    for n = 1:M_y*(mic_dis/src_step)
        src_pos = [src_pos; x_const y_const-src_step*n];
    end

    % trajectory 4.
    x_const = src_pos(end,1);
    y_const = src_pos(end,2);

    for n = 1:1*(mic_dis/src_step)
        src_pos = [src_pos; x_const+src_step*n y_const];
    end
end

if (floor(M_x/2)*2)<M_x
    % trajectory 5.
    x_const = src_pos(end,1);
    y_const = src_pos(end,2);

    for n = 1:M_y*(mic_dis/src_step)
        src_pos = [src_pos; x_const y_const+src_step*n];
    end

    % trajectory 6.
    x_const = src_pos(end,1);
    y_const = src_pos(end,2);

    for n = 1:1*(mic_dis/src_step)
        src_pos = [src_pos; x_const+src_step*n y_const];
    end

    % trajectory 7.
    x_const = src_pos(end,1);
    y_const = src_pos(end,2);

    for n = 1:M_y*(mic_dis/src_step)
        src_pos = [src_pos; x_const y_const-src_step*n];
    end
else
    % trajectory 5.
    x_const = src_pos(end,1);
    y_const = src_pos(end,2);

    for n = 1:M_y*(mic_dis/src_step)
        src_pos = [src_pos; x_const y_const+src_step*n];
    end
end

% plot the microphone and sound src ground truth
figure,
hold on;
plot(mic_pos(:,1),mic_pos(:,2),'LineStyle','none','Marker','s',...
    'MarkerEdgeColor','r');
plot(src_pos(:,1),src_pos(:,2),'LineStyle','none','Marker','x',...
    'MarkerEdgeColor','b');
xlabel('x position (m)');ylabel('y position (m)');
legend('microphone','src position');
grid on;
hold off;

%% compute the observation in TDOA (Time Difference Of Arrival)

% initialize the TOF (Time Of Flight) and TDOA
TOF = zeros(size(src_pos,1),M);
TDOA = zeros(size(src_pos,1),M-1);

% initialize noisy TDOA observation
TDOA_observation = zeros(size(src_pos,1),M-1);

% compute the TOF, TDOA and TDOA observation for each sound src location
for n=1:size(src_pos,1)
    
    % computation of TOF
    TOF(n,:) = sqrt((src_pos(n,1)-mic_pos(:,1)).^2 + ...
        (src_pos(n,2)-mic_pos(:,2)).^2)/cs;
    % computation of TDOA (ground truth)
    TDOA(n,:) = TOF(n,2:end) - TOF(n,1);
    % computation of TDOA observation
    TDOA_observation(n,:) = TDOA(n,:) + randn(1,M-1)*...
        TDOA_observation_noise/5 + mic_delay(2:end)' + n*mic_drift(2:end)';
    
end

%% ground truth of state vector

% ground truth of state vector of the microphone array
mic_state = [mic_pos mic_delay mic_drift];

% ground truth of state vector = [mic_1_x; mic_1_y; mic_1_delay;
% mic_1_draft; ... ; mic_9_x; mic_9_y; mic_9_delay; mic_9_draft; ... ;
% src_1_x; src_1_y; src_2_x; src_2_y; ... ; src_n_x; src_n_y];
g.x_gt = [reshape(mic_state',4*M,1);reshape(src_pos',2*size(src_pos,1),1)];

%% creation of edges 

edges_count = 1;

for n=1:size(src_pos,1)
    
    % (p-p) pose pose constraits 
    if(n>1)
        % type of the constraint
        type_p_p = 'P';
        % measurement (in random walker, no information about next movement
        % , therefore, the measurment is [0;0])
        measurement_p_p = [0;0];
        % information matrix [1/sigma^2 0; 0 1/sigma^2];
        information_p_p = eye(2)*(1/(random_walker_noise^2));
        % index of previous time sound src position (starting index)
        fromIdx_p_p = 2*(n-2)+1 + 4*M;
        % index of current time sound src position (starting index)
        toIdx_p_p = 2*(n-1)+1 + 4*M;
    
        % fill the above information to "g" structure
        g.edges(edges_count).type = type_p_p;
        g.edges(edges_count).measurement = measurement_p_p;
        g.edges(edges_count).information = information_p_p;
        g.edges(edges_count).fromIdx = fromIdx_p_p;
        g.edges(edges_count).toIdx = toIdx_p_p;
        % update the edges_count
        edges_count = edges_count+1;
    end
    
    % (p-l) pose landmark constraits 
    % type of the constraint
    type_p_l = 'L';
    % measurement (noisy TDOA observation)
    measurement_p_l = TDOA_observation(n,:)';
    % information matrix: (1/sigma^2)*I (8x8 matrix)
    information_p_l = eye(M-1)*(1/((TDOA_observation_noise*TDOA_observation_noise_amp_rate)^2));
    % starting index of landmark in state vector, which is always 1.
    fromIdx_p_l = 1;
    % index of current time sound src position (starting index)
    toIdx_p_l = 2*(n-1)+1 + 4*M;
    
    % fill the above information to "g" structure
    g.edges(edges_count).type = type_p_l;
    g.edges(edges_count).measurement = measurement_p_l;
    g.edges(edges_count).information = information_p_l;
    g.edges(edges_count).fromIdx = fromIdx_p_l;
    g.edges(edges_count).toIdx = toIdx_p_l;
    % update the edges_count
    edges_count = edges_count+1;
end

%% initialization of state vector

% initialize the mic state (pos: true position + big initialization noise
% for random pos, delay: 0 for absolute no knowledge of delay, drift: 0 for
% absolute no knowledge of drift (+randn*(1e-6) for avoiding singularity in
% slSLAM))
x_mic_init = [mic_pos+(mic_pos_init_noise*(rand(size(mic_pos))-0.5)*2),...
    zeros(M,1),zeros(M,1)+randn*(1e-6)];

% fix some mic position to avoid ambiguity
% 1st mic is reference, hence pos is (0,0) and 0 delay 0 drift
x_mic_init(1,:) = zeros(1,4);

% make sure the 2nd mic is on positive x axis
if(x_mic_init(2,1)<0)
    x_mic_init(2,1) = -1*x_mic_init(2,1);
end
x_mic_init(2,2) = 0;

% make sure the 3rd mic's y is positive
if(x_mic_init(4,2)<0)
    x_mic_init(4,2) = -1*x_mic_init(4,2);
end

% initialization of sound src postion. all (0,0) + noise to describe no
% knowledge about it.
% x_src_init = zeros(size(src_pos,1),2) + rand(size(src_pos,1),2);
x_src_init = g.x_gt(4*M+1:end,1) + randn(size(g.x_gt,1)-4*M,1)*...
    src_pos_init_noise;

% construct the initial state vector based on above.
g.x = [reshape(x_mic_init',4*M,1);x_src_init];

%% create offset and dimension
% just to make it suitable for cyrill's code framework

% initialize two variables
offset = [];
dimension = [];

% construct two variables for the landmark state vecotr
for n=1:M
    offset = [offset;(n-1)*4];
    dimension = [dimension;4];
end
% construct two variables for the sound src position state vecotr
for n=1:size(src_pos,1)
    offset = [offset;(n-1)*2 + 4*M];
    dimension = [dimension;2];
end

% fill this two variables into "g" structure 
for n = 1:(M + size(src_pos,1))
    g.idLookup(n).offset = offset(n);
    g.idLookup(n).dimension = dimension(n);
end  

%%
g.M_x = M_x;
g.M_y = M_y;
g.M = M;
g.mic_dis = mic_dis;

% save the "g" structure
save('../data/mic_array.mat','g');

% that's all folks