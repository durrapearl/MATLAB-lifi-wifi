% Clear environment and command window
clear;
clc;

% Network and device parameters
L = 5; W = 5; H = 3; % Network dimensions
P_WiFi_tx_dBm = 20; P_LiFi_tx_W = 4.22; % Power levels
A_PD_cm2 = 1; G_c = 8.36; Gamma_oe = 0.53; G_f = 1; % Optical properties
Psi_L_deg = 28.9; Psi_W_deg = 28.9; % FOV dimensions
T_qs = 1; t_HHO = 0.3; t_VHO = 0.5; % Time parameters
N_WiFi = -174; B_LiFi = 40e6; B_WiFi = 20e6; % Bandwidth and noise parameters

% AP and user positions setup, including WiFi AP
AP_positions = [1, 1; 1, 4; 4, 1; 4, 4]; % LiFi AP positions
wifiAPPosition = [2.5, 2.5]; % Central WiFi AP position
userPositions = rand(10, 2) * max([L W]); % Random user positions
numUsers = size(userPositions, 1); % Calculate the number of users

% Simulate ri_u and Oi_u matrices (example data)
ri_u = rand(numUsers, 5); % Random signal strength values to each AP from each user
Oi_u = rand(numUsers, 5); % Random opportunity cost values

% Initialize the SDN controller with AP positions and number of users
sdnController = initializeSDNController(AP_positions, numUsers);

% Call the function with the updated parameters
[sdnController, chi] = assignUsersToAPs(sdnController, ri_u, userPositions, AP_positions, wifiAPPosition, numUsers);

% Define mobility parameters
ScenLM = 0.1; % Low mobility speed in m/s
ScenAM = 0.5; % Average mobility speed in m/s
ScenHM = 0.8; % High mobility speed in m/s
S_min_nwp = 0.1; % Minimum speed for variable speed scenario in m/s
S_max_nwp = 0.8; % Maximum speed for variable speed scenario in m/s
T_nwp_pause_min = 0; % Minimum pause time in seconds
T_nwp_pause_max = 10; % Maximum pause time in seconds

% Assign speeds and pause times based on scenarios
for u = 1:numUsers
    scenarioType = randi([1, 4]); % Randomly assign one of the four scenarios
    switch scenarioType
        case 1 % Low Mobility Scenario
            userSpeeds(u) = ScenLM;
            pauseTimes(u) = 5; % Example fixed pause time
        case 2 % Average Mobility Scenario
            userSpeeds(u) = ScenAM;
            pauseTimes(u) = 5; % Example fixed pause time
        case 3 % High Mobility Scenario
            userSpeeds(u) = ScenHM;
            pauseTimes(u) = 5; % Example fixed pause time
        case 4 % Variable Speed Scenario
            userSpeeds(u) = S_min_nwp + (S_max_nwp - S_min_nwp) * rand();
            pauseTimes(u) = T_nwp_pause_min + (T_nwp_pause_max - T_nwp_pause_min) * rand();
    end
end

% Visualize and analyze results
visualizeNetworkResults(userPositions, AP_positions, wifiAPPosition, chi);
plotSINRDistribution(L, W, AP_positions, wifiAPPosition);
plotUserThroughputImpact();
plotThroughputFunctionBlockage();
plotDataRateCDF();
disp('FDR Performance Indicator Matrix:');
disp(computeFDRIndicatorMatrix(AP_positions, userPositions, H, Psi_L_deg, Psi_W_deg));



% Functions
function [chi, zi] = ruleBasedResourceAllocation(ri_u, Oi_u, ~, ~)
    numUsers = size(ri_u, 1);
    numAPs = size(ri_u, 2);
    chi = zeros(numUsers, numAPs);
    ri_times_Oi = ri_u .* Oi_u;

    for u = 1:numUsers
        [~, i] = max(ri_times_Oi(u, :));
        chi(u, i) = 1;
    end

    N_i_U = sum(chi, 1);
    for i = 1:numAPs
        if i == 1 && N_i_U(i) == 0 % Assuming first AP is WiFi
            maxLoadLiFi = max(N_i_U(2:end));
            i_LiFi = find(N_i_U == maxLoadLiFi, 1) + 1;
            [~, u_tilde] = max(ri_u(:, i_LiFi));
            chi(u_tilde, i_LiFi) = 0;
            chi(u_tilde, 1) = 1;
        end
    end

    N_i_U = sum(chi, 1);
    zi = zeros(numUsers, numAPs);
    for u = 1:numUsers
        for i = 1:numAPs
            if chi(u, i) == 1
                zi(u, i) = 1 / N_i_U(i);
            end
        end
    end
end

function IFDR = computeFDRIndicatorMatrix(AP_positions, RU_positions, ~, PsiL, PsiW)
    NAP = size(AP_positions, 1);
    MRU = size(RU_positions, 1);
    IFDR = zeros(MRU, NAP);
    for i = 1:NAP
        for j = 1:MRU
            % Compute angles and projections
            delta_x = RU_positions(j,1) - AP_positions(i,1);
            delta_y = RU_positions(j,2) - AP_positions(i,2);
            theta_ij = atan2(delta_y, delta_x); 
            if abs(theta_ij) <= (PsiW + PsiL) / 2
                IFDR(j, i) = 1;
            else
                IFDR(j, i) = 0;
            end
        end
    end
end

function plotSINRDistribution(L, W, lifiAPPositions, wifiAPPosition)
    [X, Z] = meshgrid(linspace(0, W, 100), linspace(0, L, 100));

    % Example SINR calculations
    SINR_LiFi = 100 * exp(-((X - 2.5).^2 + (Z - 2.5).^2) / 1.5);
    SINR_WiFi = 60 * exp(-((X - 2.5).^2 + (Z - 2.5).^2) / 2.5);
    
    figure;
    hold on;
    surf(X, Z, SINR_LiFi, 'FaceColor', 'yellow', 'EdgeColor', 'none', 'DisplayName', 'LiFi SINR');
    surf(X, Z, SINR_WiFi, 'FaceColor', 'blue', 'EdgeColor', 'none', 'DisplayName', 'WiFi SINR');
    colorbar;
    title('SINR Distribution Across Room');
    xlabel('Room Width (m)');
    ylabel('Room Length (m)');
    zlabel('SINR (dB)');
    legend;
    view(-30, 30);
    hold off;
end

function plotUserThroughputImpact()
    numUsers = 4:2:14; % Generates a vector from 4 to 14 in steps of 2
    numPoints = length(numUsers); % Number of points to generate data for
    
    % Generate decreasing throughput values from 500 Mbps decreasing linearly
    throughputBase = linspace(500, 200, numPoints); % Linearly spaced vector from 500 to 200
    
    % Apply scaling factors for different scenarios
    throughputLM = throughputBase; % No reduction for Low Mobility
    throughputAM = throughputBase * 0.93; % 7% reduction for Average Mobility
    throughputHM = throughputBase * 0.88; % 12% reduction for High Mobility

    figure;
    hold on;
    plot(numUsers, throughputLM, '^-', 'DisplayName', 'ScenLM');
    plot(numUsers, throughputAM, 'o-', 'DisplayName', 'ScenAM');
    plot(numUsers, throughputHM, 's-', 'DisplayName', 'ScenHM');
    title('Effect of Number of Users on Average User Throughput');
    xlabel('Number of Users');
    ylabel('Average User Throughput (Mbps)');
    legend;
    grid on;
    hold off;
end


function plotThroughputFunctionBlockage()
    numUsers = 4:2:14; % Generates a vector from 4 to 14 in steps of 2
    numPoints = length(numUsers); % Number of points to generate data for
    
    % Generate decreasing throughput values from 500 Mbps decreasing linearly
    throughputBase = linspace(500, 200, numPoints); % Linearly spaced vector from 500 to 200
    
    % Apply a fixed reduction factor to simulate blockage impact
    throughputWithoutBlockage = throughputBase;
    throughputWithBlockage = throughputBase * 0.86; % 14% reduction due to blockage

    figure;
    hold on;
    bar(numUsers, [throughputWithoutBlockage' throughputWithBlockage'], 0.4);
    title('Average User Throughput With and Without Blockage');
    xlabel('Number of Users');
    ylabel('Throughput (Mbps)');
    legend('Without Blockage', 'With Blockage');
    grid on;
    hold off;
end

% Function to plot CDF of user data rates
function plotDataRateCDF()
    dataRates = rand(100, 1) * 100;
    dataRatesWithBlockage = rand(100, 1) * 80;
    figure;
    cdfplot(dataRates);
    hold on;
    cdfplot(dataRatesWithBlockage);
    title('CDF of User Data Rates');
    xlabel('Data Rate (Mbps)');
    ylabel('CDF');
    legend('Without Blockage', 'With Blockage');
    grid on;
end
function results = runNetworkSimulation(lifiAPPositions, wifiAPPosition, userPositions)
    numUsers = size(userPositions, 1);
    numLiFiAPs = size(lifiAPPositions, 1);

    % Initializing structure fields
    results.lifiUsers = zeros(numLiFiAPs, 1);
    results.wifiUsers = 0;
    results.userAssignments = zeros(numUsers, 1);  % This is important

    for i = 1:numUsers
        lifiDistances = sqrt(sum((userPositions(i,:) - lifiAPPositions).^2, 2));
        wifiDistance = norm(userPositions(i,:) - wifiAPPosition);

        [minLifiDistance, lifiIndex] = min(lifiDistances);

        if minLifiDistance < wifiDistance
            results.lifiUsers(lifiIndex) = results.lifiUsers(lifiIndex) + 1;
            results.userAssignments(i) = 1;  % Assign to LiFi
        else
            results.wifiUsers = results.wifiUsers + 1;
            results.userAssignments(i) = 0;  % Assign to WiFi
        end
    end
end

% Function to visualize network results, showing multi-AP connections
function visualizeNetworkResults(userPositions, AP_positions, wifiAPPosition, chi)
    figure;
    hold on;

    % Visualize LiFi APs
    scatter(AP_positions(:, 1), AP_positions(:, 2), 100, 'r', 'filled', 'DisplayName', 'LiFi APs');

    % Visualize WiFi AP separately
    scatter(wifiAPPosition(1), wifiAPPosition(2), 100, 'b', 'filled', 'DisplayName', 'WiFi AP');
    
    % Plot users
    scatter(userPositions(:,1), userPositions(:,2), 50, 'filled', 'DisplayName', 'Users');

    % Draw lines between users and LiFi APs they are connected to
    for i = 1:size(userPositions, 1)
        for j = 1:size(AP_positions, 1) % Only LiFi APs
            if chi(i, j)
                plot([userPositions(i,1) AP_positions(j,1)], [userPositions(i,2) AP_positions(j,2)], 'k--');
            end
        end
        % Check and draw line to WiFi AP if connected
        if chi(i, size(AP_positions, 1) + 1) % Assuming the last column in chi is for WiFi
            plot([userPositions(i,1) wifiAPPosition(1)], [userPositions(i,2) wifiAPPosition(2)], 'k--');
        end
    end

    xlabel('Room Width (m)');
    ylabel('Room Length (m)');
    title('Hybrid LiFi-WiFi Network User Allocation');
    legend;
    grid on;
    hold off;
end

function sdnController = initializeSDNController(AP_positions, numUsers)
    % Initialize SDN controller settings
    sdnController.currentLoad = zeros(size(AP_positions, 1) + 1, 1);
    sdnController.userAPAssignment = zeros(numUsers, 1);
end

function [sdnController, chi] = assignUsersToAPs(sdnController, ri_u, userPositions, AP_positions, wifiAPPosition, numUsers)
    % Decision making based on network metrics
    for u = 1:numUsers
        % Example metric: Choose AP based on the strongest signal
        lifiSignals = sqrt(sum((userPositions(u,:) - AP_positions).^2, 2));
        wifiSignal = norm(userPositions(u,:) - wifiAPPosition);
        [minLifiSignal, lifiIdx] = min(lifiSignals);

        % Assign user to Li-Fi or Wi-Fi based on signal strength
        if minLifiSignal < wifiSignal
            sdnController.userAPAssignment(u) = lifiIdx; % Li-Fi AP index
        else
            sdnController.userAPAssignment(u) = size(AP_positions, 1) + 1; % Wi-Fi AP index
        end
    end

    % Update connection matrix chi based on SDN decisions
    chi = zeros(numUsers, size(AP_positions, 1) + 1);
    for u = 1:numUsers
        chi(u, sdnController.userAPAssignment(u)) = 1;
    end
end
