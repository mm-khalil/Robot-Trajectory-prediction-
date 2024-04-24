% Simple Reinforcement Learning for a Mobile Robot with 3D Movement (Live Movement)

% Environment parameters
numStatesX = 2; % Number of states in the x-axis
numStatesY = 2; % Number of states in the y-axis
numStatesZ = 2; % Number of states in the z-axis
numActions = 6;  % Number of actions (move up, move down, move left, move right, move forward, move backward)

% Q-table initialization
Q = zeros(numStatesX, numStatesY, numStatesZ, numActions);

% Parameters
alpha = 0.1;    % Learning rate
gamma = 0.9;    % Discount factor
epsilon = 0.1;  % Exploration-exploitation trade-off

% Training episodes
numEpisodes = 100;

% Plotting setup
figure;
axis([0.5 numStatesX+0.5 0.5 numStatesY+0.5 0.5 numStatesZ+0.5]);
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
grid on;
hold on;

% Plot the boundaries of the cube
plot3([1 1 1 1 1; numStatesX numStatesX numStatesX numStatesX numStatesX], ...
      [1 numStatesY numStatesY 1 1; 1 numStatesY numStatesY 1 1], ...
      [1 1 numStatesZ numStatesZ 1; 1 1 numStatesZ numStatesZ 1], 'k--', 'LineWidth', 1.5);

for episode = 1:numEpisodes
    % Initialize the state (x, y, z)
    stateX = randi(numStatesX);
    stateY = randi(numStatesY);
    stateZ = randi(numStatesZ);
    
    % Choose action using epsilon-greedy policy
    if rand() < epsilon
        action = randi([1, numActions]); % Exploration
    else
        [~, action] = max(Q(stateX, stateY, stateZ, 1:numActions)); % Exploitation
    end
    
    % Take the chosen action and observe the new state
    [nextStateX, nextStateY, nextStateZ] = updateState(stateX, stateY, stateZ, action, numStatesX, numStatesY, numStatesZ);
    
    % Calculate reward (e.g., positive for staying within the cube)
    reward = calculateReward(nextStateX, nextStateY, nextStateZ, numStatesX, numStatesY, numStatesZ);
    
    % Update Q-value using the Q-learning update rule
    Q(stateX, stateY, stateZ, action) = Q(stateX, stateY, stateZ, action) + alpha * (reward + gamma * max(Q(nextStateX, nextStateY, nextStateZ, 1:numActions)) - Q(stateX, stateY, stateZ, action));
    
    % Move to the next state
    stateX = nextStateX;
    stateY = nextStateY;
    stateZ = nextStateZ;
    
    % Plot the current state of the robot
    plot3(stateX, stateY, stateZ, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    title(['3D Movement of the Mobile Robot - Episode ', num2str(episode)]);
    drawnow;
    pause(0.1); % Adjust the pause duration as needed
end

% Testing the learned policy for live movement within the cube
currentStateX = randi(numStatesX);
currentStateY = randi(numStatesY);
currentStateZ = randi(numStatesZ);

for step = 1:20
    [~, action] = max(Q(currentStateX, currentStateY, currentStateZ, 1:numActions));
    [currentStateX, currentStateY, currentStateZ] = updateState(currentStateX, currentStateY, currentStateZ, action, numStatesX, numStatesY, numStatesZ);
    
    % Ensure the robot stays within the cube boundaries
    currentStateX = max(1, min(numStatesX, currentStateX));
    currentStateY = max(1, min(numStatesY, currentStateY));
    currentStateZ = max(1, min(numStatesZ, currentStateZ));
    
    % Plot the current state of the robot
    plot3(currentStateX, currentStateY, currentStateZ, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    title(['3D Movement of the Mobile Robot - Step ', num2str(step)]);
    drawnow;
    pause(0.5); % Adjust the pause duration as needed
end

function [nextStateX, nextStateY, nextStateZ] = updateState(stateX, stateY, stateZ, action, numStatesX, numStatesY, numStatesZ)
    % Function to update the state based on the chosen action
    % Assuming a simple grid movement for demonstration purposes
    % 1: up, 2: down, 3: left, 4: right, 5: forward, 6: backward
    nextStateX = stateX;
    nextStateY = stateY;
    nextStateZ = stateZ;

    if action == 1 % move up
        nextStateY = min(numStatesY, stateY + 1);
    elseif action == 2 % move down
        nextStateY = max(1, stateY - 1);
    elseif action == 3 % move left
        nextStateX = max(1, stateX - 1);
    elseif action == 4 % move right
        nextStateX = min(numStatesX, stateX + 1);
    elseif action == 5 % move forward
        nextStateZ = min(numStatesZ, stateZ + 1);
    elseif action == 6 % move backward
        nextStateZ = max(1, stateZ - 1);
    end
end

function reward = calculateReward(stateX, stateY, stateZ, numStatesX, numStatesY, numStatesZ)
    % Function to calculate the reward based on the new state
    % In this example, positive reward for staying within the cube
    if stateX >= 1 && stateX <= numStatesX && stateY >= 1 && stateY <= numStatesY && stateZ >= 1 && stateZ <= numStatesZ
        reward = 1;
    else
        reward = -1;
    end
end
