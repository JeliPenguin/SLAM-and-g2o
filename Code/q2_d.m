% This script runs Q2(d)
rng(0)
% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Enable the laser to support pure SLAM
configuration.enableLaser = true;

% For this part of the coursework, this should be set to false.
configuration.perturbWithNoise = false;

% Set this value to truncate the run at a specified timestep rather than
% run through the whole simulation to its end.
configuration.maximumStepNumber = 2000;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q2_d');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);
drivebotSLAMSystem.setRecommendOptimizationPeriod(inf);

% Q2d:
% Explore the  timestep where the loop closure occurs, and get
% results just before and after the loop closure event
% warning('q2_d:unimplemented', ...
%         'Analyse loop closure behaviour for Q2d.')

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

[x, afterP, landmarkAfterIds] = drivebotSLAMSystem.landmarkEstimates();

title("After Loop Closure");
saveas(gcf, 'Figures/q2d_after_loop_closure', 'png');

% Minimal output plots. For your answers, please provide titles and label
% the axes.

configuration.maximumStepNumber = 1200 ;
simulator = drivebot.DriveBotSimulator(configuration, 'q2_d');
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);
drivebotSLAMSystem.setRecommendOptimizationPeriod(inf);
drivebotSLAMSystem.setValidateGraph(false);

results = minislam.mainLoop(simulator, drivebotSLAMSystem);

title("Just Before Loop Closure");
saveas(gcf, 'Figures/q2d_before_loop_closure', 'png');

[x, beforeP, landmarkIds] = drivebotSLAMSystem.landmarkEstimates();

disp("Landmark Covariance Determinants")
disp(["Landmark ID","Uncertainty Change"])
for i=1:length(beforeP)
    landmarkCovBefore = beforeP(:,:,i);
    landmarkCovAfter = afterP(:,:,i);
    disp([landmarkIds(i),sqrt(det(landmarkCovBefore)),sqrt(det(landmarkCovAfter)),sqrt(det(landmarkCovBefore))-sqrt(det(landmarkCovAfter))])
end

