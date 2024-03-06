% This script runs Q3(a)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Enable the laser to support pure SLAM
configuration.enableGPS = false;
configuration.enableLaser = true;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Magic tuning for the no-prediction case
configuration.laserDetectionRange = 30;

% configuration.maximumStepNumber = 2;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q3_a');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Optimize every 500 timesteps to get a picture of the situation as it evolves
% drivebotSLAMSystem.setRecommendOptimizationPeriod(500);
drivebotSLAMSystem.setRecommendOptimizationPeriod(1);

% Set whether the SLAM system should remove prediction edges. If the first
% value is true, the SLAM system should remove the edges. If the second is
% true, the first prediction edge will be retained.
drivebotSLAMSystem.setRemovePredictionEdges(false, true);
% drivebotSLAMSystem.setRemovePredictionEdges(true, false);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

% plotter(["OptTime","Chi2","Covariance"],"Figures/q3a_no_opt",results)

