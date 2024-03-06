% This script runs Q1(d)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Only enable the GPS
configuration.enableGPS = true;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results for any questions or subparts of questions
% must have this value set to true.
configuration.perturbWithNoise = false;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q1_d');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);

% drivebotSLAMSystem.setRecommendOptimizationPeriod(10);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

plotter(["Errors","Covariance"],"Figures/q1d",results)

