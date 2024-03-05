% This script runs Q1(c)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Q1c: Set the configuration to enable the compass

% Set the compass angular offset. DO NOT change this value.
configuration.compassAngularOffset=0.75*pi;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Q1c Modification
configuration.enableCompass = true;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q1_c');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);

% Force the optimizer to run with this frequency. This lets you see what's
% happening in greater detail, but slows everything down.
drivebotSLAMSystem.setRecommendOptimizationPeriod(20);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(true);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

plotter([],"Figures/q1c_compass_fault",results)

