% This script runs Q2(c)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Enable the laser to support pure SLAM
configuration.enableLaser = true;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q2_c');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);
drivebotSLAMSystem.setRecommendOptimizationPeriod(inf);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);


% This is how to extract the graph from the optimizer
graph = drivebotSLAMSystem.optimizer();

% This is how to extract cell arrays of the vertices and edges from the
% graph
allVertices = graph.vertices();
allEdges = graph.edges();

% Work out the number of vehicle poses and landmarks. 
numVehicleVertices = 0;
numLandmarks = 0;

landmarkObservationsPerVehicleVertex = 0;
observationsPerLandmarkVertex = 0;

% Q2c:
% Finish implementing the code to capture information about the graph
% structure.


%Count each instance of Landmark and Vehicle State Vertex
for i = 1:length(allVertices)
    numLandmarks = numLandmarks + isa(allVertices{i},"drivebot.graph.LandmarkStateVertex");
    numVehicleVertices = numVehicleVertices + isa(allVertices{i},"drivebot.graph.VehicleStateVertex");
end

%Count each instance of Landmark Range Bearing and Vehicle Kinematics Edge
numland=0;
numveh = 0;
for i = 1:length(allEdges)
    numland = numland + isa(allEdges{i},"drivebot.graph.LandmarkRangeBearingEdge");
    numveh = numveh + isa(allEdges{i},"drivebot.graph.VehicleKinematicsEdge");
end


    
%Divide number of Landmark Range Bearing Edge by number of Vehicle Vertices
landmarkObservationsPerVehicleVertex = numland/numVehicleVertices;

%Divide number of Landmark Range Bearing Edge by number of Landmark
%Vertices
observationsPerLandmarkVertex = numland/numLandmarks;
