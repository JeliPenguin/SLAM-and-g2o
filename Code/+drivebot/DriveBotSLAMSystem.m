% This class implements an event-based estimation system using g2o and
% the barebones for building up a minimal, ideal SLAM system. The system is
% event-based and responds to a sequence of events which are time stamped
% and served in order. To implement your SLAM system, you will need to
% implement various methods which mostly involve working with the graph.
% These methods are initially stubbed out and will generate exceptions if
% you try to call them.

classdef DriveBotSLAMSystem < minislam.slam.SLAMSystem
    
    properties(Access = public, Constant)
        % Platform state dimension
        NP = 3;
        
        % Landmark dimension
        NL = 2;
        
        % Initial cache size; might help a bit with performance
        INITIAL_CACHE_SIZE = 10000;
    end
    
    properties(Access = protected)
        
        % The most recently created vehicle vertex.
        currentVehicleVertex;
        
        % The set of all vertices associated with the vehicle state over
        % time.
        vehicleVertices;
        vehicleVertexId;
        
        % The set of all prediction edges. These are removed from the graph
        % afterwards if we don't use prediction
        processModelEdges;
        numProcessModelEdges;
        
        % The landmark vertices. Confusingly enough, "Map" here refers to
        % the data structure which is used to store the landmarks. (It
        % allows random access of landmarkID to landmark object.)
        landmarkIDStateVectorMap;
        
        % How often we recommend running the optimization
        recommendOptimizationPeriod;
        
        % Flag to show if we should prune the edges. This is needed for
        % question Q3a
        removePredictionEdgesFromGraph;
        keepFirstPredictionEdge;

        % History for visualizing odometry data
        odoHistory;
        odoTime;

        %Q3b Flag to show whether graph pruning should be enabled
        prune;
    end
    
    methods(Access = public)
        
        % Create the localization system and start it up.
        function this = DriveBotSLAMSystem(configuration)
            
            % Call the base class constructor
            this = this@minislam.slam.SLAMSystem(configuration);
            
            % Preallocate for convenience
            this.vehicleVertices = cell(1, this.INITIAL_CACHE_SIZE);
            
            % No vehicle vertices initally set
            this.vehicleVertexId = 0;
            
            % The set of prediction edges, initially empty
            this.processModelEdges = cell(1, this.INITIAL_CACHE_SIZE);
            this.numProcessModelEdges = 0;
            
            % Allocate the landmark map
            this.landmarkIDStateVectorMap = containers.Map('KeyType', 'int64', 'ValueType', 'any');
            
            % By default, run very infrequently
            this.recommendOptimizationPeriod = inf;
            
            this.removePredictionEdgesFromGraph = false;
            this.keepFirstPredictionEdge = false;
            
            %Q3b: Pruning turned off by default
            this.prune = false;

            this.odoHistory = [];
            this.odoTime = [];
        end

        function [odoHistory,odoTime] = getOdoHistory(this)
            % Get Odometry data history
            odoHistory = this.odoHistory;
            odoTime = this.odoTime;
        end
        
        % Destroy the graph when we destroy the SLAM system.
        % Without this, MATLAB will crash whenever this object is destroyed.

        function delete(this)
            vertices = this.graph.vertices();

            for v = 1 : length(vertices)
                this.graph.removeVertex(vertices{v});
            end
        end
        
        % Recommend if an optimization is a good idea. Based on an event,
        % some activities (e.g., such as loop closing) can have a very big
        % impact on the estimates. The logic we have here just recommends
        % an optimization if a fixed number of steps have been completed.
        
        function recommendation = recommendOptimization(this)
            
            % This is how to do it after every 100 steps
            recommendation = rem(this.stepNumber, ...
                this.recommendOptimizationPeriod) == 0;
        end
        
        % Set the value of how often recommend optimization should return
        % true
        function setRecommendOptimizationPeriod(this, newRecommendOptimizationPeriod)
            this.recommendOptimizationPeriod = newRecommendOptimizationPeriod;
        end
        
        % Return the current mean and covariance estimate of the robot.
        % This is only valid after optimization has been called.
        function [x, P] = platformEstimate(this)
            [xS, PS] = this.graph.computeMarginals(this.currentVehicleVertex);
            x=full(xS);
            P=full(PS);
            % disp(P)
        end
        
        % Returns the entire history of the platform estimates. Suppose
        % there are n vehicle vertices. T is a 1 by N dimensional vector of
        % timesteps. X is a 3 by N dimensional vector of vehicle state (x,
        % y, theta). P is a 3 by N dimensional vector where the nth column
        % are the diagonals from the covariance matrix.
        function [T, X, P] = platformEstimateHistory(this)
            
            % Extract the graph
            [xS, PS] = this.graph.computeMarginals();
            
            % Create the output array
            X = zeros(this.NP, this.vehicleVertexId);
            P = zeros(this.NP, this.vehicleVertexId);
            T = zeros(1, this.vehicleVertexId);
            
            % Copy the outputs over
            for v = 1 : this.vehicleVertexId
                idx = this.vehicleVertices{v}.hessianIndex();
                
                T(v) = this.vehicleVertices{v}.time();
                
                % Copy the estimate into the array. If the vertices is
                % fixed (conditioned), its estimate is okay. The covariance
                % is not explicitly defined, but has a value of zero.
                % Therefore we fill this manually.
                if (isempty(idx) == true)
                    X(:, v) = this.vehicleVertices{v}.estimate();
                    P(:, v) = zeros(3, 1);
                else
                    X(:, v) = full(xS(idx));
                    P(:, v) = full(diag(PS(idx, idx)));
                end
            end
        end
        
        % Return the means and covariances of the landmark estimates. These
        % are only valid after optimization has been called.
        function [x, P, landmarkIds] = landmarkEstimates(this)
            
            landmarkVertices = values(this.landmarkIDStateVectorMap);
            
            numberOfLandmarks = length(landmarkVertices);
            
            landmarkIds = NaN(1, numberOfLandmarks);
            x = NaN(this.NL, numberOfLandmarks);
            P = NaN(this.NL, this.NL, numberOfLandmarks);
            
            [xS, PS] = this.graph.computeMarginals();
            
            for l = 1 : numberOfLandmarks
                landmarkIds(l) = landmarkVertices{l}.landmarkId();
                idx = landmarkVertices{l}.hessianIndex();
                x(:, l) = full(xS(idx));
                if (isempty(idx == true))
                    P(:, :, l) = zeros(3, 3);
                else
                    P(:, :, l) = full(PS(idx, idx));
                end
            end
        end
        
        % We overload the optimize method so that you can add additional
        % logic here
        function chi2 = optimize(this, maximumNumberOfOptimizationSteps)
            
            % Remove the prediction edges if requested.
            if (this.removePredictionEdgesFromGraph == true)
                this.deleteVehiclePredictionEdges();
            end

            
            
            % Now call the actual optimizer. Let it handle the default if
            % no steps are specified.
            if (nargin > 1)
                chi2 = optimize@minislam.slam.SLAMSystem(this, ...
                    maximumNumberOfOptimizationSteps);
            else
                chi2 = optimize@minislam.slam.SLAMSystem(this);
            end
            
            %Q3b: Prune graph if pruning is enabled.
            if (this.prune == true)
                this.pruneGraph();
            end
            
        end
        
        function setRemovePredictionEdges(this, removeEdges, keepFirst)
            this.removePredictionEdgesFromGraph = removeEdges;
            this.keepFirstPredictionEdge = keepFirst;
            
        end
        
        %Q3b: Set pruning on or off
        function setPruneOn(this,pruneOn)
            this.prune = pruneOn;
        end
    end
    


    % These are the methods you will need to overload
    methods(Access = protected)
        
        % Handle the initial condition
        
        function handleInitialConditionEvent(this, event)
            
            % Create the first vertex, set its estimate to the initial
            % value and add it to the graph.
            this.currentVehicleVertex = drivebot.graph.VehicleStateVertex(this.currentTime);
            this.currentVehicleVertex.setEstimate(event.data);
            this.graph.addVertex(this.currentVehicleVertex);
            
            % Set the book keeping for this initial vertex.
            this.vehicleVertexId = 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
            
            % If the covariance is 0, the vertex is known perfectly and so
            % we set it as fixed. If the covariance is non-zero, add a
            % unary initial prior condition edge instead. This adds a soft
            % constraint on where the state can be.
            if (det(event.covariance) < 1e-6)
                this.currentVehicleVertex.setFixed(true);
            else
                initialPriorEdge = drivebot.graph.InitialPriorEdge();
                initialPriorEdge.setMeasurement(event.data);
                initialPriorEdge.setInformation(inv(event.covariance));
                initialPriorEdge.setVertex(this.currentVehicleVertex);
                this.graph.addEdge(initialPriorEdge);
            end
        end
        
        function handleNoPrediction(~)
            % Nothing to do
        end
        
        function handleHeartbeatEvent(this, ~)
            % Nothing to do
        end
        
        function handlePredictToTime(this, time, dT)

            % Create the next vehicle vertex and add it to the graph
            
            this.currentVehicleVertex = drivebot.graph.VehicleStateVertex(time);
            
            % Q1b:
            % Implement prediction code here
            odometry=this.u;
            this.odoHistory = [this.odoHistory;odometry'];
            this.odoTime = [this.odoTime,time];
            Q = this.uCov;
            omegaQ = inv(Q);

            prevVehicleVertex = this.vehicleVertices{this.vehicleVertexId}; 
            
            % Setting up new process model edge
            processModelEdge = drivebot.graph.VehicleKinematicsEdge(dT);
            % Connecting Vehicle state veterx of previous time step with
            % state vertex of current timestep
            processModelEdge.setVertex(1, prevVehicleVertex);
            processModelEdge.setVertex(2, this.currentVehicleVertex);
            processModelEdge.setMeasurement(odometry);
            processModelEdge.setInformation(omegaQ);
            processModelEdge.initialize();

            % Adding process model edge and vehicle state vertex of current
            % timestep to the graph
            this.graph.addEdge(processModelEdge);
            this.graph.addVertex(this.currentVehicleVertex);

            % Storing the process model edge to be used for Q3
            this.numProcessModelEdges = this.numProcessModelEdges + 1;
            this.processModelEdges{this.numProcessModelEdges} = processModelEdge;

            % Bump the indices
            this.vehicleVertexId = this.vehicleVertexId + 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
        end
        
        function handleGPSObservationEvent(this, event)

            % Q1d:
            % Create a GPS measurement edge and add it to the graph
            % warning('drivebotslam:handlegpsobservationevent:unimplemented', ...
            %     'Implement the rest of this method for Q1c.');

            % Creates a new GPSMeasurement Edge
            gpsMeasurementEdge = drivebot.graph.GPSMeasurementEdge(this.configuration.gpsPositionOffset);

            % Connect the edge to the current vehicle state vertex
            gpsMeasurementEdge.setVertex(1, this.currentVehicleVertex);
            gpsMeasurementEdge.setMeasurement(event.data);
            gpsMeasurementEdge.setInformation(inv(event.covariance));
            
            % Add edge to the graph
            this.graph.addEdge(gpsMeasurementEdge);
        end
        
        function handleCompassObservationEvent(this, event)
            
            % Q1c
            % Create a compass measurement edge and add it to the graph
            compassMeasurementEdge = drivebot.graph.CompassMeasurementEdge(this.configuration.compassAngularOffset);
            compassMeasurementEdge.setVertex(1, this.currentVehicleVertex);
            compassMeasurementEdge.setMeasurement(event.data);
            compassMeasurementEdge.setInformation(inv(event.covariance));
            this.graph.addEdge(compassMeasurementEdge);
        end
        
        function handleLandmarkObservationEvent(this, event)

            % Iterate over all the landmark measurements
            for l = 1 : length(event.landmarkIds)
                
                % Get the landmark vertex associated with this measurement.
                % If necessary, a new landmark vertex is created and added
                % to the graph.
                [landmarkVertex, newVertexCreated] = this.createOrGetLandmark(event.landmarkIds(l));
                z = event.data(:, l);

                % Q2b:
                % Complete the implementation
                % warning('drivebotslamsystem:handlelandmarkobservationevent:unimplemented', ...
                %     'Implement the rest of this method for Q2b.');

                % Construct LandmarkRangeBearing Edge
                landmarkRangeBearingEdge = drivebot.graph.LandmarkRangeBearingEdge();
                % Connect the edge with current vehicle vertex edge and
                % landmark edge
                landmarkRangeBearingEdge.setVertex(1,this.currentVehicleVertex);
                landmarkRangeBearingEdge.setVertex(2,landmarkVertex);
                landmarkRangeBearingEdge.setMeasurement(z);
                landmarkRangeBearingEdge.setInformation(inv(event.covariance));
                if newVertexCreated
                    % Initialize the landmark if it is the first time seeing
                    % the landmark
                    landmarkRangeBearingEdge.initialize()
                end
                % Add edge to the graph
                this.graph.addEdge(landmarkRangeBearingEdge);
            end
        end
        
        function deleteVehiclePredictionEdges(this)

            % Q3a:            
            % warning('drivebotslam:deletevehiclepredictionedges:unimplemented', ...
            %     'Implement the rest of this method for Q3a.');

            if ~this.keepFirstPredictionEdge
                % If removing all, then start at the beginning of the cells
                start = 1;
            elseif this.keepFirstPredictionEdge
                % If removing all but first, then start at the second of the cells
                start = 2;
            end

            % Iterate through stored process model edges and remove the
            % edges

            disp(["Before:",this.numProcessModelEdges])
            for v = start : this.numProcessModelEdges
                    this.graph.removeEdge(this.processModelEdges{v});
            end

            this.numProcessModelEdges = start - 1;

            disp(["After:",this.numProcessModelEdges])

            % % Code Verification that the issue arises when the initial state
            % % vertex isn't connect to rest of the graph
            % if this.numProcessModelEdges > 0
            %     this.graph.removeEdge(this.processModelEdges{1});
            % end
        end
        
        function pruneGraph(this)
            %Q3b
            %Iterating through graph to prune edges and vertices with high
            %scale-invariant density.

            if this.prune
                %Extracting all stored edges and vertices of the graph
                allEdges = this.graph.edges();
                allVertices = this.graph.vertices();
                numV = length(allVertices);
                numE = length(allEdges);
                              
                sid = zeros(numV,1);

                for v = 1:numV
                    vehicleVerticeIdx(v) = isa(allVertices{v},"drivebot.graph.VehicleStateVertex");
                end

                %Extracting platform estimate history and landmark
                %estimates to determine positions
                [T_v, X_v, P_v] = this.platformEstimateHistory();
                [x_l, P_l, landmarkIds] = this.landmarkEstimates();
                i = 1;
                l = 1;

                %To compute state-invariant densities of all vertices, the
                %platform estimates and landmark estimates are first combined
                %into a single array in the order that they appear in the
                %graph
                for v = 1:numV
                    if vehicleVerticeIdx(1,v) == true
                        X(:,v) = X_v(:,i);
                        %Ensuring not to prune edges associated to turns
                        if mod(X_v(3,i),pi/4) > pi/10
                            X(:,v) = [inf;inf;inf];
                        end
                        i = i+1;
                    else
                        X(:,v) = [x_l(:,l);0];
                        l = l+1;
                    end
                end
                
                %Computing state-invariant densities for each vertex,
                %taking into account only its 20 closest points to reduce
                %computational cost
                for v = 2:numV
                    for i = -10:10
                        if mod(v-i+numV,numV) ~= 0
                            if norm(X(:,mod(v-i+numV,numV))-X(:,v)) ~= 0
                                sid(v) = sid(v) + 1/norm(X(:,mod(v-i+numV,numV))-X(:,v));
                            end
                        end
                    end
                    sid(v) = sid(v)/pi;
                end

                %Pruning edges associated with vertices with the highest
                %state-invariant densities, until 20% of edges have been
                %pruned.
                while length(this.graph.edges()) > 0.8*numE
                    [~,pruneV] = max(sid);
                    sid(pruneV) = -inf;
                    if isa(allVertices{pruneV},"drivebot.graph.VehicleStateVertex")
                        edge = allVertices{pruneV}.edges();
                        for e = 1:length(edge)
                            if isa(edge{e},"drivebot.graph.LandmarkRangeBearingEdge")
                                this.graph.removeEdge(edge{e});
                            end
                        end
                    end
                end
            %Initialising optimisation after pruning is complete    
            this.graph.initializeOptimization(this.validateGraphOnInitialization);
            end

        
        end
        
        % This method returns a landmark associated with landmarkId. If a
        % landmark exists already, it is returned. If it does not exist, a
        % vertex is created and is added to the graph.
        function [landmarkVertex, newVertexCreated] = createOrGetLandmark(this, landmarkId)

            
            % If the landmark exists already, return it
            if (isKey(this.landmarkIDStateVectorMap, landmarkId) == true)
                landmarkVertex = this.landmarkIDStateVectorMap(landmarkId);
                newVertexCreated = false;
                return
            end
            
            fprintf('Creating landmark %d\n', landmarkId);
            
            % Create the new landmark add it to the graph
            landmarkVertex = drivebot.graph.LandmarkStateVertex(landmarkId);
            this.landmarkIDStateVectorMap(landmarkId) = landmarkVertex;
            
            this.graph.addVertex(landmarkVertex);
            
            newVertexCreated = true;
        end
        
        function storeStepResults(this)
            % Nothing
        end
        
    end
end
