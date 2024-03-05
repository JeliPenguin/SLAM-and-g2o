% This edge encodes a 3D range bearing measurement.
%
% The measurement is in spherical polar coordinates

% Jacobian from https://github.com/petercorke/robotics-toolbox-matlab/blob/master/RangeBearingSensor.m

classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
    
    methods(Access = public)
    
        function this = LandmarkRangeBearingEdge()
            this = this@g2o.core.BaseBinaryEdge(2);
        end
        
        function initialize(this)
            % Q2b:
            % Complete implementation
            % warning('landmarkrangebearingedge:initialize:unimplemented', ...
            %     'Implement the rest of this method for Q1b.');

            % Initialize landmark based on the inverse observation model
            x = this.edgeVertices{1}.estimate();
            xk = x(1);
            yk = x(2);
            phik = x(3);
            r = this.z(1);
            beta = this.z(2);

            xi = xk + r*cos(beta+phik);
            yi = yk + r*sin(beta+phik);
            this.edgeVertices{2}.setEstimate([xi;yi]);
        end
        
        function computeError(this)

            % Q2b:
            % Complete implementation

            % Compute error based on landmark observation model
            x = this.edgeVertices{1}.estimate();
            dx = this.edgeVertices{2}.estimate() - x(1:2);
            
            this.errorZ(1) = norm(dx) - this.z(1);
            this.errorZ(2) = g2o.stuff.normalize_theta(atan2(dx(2), dx(1)) - x(3) - this.z(2));

        end
        
        function linearizeOplus(this)
            % Q2b:
            % Complete implementation
            % warning('landmarkrangebearingedge:linearizeoplus:unimplemented', ...
            %     'Implement the rest of this method for Q1b.');

            % Compute Jacobian of the error with respect to the state
            x = this.edgeVertices{1}.estimate();
            dx = this.edgeVertices{2}.estimate() - x(1:2);
            r = norm(dx);
            
            this.J{1} = ...
                [-dx(1)/r -dx(2)/r 0;
                dx(2)/r^2 -dx(1)/r^2 -1];

            this.J{2} = - this.J{1}(1:2, 1:2);
        end        
    end
end