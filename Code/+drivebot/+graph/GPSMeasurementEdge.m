classdef GPSMeasurementEdge < g2o.core.BaseUnaryEdge
   
    properties(Access = protected)
        
        xyOffset;
        
    end
    
    methods(Access = public)
    
        function this = GPSMeasurementEdge(xyOffset)
            this = this@g2o.core.BaseUnaryEdge(2);
            this.xyOffset = xyOffset;
        end
        
        function computeError(this)

	    % Q1d:
        % Implement the code
        % warning('gpsmeasurementedge:computeerror:unimplemented', ...
        %         'Implement the rest of this method for Q1d.');

        % Compute error using the given GPS Observation Model
        
        vertex = this.edgeVertices{1};

        x = vertex.estimate();

        c = cos(x(3));
        s = sin(x(3));
        M = [c -s;
             s c ;];

        this.errorZ = (this.z(1:2)-x(1:2)-M*this.xyOffset);

        end
        
        function linearizeOplus(this)

	    % Q1d:
        % Implement the code
        % warning('gpsmeasurementedge:lineareizeoplus:unimplemented', ...
        %         'Implement the rest of this method for Q1d.');

        % Compute Jacobian of error with respect to the state

        this.J{1} = [-1 0 0;
                    0 -1 0];

        end
    end
end
