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

        vertex = this.edgeVertices{1};

        x = vertex.estimate();
        deltax = this.xyOffset(1);
        deltay = this.xyOffset(2);
        c = cos(x(3));
        s = sin(x(3));

        dx = deltax*s + deltay*c;
        dy = -deltax*c+ deltay*s;

        this.J{1} = [-1 0 dx;
                    0 -1 dy];

        end
    end
end
