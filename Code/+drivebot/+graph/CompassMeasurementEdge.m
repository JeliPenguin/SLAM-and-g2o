classdef CompassMeasurementEdge < g2o.core.BaseUnaryEdge
   
    % Q1c:
    % This implementation contains a bug. Identify the problem
    % and fix it as per the question.

    properties(Access = protected)
        
        compassAngularOffset;
        
    end
    
    methods(Access = public)
    
        function this = CompassMeasurementEdge(compassAngularOffset)
            this = this@g2o.core.BaseUnaryEdge(1);
            this.compassAngularOffset = compassAngularOffset;
        end
        
        function computeError(this)
            x = this.edgeVertices{1}.estimate();
            % this.errorZ = x(3) + this.compassAngularOffset - this.z;
            
            % Q1c:Normalise angle between [-\pi,\pi], the bug that wasn't fixed
            % before
            this.errorZ = g2o.stuff.normalize_theta(this.z - x(3) - this.compassAngularOffset);
        end
        
        function linearizeOplus(this)
            % this.J{1} = [0 0 1];

            this.J{1} = [0 0 -1];
        end        
    end
end