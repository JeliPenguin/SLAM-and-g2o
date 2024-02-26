classdef ObjectPolarMeasurementEdge < g2o.core.BaseUnaryEdge
   
    properties(Access = protected)
        % The x,y and theta of the sensor
        sensorPose;
    end
    
    methods(Access = public)
    
        function this = ObjectPolarMeasurementEdge()
            this = this@g2o.core.BaseUnaryEdge(2);
            this.sensorPose = zeros(3, 1);
        end
        
        function setSensorPose(this, sensorPose)
            this.sensorPose = sensorPose;
        end
        
        function computeError(this)
            xs = this.sensorPose(1);
            ys = this.sensorPose(2);
            phi_s = this.sensorPose(3);
            xk = this.edgeVertices{1}.x(1);
            yk = this.edgeVertices{1}.x(3);
            rk = sqrt((xk-xs)^2+(yk-ys)^2);
            beta_k = atan2((yk-ys),(xk-xs)) - phi_s;

            this.errorZ = this.z - [rk;beta_k];
        end
        
        function linearizeOplus(this)
            xs = this.sensorPose(1);
            ys = this.sensorPose(2);
            phi_s = this.sensorPose(3);
            xk = this.edgeVertices{1}.x(1);
            yk = this.edgeVertices{1}.x(3);
            rk = sqrt((xk-xs)^2+(yk-ys)^2);
            beta_k = atan2((yk-ys),(xk-xs)) - phi_s;

            dxr = rk * (xk-xs);
            dyr = rk * (yk-ys);
            this.J{1} = [dxr,0,dyr,0
                         -(yk-ys)/(1+(xk-xs)^2),0,1/((xk-xs)*(1+(yk-ys)^2)),0];
        end        
    end
end