function plotter(plots,plot_heading,results)

if isempty(plots) || ismember("OptTime",plots)
% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{:}.vehicleStateTime, results{:}.optimizationTimes', '*')
ylabel("Optimization Time (s)")
xlabel("Vehicle State Time (s)")
title("Optimization times")
grid on
saveas(gcf, plot_heading+'_opt_time', 'png');
hold on
end

if isempty(plots) || ismember("Errors",plots)
minislam.graphics.FigureManager.getFigure('Errors');
clf
error = results{:}.vehicleStateHistory'-results{:}.vehicleTrueStateHistory';
plot(results{:}.vehicleStateTime, error(:,1:2))
xlabel("Vehicle State Time (s)")
ylabel("Error (m)")
legend("X Error","Y Error")
title("Vehicle Position Error")
grid on
saveas(gcf, plot_heading+'_position_error', 'png');
end

if isempty(plots) || ismember("Errors",plots)
minislam.graphics.FigureManager.getFigure('Heading Errors');
clf
error = results{:}.vehicleStateHistory'-results{:}.vehicleTrueStateHistory';
plot(results{:}.vehicleStateTime, error(:,3))
xlabel("Vehicle State Time (s)")
ylabel("Error (rad)")
legend("\Phi Error")
title("Vehicle Heading Error")
grid on
saveas(gcf, plot_heading+'_heading_error', 'png');
end

if isempty(plots) || ismember("Covariance",plots)
% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
cov = results{:}.vehicleCovarianceHistory';
plot(results{:}.vehicleStateTime, cov(:,1:2))
ylabel("Covariance (m^2)")
xlabel("Vehicle State Time (s)")
legend("X Variance","Y Variance")
title("Vehicle Position Covariances")
grid on
saveas(gcf, plot_heading+'_position_covariance', 'png');
hold on
end

if isempty(plots) || ismember("Covariance",plots)
% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Heading Covariances');
clf
cov = results{:}.vehicleCovarianceHistory';
plot(results{:}.vehicleStateTime, cov(:,3))
ylabel("Covariance (rad^2)")
xlabel("Vehicle State Time (s)")
legend("\Phi Covariance")
title("Vehicle Heading Covariances")
grid on
saveas(gcf, plot_heading+'_heading_covariance', 'png');
hold on
end

if isempty(plots) || ismember("Chi2",plots)
% Plot chi2
minislam.graphics.FigureManager.getFigure('Vehicle Chi Squared');
clf
plot(results{:}.chi2Time, results{:}.chi2History')
ylabel("Chi Squared")
xlabel("Vehicle State Time (s)")
title("Vehicle Chi Squared")
grid on
saveas(gcf, plot_heading+'_chi2', 'png');
hold on
end

end