function plotter(plots,plot_heading,results)

if ismember("OptTime",plots)
% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{:}.vehicleStateTime, results{:}.optimizationTimes', '*')
ylabel("Optimization Time (s)")
xlabel("Vehicle State Time (s)")
title("Optimization times")
saveas(gcf, plot_heading+'_opt_time', 'png');
hold on
end

if ismember("Errors",plots)
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{:}.vehicleStateTime, results{:}.vehicleStateHistory'-results{:}.vehicleTrueStateHistory')
xlabel("Vehicle State Time (s)")
ylabel("Error")
legend("X Error","Y Error","Phi Error")
title("State Error")
saveas(gcf, plot_heading+'_error', 'png');
hold on
end

if ismember("Covariance",plots)
% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{:}.vehicleStateTime, results{:}.vehicleCovarianceHistory')
ylabel("Covariance")
xlabel("Vehicle State Time (s)")
legend("X Variance","Y Variance","Phi Variance")
title("State Covariances")
saveas(gcf, plot_heading+'_covariance', 'png');
hold on
end

if ismember("Chi2",plots)
% Plot chi2
minislam.graphics.FigureManager.getFigure('Vehicle Chi Squared');
clf
plot(results{:}.chi2Time, results{:}.chi2History')
ylabel("Chi Squared")
xlabel("Vehicle State Time (s)")
title("Vehicle Chi Squared")
saveas(gcf, plot_heading+'_chi2', 'png');
hold on
end

end