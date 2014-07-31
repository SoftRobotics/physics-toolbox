learningEndEffector = csvread('./devArm/circle/effector.csv.learning');
endEffector = csvread('./devArm/circle/effector.csv.working');


c = hsv(length(endEffector(1:35000,1)));

fh = figure;
set(fh,'DefaultAxesFontSize',15,'DefaultAxesFontName','helvetica');
hold;
scatter(endEffector(1:35000, 1), endEffector(1:35000,2), 5, c);
colormap hsv;
ch = colorbar('location', 'EastOutside');
caxis([0.0 416.6]);
yLabel = get(ch, 'YTickLabel');
yLabel = strcat(yLabel,{' s'});
set(ch, 'YTickLabel', yLabel);
%plot(endEffector(16194:16289,1), endEffector(16194:16289,2), 'r', 'LineWidth', 3);
%plot(endEffector(20574:20694,1), endEffector(20574:20694,2), 'r', 'LineWidth', 3);
plot(learningEndEffector(:,1), learningEndEffector(:,2), 'k', 'LineWidth', 3);
axis equal;
hold;

%scatter(endEffector(:, 1), endEffector(:,2), 5, c);
%plot(endEffector(16000:18000,1),endEffector(16000:18000,2), '.b', endEffector(16816:16846,1),endEffector(16816:16846,2), '.r');
%plot(endEffector(16816:16846,1),endEffector(16816:16846,2), '.r');

debug = csvread('./devArm/circle/debug.csv.working');
angles = csvread('./devArm/circle/angles.csv');

x=1:1:6283;

fh = figure;
set(fh,'DefaultAxesFontSize',15,'DefaultAxesFontName','helvetica');
plot( x(:), debug(2*6283+701:3*6283+700,1), x(:), angles(:,1));

fh = figure;
set(fh,'DefaultAxesFontSize',15,'DefaultAxesFontName','helvetica');
plot( x(:), debug(2*6283+701:3*6283+700,2), x(:), angles(:,2));

%% Clear temporary variables
