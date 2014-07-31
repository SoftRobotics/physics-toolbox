network = csvread('circle/velocities.csv');

x = 1:1:length(network(:,1));

cc=hsv(length(network(1,:)));

figure;
hold on
for i = 1:length(network(1,:))
 a = network(:,i);
 %z = (a - mean(a))/std(a);
 z = zscore(a);
 plot(x(:), z(:), 'color',cc(i,:))
end

%figure;
%hold on
%for i = 1:length(network(1,:))
 %a = network(:,i);
 %plot(x(:), a(:), 'color',cc(i,:))
%end