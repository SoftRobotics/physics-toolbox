clear

angles = csvread('./devArm/circle/angles.csv');
readouts = csvread('./devArm/circle/output.csv');
weights = csvread('./devArm/circle/weights.csv');

%angles = angles*100000;

cutoff = 6284;

A = readouts(cutoff:end,:);

%adjust angles
ang = angles(mod((1:end)-mod(cutoff, length(angles(:,1)))-1, end)+1,:);
ang = repmat(ang,floor(length(A(:,1))/length(ang(:,1))+1),1);

%add noise
%A = A + 0*rand(size(A));

noiseAmplitude = 0.5;

if noiseAmplitude ~= 0
    noise = zeros(size(A));
    for i = 1:length(noise(:,1));
        if rand(1) > 0.95
            tmp = noiseAmplitude*rand(5, length(noise(1,:)));
            for j = 1:5
                noise(i,:) = tmp(j,:);
                i = i + 1;
                if i > length(noise(:,1))
                    break;
                end;
            end;
        end;
    end;
    A = A + noise;
end;

%w = pinv(A'*A)*A'*ang(1:length(A(:,1)),:);
%simpler syntax for least squares
w = A\ang(1:length(A(:,1)),:);

readouts = readouts(length(angles(:,1)):length(angles(:,1)) + length(angles(:,1)),:);

weights = transpose(weights);

oldtraj = readouts * w;
newtraj = readouts * w;

x = 1:1:length(angles(:,1));

%angles = angles.*1000;

fh = figure;
set(fh,'DefaultAxesFontSize',15,'DefaultAxesFontName','helvetica');
plot( x(:), oldtraj(1:length(angles(:,1)),1), x(:), angles(:,1), x(:), newtraj(1:length(angles(:,1)),1) );

fh = figure;
set(fh,'DefaultAxesFontSize',15,'DefaultAxesFontName','helvetica');
plot( x(:), oldtraj(1:length(angles(:,1)),2), x(:), angles(:,2), x(:), newtraj(1:length(angles(:,1)),2) ); 

csvwrite('./devArm/circle/weights.csv', w');