function [max_err, end_err] = test_3axis0(dat_fn)

dat = load(dat_fn);
test = dat(:,9:11)

rotx = @(t) [1 0 0; 0 cos(t) -sin(t) ; 0 sin(t) cos(t)] ;
roty = @(t) [cos(t) 0 sin(t) ; 0 1 0 ; -sin(t) 0  cos(t)] ;
rotz = @(t) [cos(t) -sin(t) 0 ; sin(t) cos(t) 0 ; 0 0 1] ;

% rotate <360,1080,720>*clock
R = deg2rad([360,1080,720]);
t = linspace(0,100,100);

rlist = zeros(100,3);
for i = 1:100
    dR = t(i) .* R;
    
    A = eye(3);
    A = rotx(dR(1)) * A;
    A = roty(dR(2)) * A;
    A = rotz(dR(3)) * A;
    
    r = rotationMatrixToVector(A);
    rlist(i,:) = r;
end

% hmm..
rlist(:,2) = -rlist(:,2);

err = vecnorm(rlist - test, 2, 2);
max_err = max(err);
end_err = err(end);

figure(1);
hold off;
plot(rlist);
hold on;
plot(test, 'x-');
grid on;

figure(2);
hold off;
plot(err);
grid on;

csvwrite('3axis0_gt.csv', rlist);
