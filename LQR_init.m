%% NOTES FROM SS
% close all; clear; clc; format compact;
% M = .1;
% m = 0.1;
% l = 0.5;
% % alpha = 9.8 / (l*(M+m) -1);
% % beta = 1 / (l*(M+m) - 1);
% % gamma = (M+m)*alpha;
% % % 
% alpha = -10.32;
% beta = -1.0582;
% gamma = -1.141;
% 
% % Not sure why he got the above values, but that's where we're at
% 
% A = [ 0 0 1 0; 0 0 0 1; 0 -alpha 0 0; 0 gamma 0 0];
% B = [ 0 0 beta*l -beta]';
% C = [0 1 0 0];
% D = 0;
% sys = ss(A, B, C, D);
% 
% eig(A); %eigenvals have two at origin, and two at oscillating poles with real of zero
% M = ctrb(A, B);
% ctrbRank = rank(M); % checks controllability of this stuff)
% N = obsv(A, C);
% obsvRank = rank(N); % The arm is totally observable, but the cart is NOT fully observable
% % We assumed that theta would NOT impact x1
% 
% % If we make the C matrix...
% C = [1 0 0 0; 0 1 0 0]; % It creates 2 outputs
% N = obsv(A, C);
% obsvRank = rank(N); % Now the rank is 4. We want to monitor position theta
% % This means we can get our angle theta just by knowing x. All states
% % affect output x. If we only measure the cart, then we can control
% % everything.
% 
% %{
%  Designing a system to stabilize this thing
% where should we place poles?
% - We'll try all of them at -3
% %}
% 
% P = [-3 -3 -3 -3];
% sys = ss(A, B, C, D);
% %Do acker to find the gains
% K = acker(A,B,P);
% % K = place(A,B,P); %place returns an error because P is rank 1 ( make one of them -3.1 or something like that)
% 
% Acl = A-B*K;
% CLsys = ss(Acl, B, C, D);
% step(CLsys) 
% hold on
% % this thing goes backwards first to stay stable
% 
% % We're really trying to continually balance this thing. That implies an
% % LQR controller. We don't want to define poles, we want to put penalties
% % on our states. We want theta to be near zero, and we don't really care
% % about zero. 
% 
% % We'll say that the penalty happens at 5 degrees (which is good enough to
% % make our small angle approximation valid). The bigger we make the Q, the
% % bigger we make the penalty. We don't want it to be zero. We don't care as
% % much about our x, as that isn't our goal.
% 
% % definining a Q
% % We want x WITHIN a half meter, and theta within 5.7 deg (.1 rad). We don't
% % care about velocity of the cart or of angular acceleration. We can't
% % define what direction the penalty due to the squaring and the fact that
% % it will no longer be continuous. 
% penalties = [ 1/(.5^2), 1 / (.1^2), 0, 0];
% Q = diag(penalties);
% % Defining R We'll say that the max input is 2 Newtons. We don't really
% % know this one, so we're making it up.
% 
% R = [1 / (2^2)];
% 
% Klqr = lqr(sys, Q, R); 
% % We have gains for all states, but we aren't directly observing all
% % states. We can either buy more sensors and directly observe them, or
% % build an observer and make it work. 
% 
% Acllqr = A - B*Klqr;
% eig(Acllqr); % We only care about these poles when we have an observer
% lqrsys = ss(Acllqr, B, C, D);
% step(lqrsys)
% 


%% 
close all; clear; clc; format compact; 