A = cat(2, zeros(3,3), ones(3))
A = cat(1, A, zeros(3,6))


% B = cat(1, zeros(3), diag([6.993e4, 6.993e4, 3.46e4]))
% B = cat(2, zeros(6,3), B)
B = [0, 0, 0, 6.993e4, 6.993e4, 3.46e4]'

C = cat(2, diag([1, 1, 1]), zeros(3))
D = zeros(3,6)';

M = ctrb(A, B);
rank(M)
obsv(A, C)
% N = obsv(A, B);
rank(N)
% sys = ss(A, B, C, D)
[T, D] = eig(A, diag(B))