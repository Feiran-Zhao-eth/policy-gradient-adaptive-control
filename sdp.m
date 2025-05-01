function [K] = sdp(n,T, Q,R,lambda,X0,U0,X1)
% It solves the direct certainty-equivalence LQR problem with a regularizer
% see the paper: https://arxiv.org/abs/2503.02985
% Note that without regularization, direct LQR with covariance
% parameterization is equivalent to indirect certainty-equivalence LQR

D0 = [U0;X0];
X0_bar = X0*D0'/T;
X1_bar = X1*D0'/T;
U0_bar = U0*D0'/T;
Cov = D0*D0'/T;
cvx_begin sdp quiet
    variable L(m,m) symmetric
    variable S(n,n) symmetric
    variable W(m+n,n) 
    variable Regu(m+n,m+n) symmetric
    minimize( trace(Q*S) + trace(R*L) + lambda*trace(Regu*Cov)) % the last term is the robustness-prompting regularizer
    subject to
        S == X0_bar*W
        [S-eye(n), X1_bar*W; W'*X1_bar', S] >= 0
        [L, U0_bar*W; W'*U0_bar', S] >= 0
        [Regu, W; W', S] >= 0
cvx_end
K = U0_bar*W/S;
end