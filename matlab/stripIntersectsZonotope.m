%% myIntersect
% This evaluates if the strip given as input has non-empty intersection
% with thezonotope given as center and generator matrix.
%
% Inputs:
% - strip: strip points set
% - center: zonotope center
% - G: zonotope generator matrix
%
% Outputs:
% - res: comparison result
%

function res = stripIntersectsZonotope(P, cZ)

% Reorders strip and Z, then calls reorderNumeric(Z, strip) (does nothing).

% There is a bit of confusion and possibly bugs about the isIntersecting
% function, since:
% - it assumes there exists a 'precedence' property of the objects Z and
% strip, which is absent;
% - from this it follows the function loops over calls to two different
% internal functions, both calling each other in a loop.
%
% It looks like the correct function which should implement the
% intersection between the strip and the zonotope with the 'exact' method
% is aux_isIntersecting_P_cZ(P, cZ), where P = strip, cZ = Z.

% res = aux_isIntersecting_P_cZ(P, cZ) checks if a polytope {x | H*x <= d, 
% He*x = de} and a constraint zonotope {x = c + G*beta | A*beta = b, beta 
% \in [-1,1]} intersect by solving the following linear program:
%
% min_{t,x,beta} t
%
% s.t. Hx - d <= t
%        He x = de
%   c + Gbeta = x
%      A beta = b
%        beta \in [-1,1]

H = P.A; d = P.b;
nrIneq_poly = size(H,1);
He = P.Ae; de = P.be;
nrEq_poly = size(He,1);

% center, generator matrix, and constraints of constrained zonotope
c = cZ.c; G = cZ.G;
A = []; b = [];
% dimension, number of generators, number of constraints
n = length(c);
nrGen = size(G,2);
nrEq_conZono = size(A,1);

% optimization variable is [t;x;beta] with length 1 + n + nrGen

% construct inequality constraints
%   Hx - d <= t      <=>  Hx <= t + d
%   beta \in [-1,1]  <=>  -beta <= 1, beta <= 1

% compute number of elements and minimum sparsity
numElem = (nrIneq_poly+2*nrGen)*(1+n+nrGen);
minNumZeros = 2*nrGen*(1+n) + nrIneq_poly*nrGen + 2*nrGen*(nrGen-1);
sparsity = minNumZeros / numElem;
% use sparse representation if beneficial
if sparsity > 0.5
    Aineq = [-ones(nrIneq_poly,1), H, sparse(nrIneq_poly,nrGen);
        sparse(nrGen,1+n), -speye(nrGen);
        sparse(nrGen,1+n), speye(nrGen)];
else
    Aineq = [-ones(nrIneq_poly,1), H, zeros(nrIneq_poly,nrGen);
        zeros(nrGen,1+n), -eye(nrGen);
        zeros(nrGen,1+n), eye(nrGen)];
end
bineq = [d;ones(2*nrGen,1)];

% construct equality constraints
% He*x = de
% x = c + G*beta  <=>  x - G*beta = c
% A*beta = b
Aeq = [[zeros(nrEq_poly,1),He,zeros(nrEq_poly,nrGen)];
    [zeros(n,1),eye(n),-G];
    [zeros(nrEq_conZono,1+n),A]];
beq = [de; c; b];

% construct objective function
% min t
f = [1;zeros(n+nrGen,1)];

% init linprog struct
problem.f = f;
problem.Aineq = Aineq;
problem.bineq = bineq;
problem.Aeq = Aeq;
problem.beq = beq;
problem.lb = [];
problem.ub = [];

% solve linear program
[~,val,exitflag] = CORAlinprog(problem);

% multiple cases:
% 1. constrained zonotope (and possibly polytope) is empty
%    => infeasible (dual unbounded)
% 2. polytope empty => max(y)>0
%     OR
%    no intersection point between non-empty polytope and non-empty cZ
% ELSE: error

% cannot become unbounded above (left to the reader as an exercise);
% also cannot become unbounded below: That implies there exists an
% x in the intersection, and since cZ is bounded, so is H*x-d,
% which is a lower bound on y

if exitflag == -2
    % 1. no feasible point was found
    res = false;
elseif exitflag == 1 % one could maybe include 3 and 0
    % 2. feasible point found, check if max(y) < 0 == val
    % tol? => use constraint tolerance (default: 1e-6)
    tol = 1e-6;
    res = val < tol;
elseif exitflag == -3
    % unbounded (because no inequality constraints in P) => feasible
    res = true;
else
    throw(CORAerror('CORA:solverIssue','linprog'));
end



end