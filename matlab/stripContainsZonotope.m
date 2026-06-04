%% stripContainsZonotope
% This evaluates if the strip given as input contains a zonotope.
% 
% Many assumptions about inputs: strip is given as intersection of
% halfspaces (in the form of a matrix and a vector A*x <= b), zonotope is 
% given as center and generators matrix. Method is assumed to be 'exact', 
% tolerance is assumed 100*eps.
% 
% Inputs:
% - sA: strip matrix
% - sb: strip known term
% - Zc: zonotope center
% - ZGen: zonotope generator matrix
% 
% Outputs:
% - res: 1 if strip contains zonotope, 0 otherwise
% 

function res = stripContainsZonotope(sA, sb, Zc, ZGen)

% * Sets default values for 
% - method: method for computation ('exact' or 'approx', or any method
%           name specific to a certain set representation; see the
%           documentation of the corresponding contains_ function, e.g.,
%           zonotope/contains_ )
% - tol: tolerance
% - maxEval: maximal number of iterations for optimization-based methods.
%            See the corresponding contains_ (e.g., zonotope/contains_)
%            function for more details
% 

% * Gives maxEval a value based on the zonotope's number of generators

% tol = eps;
% maxEval = max(500, 200*size(G, 2));

% * Checks input arguments

% * Checks if strip can be represented in "fullspace" representation, or is
% numeric. Then, computes Z = compact(Z) (removes redundancies in the
% representation of the set, minimal representation of the original set).

% Checks if the zonotope is now empty, or if strip is, or if Z is a point.
% ZGen = nonzeroFilter(ZGen, tol);

% * Now that the trivial cases have been dealt with, checks if P is 1D.
% It's not, so call aux_exactParser since method is exact.

% * If Z is a zonotope, the containment check is guaranteed to be exact

% * If the outer body (strip) is in halfspace representation, the check is
% made via support functions Function aux_contains_P_Hpoly(strip, Z, tol,
% scalingToggle) does this. scalingToggle was zero since scalingToggle = 

% * aux_contains_P_Hpoly checks if Z is a polytope and in vertex
% representation (it's not) for a faster method. 

% * priv_equalityToInequality is called to rewrite all equality constraints
% as inequality constraints (easy, assumed).

% * aux_contains_P_Hpoly then loops over all (inequality) constraints (only
% 2 in our case). res is set true. supportFunc_(Z, A(i,:)', 'upper') is 
% called.

% * supportFunc_(Z, dir, type, varargin) calculates the upper or lower 
% bound of a zonotope along a certain direction. It computes a proportion
% of the projection of the zonotope's center and every generator on the
% vector normal to the hyperplane defining one of the halfspaces that
% contitute the strip. Then it returns the sum of the projection of the
% center and the absolute values of all generators' ones.

% * If this sum is greater than the value of the constraint (hyperplane
% offset) even beyond the given (absolute OR relative) tolerance, then
% strip does NOT contain Z. If scalingToggle is false, we return.
res = true;
cproj = sA(1,:) * Zc;
Gproj = sA(1,:) * ZGen;
Gproj = sum(abs(Gproj));

if Gproj + cproj > sb(1) || Gproj - cproj > sb(2)
    res = false;
end

return;

end