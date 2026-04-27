function [c, ceq] = nonlinconfunctionArx(sysVec, u, yPast, uPast, Nhor, NhorU, Nscen, umax, umin, ymax, ymin)
% nonlinconfunctionArx      Computes the constraint satisfaction of the MPC
% problem. 
% 
% They consist in input and output constraints, in the form of
%   umin <= u <= umax
%   ymin <= y <= ymax
%   <=> ymin <= [ySamples, uSamples] * theta <= ymax
% 
% Constraints for all scenarios must be considered and applied for each
% time instant (k+1, k+2, ..., k+Nhor). 
% 
%   yPast = [y1(k) ... y1(k-na+1)
%            y2(k) ... y2(k-na+1)
%                    ...            
%            ynscn(k) ... ynscn(k-na+1)] initial conditions for all scenarios
%   uPast = [u(k-1) ... u(k-nd) ... u(k-nd-nb+2)] initial conditions for 
% all scenarios
%   u = [u(k+NHorU-1) ... u(k)] optimization variable(s)

% Control input, optimization variable
u = reshape(u, 1, NhorU); % [u(k+NhorU-1) ... u(k)]

% Initialize constraints
c = [];
ceq = [];

% Linear constraints on input for each time instant
% c = [c; [eye(NhorU); -eye(NhorU)] * u' - [repmat(umax, NhorU, 1); -repmat(umin, NhorU, 1)] ];
% 
% Linear constraints on output
%   ymin <= y <= ymax
%   ymin <= Phi * theta <= ymax

for k=1:NhorU
    uSamples = [u(:,k), uPast];
    for l=1:Nscen+1
        yScen = sysVec{l}.computeOutput(yPast(l,:), uSamples);
        c = [c; yScen - ymax; -yScen + ymin];
        yPast(l,:) = [yScen, yPast(l, 1:end-1)];
    end    
    uPast = [uSamples(1), uPast(1:end-1)];
end

for k=NhorU+1:Nhor
    uSamples = [uPast(1), uPast];
    for l=1:Nscen+1
        yScen = sysVec{l}.computeOutput(yPast(l,:), uSamples);
        c = [c; yScen - ymax; -yScen + ymin];
        yPast(l,:) = [yScen, yPast(l, 1:end-1)];
    end    
    uPast = [uSamples(1), uPast(1:end-1)];
end


end
