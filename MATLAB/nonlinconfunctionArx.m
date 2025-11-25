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
% time instant (k, k+1, ..., k+Nhor). 
% 
%   yPast = [y1(k-1) ... y1(k-na)
%            y2(k-1) ... y2(k-na)
%                    ...            
%            ynscn(k-1) ... ynscn(k-na)] initial conditions for all scenarios
%   uPast = [u(k-1) ... u(k-nd) ... u(k-nd-nb+1)] initial conditions for 
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

yPast = repmat(yPast,Nscen,1);

for k=1:NhorU
    uSamples = [u(:,k), uPast];
    for i=1:Nscen
        yScen = sysVec{i}.computeOutput(yPast(i,:), uSamples);
        c = [c; yScen - ymax; -yScen + ymin];
        yPast(i,:) = [yScen, yPast(i, 1:end-1)];
    end    
    uPast = [uSamples(1), uPast(1:end-1)];
end

for k=NhorU+1:Nhor
    uSamples = [uPast(1), uPast];
    for i=1:Nscen
        yScen = sysVec{i}.computeOutput(yPast(i,:), uSamples);
        c = [c; yScen - ymax; -yScen + ymin];
        yPast(i,:) = [yScen, yPast(i, 1:end-1)];
    end    
    uPast = [uSamples(1), uPast(1:end-1)];
end


end
