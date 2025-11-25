function [A, B, C, D] = arx2stateSpace(arxObject)
% arx2stateSpace    This returns the A, B, C, D matrices of the state space
% representation of the ARX system given as input. 
% 
% The state of this new system is given by the past output samples
%   x1(k) := y(k-1)
%   x2(k) := y(k-2)
%   ...
%   xna(k) := y(k-na)
% Then the output can be expressed as:
%   y(k) = a_1 x1(k) + a_2 x2(k) + ... + a_na xna(k) + 
%           + b_1 u(k-nd) + ... + b_nb u(k-nd-nb+1)
% 


end