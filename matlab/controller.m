%% [Z, yHist, uHist, uOpt] = controller(yrefdig, ycurrdig, yHist, uHist, Z)
% This performs all SCMPC controller's operations.
% 
function [Z, yHist, uHist, uOpt] = controller(yrefdig, ycurrdig, yHist, uHist, Z, ymin, ymax, umin, umax)

yref = map(yrefdig, 0, 4095, ymin, ymax);
ycurr = map(ycurrdig, 0, 4095, ymin, ymax);

% uOpt = zeros(1,)

end