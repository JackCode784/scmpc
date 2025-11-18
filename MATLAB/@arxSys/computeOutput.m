function y = computeOutput(object, ySeq, uSeq)
% computeOutput     Computes the output of the ARX model.
% 
% y = computeOutput(object, yPast, uPast)
% Returns the system output computed as 
%   y(k) = a_1 y(k-1) + ... + a_na y(k-na) +
%           + b_1 u(k-nd) + ... + b_nb u(k-nk-nb+1) =
%        = Phi * theta
% 
% yPast, uPast contain the past input/output samples and the current input,
% ordered as
%   yPast = [y(k-1) ... y(k-na)]
%   uPast = [u(k) ... u(k-nd) ... u(k-nk-nb+1)]
% 
% yPast, uPast can be matrices, in which case this function returns a
% (column) vector of the outputs produced with the given values of 
% input/output past and current samples.

ySeq = reshape(ySeq, [], object.na);
uSeq = uSeq(end-nb+1:end);
uSeq = reshape(uSeq, [], object.nb);
y = [ySeq, uSeq] * object.theta;

end