function varargout = getParameters(object, varargin)

switch nargin
    case 1
        varargout{1} = object.na;
        varargout{2} = object.nb;
        varargout{3} = object.nd;
        varargout{4} = object.theta;
        varargout{5} = object.Ts;
    case 2
        switch lower(varargin{1})
            case 'na'
                varargout{1} = object.na;
            case 'nb'
                varargout{1} = object.nb;
            case 'nd'
                varargout{1} = object.nd;
            case 'theta'
                varargout{1} = object.theta;
            case 'ts'
                varargout{1} = object.Ts;
            otherwise
                error('Unrecognized input argument name.');
        end
    otherwise
        error('Wrong number of inputs.');
end

end