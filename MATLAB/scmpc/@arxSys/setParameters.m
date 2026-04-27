function object = setParameters(object, varargin)

% object = setParameters(object, NAME, VAL)

if nargin ~= 3
    error('Wrong number of inputs.');
end

name = varargin{1};
val = varargin{2};

switch lower(name)
    case 'na'
        object.na = val;

    case 'nb'
        object.nb = val;

    case 'nd'
        object.nd = val;
        
    case 'theta'
        object.theta = val;

    otherwise
        error('Unrecognized name input.');

end