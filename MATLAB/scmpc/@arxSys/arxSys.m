classdef arxSys
    % arxSys    Linear time-invariant ARX dynamical system object
    %
    % This represents an ARX system in the form:
    %
    %       A(z) y(k) = B(z) u(k) + n(k)
    % <=>   (1 - a_1 z^-1 - ... - a_na z^-n_a) y(k) =
    %       = (0 + ... + 0 + b_1 z^-nd + ... + b_nb z^-(nk+nb-1) ) u(k) +
    %           + n(k)
    %
    % where z^-k is the delay operator of k steps. The above expression can
    % be equivalently written as
    %
    % y(k) = a_1 * y(k-1) + ... + a_na * y(k-na) + b_1 * u(k-nd) + ... +
    %        b_nb * u(k-nd-nb+1) + n(k)
    %      = Phi * theta + n(k)
    %
    % where k denotes the discrete-time instant, with sampling time Ts, Phi
    % is the (row) vector of past output and input values, theta is the
    % parameters vector as shown below.
    %
    % The other quantities are:
    % - y(k): system output at instant k (size ny)
    % - u(k): system input at instant k (size nu)
    % - na: number of output coefficients
    % - nb: number of input coefficients
    % - nd: system internal delay
    % - theta = [a_1; ...; a_na; b_1; ...; b_nb]: parameter vector
    % - n(k): gaussian white noise with zero mean and lambda^2 variance
    %
    % The expected value of the system output is easily computable as
    %
    % E{y(k)} = a_1 * y(k-1) + ... + a_na * y(k-na) + b_1 * u(k-nd) + ... +
    %        b_nb * u(k-nd-nb+1)
    %         = Phi * theta
    %
    % OBJ = arxSys() - empty arxSys object.
    % OBJ = arxSys(na, nb, nd, theta) - arxSys object without specified
    % sampling time.
    % OBJ = arxSys(na, nb, nd, theta, Ts) - arxSys object with specified
    % sampling time.

    % Properties
    properties (Access = public)
        theta = [];
        na = [];
        nb = [];
        nd = [];
        Ts = [];
    end

    methods

        % Constructor
        function model = arxSys(varargin)

            switch nargin
                case 0
                    model.na = [];
                    model.nb = [];
                    model.nd = [];
                    model.theta = [];
                case {4, 5}
                    model.na = varargin{1};
                    model.nb = varargin{2};
                    model.nd = varargin{3};

                    % Check dimensions
                    if model.na < 0
                        error('Number of output coefficients must be non-negative.');
                    end
                    if model.nb < 0
                        error('Number of output coefficients must be non-negative.');
                    end
                    if model.nd < 0
                        error('Number of output coefficients must be non-negative.');
                    end

                    model.theta = reshape(varargin{4}, [], 1);
                    if length(model.theta) ~= model.na + model.nb
                        error('Incorrect dimension of parameters vector.');
                    end

                    if nargin == 5
                        model.Ts = varargin{5};
                    else 
                        model.Ts = 0;
                    end

                otherwise
                    error('Wrong number of input arguments.');
            end
        end

        % Set methods
        object = setParameters(object, varargin);

        % Get methods
        varargout = getParameters(object, varargin);

        % Other methods
        y = computeOutput(object, yPast, uPast);

        % disp(object);

        % Realization of ARX model to state space
        % ssSys = realization(object);

    end

end