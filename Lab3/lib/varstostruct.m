function s = varstostruct(varargin)

s = struct();
for i=1:length(varargin),
    s.(varargin{i}) = evalin('caller', varargin{i});
end;