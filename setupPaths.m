function setupPaths()
%SETUPPATHS Add repository folders to the MATLAB path.
%   Run once per MATLAB session from the repository root.
root = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root, 'src')));   % includes src/utils (parent of +att)
addpath(fullfile(root, 'missions'));
addpath(fullfile(root, 'scripts'));
fprintf('leo-gnc-lab paths added (root: %s)\n', root);
end
