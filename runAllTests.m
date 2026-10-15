%RUNALLTESTS Run every unit test in tests/ and report results.
setupPaths();
root = fileparts(mfilename('fullpath'));
results = runtests(fullfile(root, 'tests'), 'IncludeSubfolders', true);
disp(table(results));
assertSuccess(results);
