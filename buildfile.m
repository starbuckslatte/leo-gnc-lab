function plan = buildfile
%BUILDFILE Static code check, then the full test suite.
%   buildtool            run default tasks (check, test)
%   buildtool test       run tests only
import matlab.buildtool.tasks.CodeIssuesTask
import matlab.buildtool.tasks.TestTask

plan = buildplan(localfunctions);

plan("check") = CodeIssuesTask(["src", "tests", "missions", "scripts"]);

plan("test") = TestTask("tests", ...
    IncludeSubfolders = true, ...
    SourceFiles = "src", ...
    TestResults = "results/test-results.xml");

plan.DefaultTasks = ["check", "test"];
end
