function formation_log_review(varargin)
project_directory = fileparts(mfilename('fullpath'));
implementation_directory = fullfile(project_directory, 'matlab', 'analysis');
addpath(implementation_directory);
cleanup_handler = onCleanup(@() rmpath(implementation_directory)); %#ok<NASGU>
fprintf('formation_log_review.m is a legacy sample review entrypoint. For project-owned review workflows, prefer analysis/matlab in the external project repository template.\n');
formation_log_review_impl(varargin{:});
end