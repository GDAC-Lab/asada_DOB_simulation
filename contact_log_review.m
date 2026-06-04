function contact_log_review(varargin)
project_directory = fileparts(mfilename('fullpath'));
implementation_directory = fullfile(project_directory, 'matlab', 'analysis');
shared_directory = fullfile(project_directory, 'matlab', 'shared');
addpath(implementation_directory, shared_directory);
cleanup_handler = onCleanup(@() rmpath(implementation_directory, shared_directory)); %#ok<NASGU>
fprintf('contact_log_review.m is a legacy sample review entrypoint. For project-owned review workflows, prefer analysis/matlab in the external project repository template.\n');
contact_log_review_impl(varargin{:});
end
