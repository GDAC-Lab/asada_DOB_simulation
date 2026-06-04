function jetson_compute_budget_review(varargin)
project_directory = fileparts(mfilename('fullpath'));
implementation_directory = fullfile(project_directory, 'matlab', 'analysis');
shared_directory = fullfile(project_directory, 'matlab', 'shared');
addpath(implementation_directory, shared_directory);
cleanup_handler = onCleanup(@() rmpath(implementation_directory, shared_directory)); %#ok<NASGU>
fprintf('jetson_compute_budget_review.m reviews single-UAV controller compute time and packet age using the network metadata view.\n');
contact_log_review_impl('network', varargin{:});
end