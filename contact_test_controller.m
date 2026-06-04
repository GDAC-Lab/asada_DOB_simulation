function contact_test_controller(varargin)
project_directory = fileparts(mfilename('fullpath'));
implementation_directory = fullfile(project_directory, 'matlab', 'experiments');
shared_directory = fullfile(project_directory, 'matlab', 'shared');
addpath(implementation_directory, shared_directory);
cleanup_handler = onCleanup(@() rmpath(implementation_directory, shared_directory)); %#ok<NASGU>
fprintf('contact_test_controller.m is a legacy sample entrypoint. For project-owned experiment orchestration, prefer experiments/matlab in the external project repository template.\n');
contact_test_controller_impl(varargin{:});
end
