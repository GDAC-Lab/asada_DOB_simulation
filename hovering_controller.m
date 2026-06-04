function hovering_controller(varargin)
project_directory = fileparts(mfilename('fullpath'));
implementation_directory = fullfile(project_directory, 'matlab', 'controllers');
shared_directory = fullfile(project_directory, 'matlab', 'shared');
addpath(implementation_directory, shared_directory);
cleanup_handler = onCleanup(@() rmpath(implementation_directory, shared_directory)); %#ok<NASGU>
fprintf('hovering_controller.m is a legacy top-level sample entrypoint. For project work, prefer project-owned files under controllers/matlab or experiments/matlab in the external project repository.\n');
hovering_controller_impl(varargin{:});
end