function project_contact_review(varargin)
project_directory = fileparts(fileparts(fileparts(mfilename('fullpath'))));
simulator_root = fullfile(project_directory, 'external', 'mujoco_wheeled_uav_simulator');
analysis_directory = fullfile(simulator_root, 'matlab', 'analysis');
shared_directory = fullfile(simulator_root, 'matlab', 'shared');
addpath(analysis_directory, shared_directory);
cleanup_handler = onCleanup(@() rmpath(analysis_directory, shared_directory)); %#ok<NASGU>
contact_log_review_impl(varargin{:});
end