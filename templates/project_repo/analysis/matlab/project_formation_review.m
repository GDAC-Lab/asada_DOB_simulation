function project_formation_review(varargin)
project_directory = fileparts(fileparts(fileparts(mfilename('fullpath'))));
simulator_root = fullfile(project_directory, 'external', 'mujoco_wheeled_uav_simulator');
analysis_directory = fullfile(simulator_root, 'matlab', 'analysis');
addpath(analysis_directory);
cleanup_handler = onCleanup(@() rmpath(analysis_directory)); %#ok<NASGU>
formation_log_review_impl(varargin{:});
end