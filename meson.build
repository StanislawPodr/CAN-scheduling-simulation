project('canSim', 'c', default_options: ['warning_level=2', 'buildtype=debug', 'c_std=c23'])

subdir('algorithm')
subdir('simulation')

executable(
    'can-simulation',
    simulation_src,
    algorithm_src,
    include_directories: algorithm_inc,
)
