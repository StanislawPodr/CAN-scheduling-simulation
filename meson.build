project('canSim', 'c', default_options: ['warning_level=2', 'buildtype=debug'])

subdir('algorithm')
subdir('simulation')

executable(
    'simulation',
    simulation_src,
    algorithm_src,
    include_directories: algorithm_inc,
)
