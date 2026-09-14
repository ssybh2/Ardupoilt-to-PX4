px4_add_module(
MODULE modules__l1_adaptive_control
MAIN l1_adaptive_control
COMPILE_FLAGS
SRCS
L1AdaptiveControl.cpp
L1AdaptiveControl.hpp
L1SourceConfig.hpp
ACRLTrajectories.cpp
ACRLTrajectories.hpp
TrajectoryGenerator.cpp
TrajectoryGenerator.hpp
GeometricController.cpp
GeometricController.hpp
L1AdaptiveAugmentation.cpp
L1AdaptiveAugmentation.hpp
MotorMixer.cpp
MotorMixer.hpp
DEPENDS
px4_work_queue
)

px4_add_unit_gtest(
SRC TrajectoryGeneratorTest.cpp
EXTRA_SRCS TrajectoryGenerator.cpp ACRLTrajectories.cpp
)

px4_add_unit_gtest(
SRC GeometricControllerTest.cpp
EXTRA_SRCS GeometricController.cpp
)

px4_add_unit_gtest(
SRC L1AdaptiveAugmentationTest.cpp
EXTRA_SRCS L1AdaptiveAugmentation.cpp
)

px4_add_unit_gtest(
SRC MotorMixerTest.cpp
EXTRA_SRCS MotorMixer.cpp
)
