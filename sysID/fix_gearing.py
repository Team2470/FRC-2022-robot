#! /usr/bin/env python3

import json
import sys
from pathlib import Path

if len(sys.argv) == 1:
    print("Usage: {} sysid_data.json".format(sys.argv[0]))
    sys.exit(1)

# Load in SysID data
print(f"sysid data file correct: {sys.argv[1]}")
sysid_data = None
with open(sys.argv[1]) as f:
    sysid_data = json.load(f)

# Data format: https://github.com/wpilibsuite/sysid/blob/main/docs/data-collection.md#drivetrain
# | Index | Value        |
# | ----- | ------------ |
# | 0     | timestamp    |
# | 1     | l voltage    |
# | 2     | r voltage    |
# | 3     | l position   |
# | 4     | r position   |
# | 5     | l velocity   |
# | 6     | r velocity   |
# | 7     | angle        |
# | 8     | angular rate |
#
# Note that all positions and velocities should be in rotations of the output and rotations/sec of the output
# respectively. If there is a gearing between the encoder and the output, that should be taken into account.

SYSID_TIMESTAMP  = 0
SYSID_L_VOLTAGE  = 1
SYSID_R_VOLTAGE  = 2
SYSID_L_POSITION = 3
SYSID_R_POSITION = 4
SYSID_L_VELOCITY = 5
SYSID_R_VELOCITY = 6
SYSID_L_ANGLE    = 7
SYSID_R_ANGLE    = 8

# This is how SysID processes
# https://github.com/wpilibsuite/sysid/blob/main/sysid-projects/drive/src/main/cpp/Robot.cpp#L50-L53
#   double cpr = m_json.at("counts per rotation").get<double>();
#   double gearingNumerator = m_json.at("gearing numerator").get<double>();
#   double gearingDenominator = m_json.at("gearing denominator").get<double>();
#   double gearing = gearingNumerator / gearingDenominator;

# This is how we configured our encoder gearing
# 1:26.04
# From config.json
# {
#   ...
#   "gearing denominator": 26.04,
#   "gearing numerator": 1,
#   ...
# }

# When encoders are setup this is how the overall CPR (counters per revolution) is calculated:
# https://github.com/wpilibsuite/sysid/blob/main/sysid-library/src/main/cpp/generation/SysIdSetup.cpp#L167
#   double combinedCPR = cpr * gearing;
# With the gearing ratio flipped, it reduced how many encoder counters occur for each turn of the wheel.
# When in reality for 1 wheel revolution the encoder turns 26.04 times, so the combinedCPR needs to be
# 2048 * (26.04/1) = 53,329.92, and not 2048 * (1/26.04) = 78.65.

# Position is measured in rotations
# Velocity is measured in rotations per second

# This is how the SysID converts the encoder value to rotations
# https://github.com/wpilibsuite/sysid/blob/main/sysid-library/src/main/cpp/generation/SysIdSetup.cpp#L144
#   position = [=] { return talonController->GetSelectedSensorPosition() / cpr; };
#   rate = [=] {
#     return talonController->GetSelectedSensorVelocity() / cpr /
#            0.1;  // Conversion factor from 100 ms to seconds
#   };

# What the calculation should be:
# cpr = 2048 * (26.04/1) = 53,329.92
# rotations = position / (53,329.92)

# What we ended up with the incorrect gear ratio:
# cpr = 2048 * (1/26.04) = 53,329.92
# rotations = position / (78.65)

# So, in order to correct the incorrect gearing we need to divide the gearing by 26.04 twice. Basically we want to make
# the denominator 78.65 turn into 53,329.92.
# 1. The first divide gets our position/velocity measurements in reference to the builtin Falcon encoder
# 2. The second divide gets our position/velocity measures in reference to the rotations of the robot wheels.

gearing = 26.04
for test_name in ["fast-backward", "fast-forward", "slow-backward", "slow-forward"]:
    for datapoint in sysid_data[test_name]:

        # Fix up the metrics from this datapoint
        for metric in [SYSID_L_POSITION, SYSID_R_POSITION, SYSID_L_VELOCITY, SYSID_R_VELOCITY]:
            datapoint[metric] = datapoint[metric] / gearing / gearing

# Write out correct sysid data file
output_file_path = Path(sys.argv[1])
output_file_path = output_file_path.with_stem(f"{output_file_path.stem}-fixed")
print(f"Writing fixed sysid data file to {output_file_path}")

with open(output_file_path, 'w') as f:
    json.dump(sysid_data, f, indent=4, sort_keys=True)
