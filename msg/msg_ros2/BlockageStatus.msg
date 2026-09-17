# Blockage/dirt counter for hesai JT128 and OT128
# Reports per sector (splitting the full scan into n_elev_bins vertical and
# n_azim_bins horizontal slices) for a total of n_elev_bins*n_azim_bins sectors.
# The number of sectors are configured in the parameters for the driver.
# Arrays are flattened row-major: index = elev_bin * n_azim_bins + azim_bin
# Elevation bins are equal-angle slices of [elev_min_deg, elev_max_deg], from
# the sensor's angle correction. Azimuth bins are equal slices of 0..360.

std_msgs/Header header
# Reports sensor model as detected by the blockage detector. Only OT128 or JT128
string sensor_model

uint16  n_elev_bins
uint16  n_azim_bins
float32 elev_min_deg
float32 elev_max_deg
uint32  frames_accumulated    # frames counted into this message

# Always true for JT128, Only true if on for OT128
bool detection_enabled

# Total points checked per sector, for scaling the amount of errors if desired
uint32[] samples

# Contamination Status Hit Counts (OT128 & JT128) per sector
# OT128: 0=Normal, 1=Blocked, 2=Proximity, 3=No Return
# JT128: dirtyLevel 0-3 from weightFactor
uint32[] code_0_hits
uint32[] code_1_hits
uint32[] code_2_hits
uint32[] code_3_hits

# JT128 Environmental Noise Hit Counts
# From noiseLevel (0-63) in weightFactor
uint32[] moderate_noise_hits  # levels 22-42
uint32[] high_noise_hits      # levels 43-63
