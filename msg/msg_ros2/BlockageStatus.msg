std_msgs/Header header
string sensor_model

# Contamination Status Hit Counts (OT128 & JT128)
# OT128: 0=Fault, 1=Blocked, 2=Proximity, 3=No Return
# JT128: dirtyLevel 0-3 from weightFactor
uint32[128] code_0_hits
uint32[128] code_1_hits
uint32[128] code_2_hits
uint32[128] code_3_hits

# JT128 Environmental Noise Hit Counts
# From noiseLevel (0-63) in weightFactor
uint32[128] moderate_noise_hits  # e.g., levels 22-42
uint32[128] high_noise_hits      # e.g., levels 43-63
