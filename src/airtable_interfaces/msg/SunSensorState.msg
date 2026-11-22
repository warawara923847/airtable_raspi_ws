# One nanoSSOC-D60 observation, expressed in the sensor frame (header.frame_id).

# Reported by the sensor (decimal, as in its specification).
uint8 ERROR_NONE=0
uint8 ERROR_NOT_ENOUGH_RADIATION=10
uint8 ERROR_ONLY_ALBEDO=11
uint8 ERROR_BOTH_SUN_AND_ALBEDO=12
uint8 ERROR_OUT_OF_FIELD_OF_VIEW=13
# Detected by the host; the angles and intensity are NaN for these except INVALID_ANGLE.
uint8 ERROR_TRANSMISSION_FAILED=20
uint8 ERROR_FRAME_SIZE_MISMATCH=21
uint8 ERROR_CHECKSUM_MISMATCH=22
uint8 ERROR_INVALID_ANGLE=23

std_msgs/Header header

# True when the angles can be used and sun_vector holds a direction: error_code is NONE,
# ONLY_ALBEDO or BOTH_SUN_AND_ALBEDO. Read error_code for the quality. False without light
# (NOT_ENOUGH_RADIATION), out of the field of view, for unknown codes and for host errors.
bool valid
uint8 error_code

float64 alpha_deg
float64 beta_deg
float64 intensity_percent

# Unit vector toward the sun: normalized (tan(alpha), tan(beta), 1). All zero when not valid.
geometry_msgs/Vector3 sun_vector
