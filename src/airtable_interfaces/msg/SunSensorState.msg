# One nanoSSOC-D60 observation, expressed in the sensor frame (header.frame_id).

# Reported by the sensor.
uint8 ERROR_NONE=0
uint8 ERROR_NOT_ENOUGH_RADIATION=16
uint8 ERROR_ONLY_ALBEDO=17
uint8 ERROR_BOTH_SUN_AND_ALBEDO=18
uint8 ERROR_OUT_OF_FIELD_OF_VIEW=19
# Detected by the host; the angles and intensity are NaN for these except INVALID_ANGLE.
uint8 ERROR_TRANSMISSION_FAILED=20
uint8 ERROR_FRAME_SIZE_MISMATCH=21
uint8 ERROR_CHECKSUM_MISMATCH=22
uint8 ERROR_INVALID_ANGLE=23

std_msgs/Header header

# True only when error_code is ERROR_NONE and sun_vector holds a direction.
bool valid
uint8 error_code

float64 alpha_deg
float64 beta_deg
float64 intensity_percent

# Unit vector toward the sun: normalized (tan(alpha), tan(beta), 1). All zero when not valid.
geometry_msgs/Vector3 sun_vector
