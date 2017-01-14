# Contains stamp of when measurment was taken and frame id of encoder
Header header

# Distance (signed) reported from Encoder::GetDistance(), in units used in call to SetDistancePerPulse()
int32 distance

# Rate (signed) reported from Encoder::GetRate(), in units used in call to SetDistancePerPulse() / second
float64 rate
