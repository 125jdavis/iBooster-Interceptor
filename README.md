# iBooster-Interceptor
Signal interceptor to allow calibration of iBooster brake pedal assist curve

Current strategy: read only S2 input duty to estimate pedal travel, then generate both output signals with complementary duty cycles (`S2 + S4 = 100`).
