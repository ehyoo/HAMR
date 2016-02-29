typedef struct location {
  float x;
  float y;
  float theta;

  // Constructor
  location() { 
    x = 0.0;
    y = 0.0;
    theta = 0.0;
  }

  // Update location using encoder integration
  void update(int encoder_counts_left, float encoder_counts_right, 
              float ticks_per_rev, float wheel_rad, float wheel_dist) {
    
    // Find number of wheel rotations from encoder counts
    float right_rot = encoder_counts_right / ticks_per_rev; 
    float left_rot = encoder_counts_left / ticks_per_rev;
    
    // Calculate displacement
    float delta_s = wheel_rad * (right_rot + left_rot) / 2.0; // linear displacement
    float delta_theta = wheel_rad / wheel_dist * (right_rot - left_rot);  // angular displacement

    // Update variables
    x = x + delta_s * cos(theta);
    y = y + delta_s * sin(theta);
    theta = theta + delta_theta;
  }
};

