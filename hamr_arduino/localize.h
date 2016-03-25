#include "math.h"
#include "Arduino.h"

typedef struct location {
  // Current position
  float x;
  float y;
  float theta;

  // Current velocity and angular rate
  float ds;
  float dtheta;

  // Constructor
  location() { 
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    ds = 0.0;
    dtheta = 0.0;
  }

  // Update location using encoder integration
  void update(int encoder_counts_left, float encoder_counts_right, 
              float ticks_per_rev, float wheel_rad, float wheel_dist) {
    
    // Find number of wheel rotations from encoder counts
    float right_rot = encoder_counts_right / ticks_per_rev; 
    float left_rot = encoder_counts_left / ticks_per_rev;
    
    // Calculate displacement
    ds = wheel_rad * (right_rot + left_rot) / 2.0; // linear displacement
    dtheta = (wheel_rad / wheel_dist) * (right_rot - left_rot);  // angular displacement

    // Update variables
    float dx = ds * cos(theta);
    float dy = ds * sin(theta);
    x = x + dx;
    y = y + dy;
    theta = theta + dtheta;

    Serial.print("x: ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print("y: ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print("theta (deg): ");
    Serial.print(180 * theta / PI);
    Serial.print(", ");
    Serial.print("dtheta (deg/s): ");
    Serial.print(180 * dtheta / PI);
    Serial.print("\n");
  }
};

