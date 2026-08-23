#ifndef MOTOR_MAPPING_H
#define MOTOR_MAPPING_H

// Readable, tweakable mapping from each motor to orientation axes.
// Coefficients are dimensionless, used as relative weights for distributing
// corrective effort across motors. Signs indicate how increasing a given
// motor's speed affects the axis (positive = increases that axis error).

typedef struct
{
    float roll;  // + means motor increase tends to roll right
    float pitch; // + means motor increase tends to pitch forward
    float yaw;   // + means motor increase tends to yaw clockwise (looking down)
} MotorInfluence;

// Order must match motor_id_t: TR, BR, TL, BL.
// Spins viewed from above (2026-08-23, XT60 = nose, A at top-right clockwise):
// TR=A CW, BR=B CCW (phase-wire swap), TL=D CCW, BL=C CW.
// A CCW motor's drag reaction torques the frame CW looking down => +yaw.
static const MotorInfluence DEFAULT_MOTOR_INFLUENCE[4] = {
    // roll, pitch, yaw
    {+1.0f, -1.0f, -1.0f}, // Top Right A (CW)
    {+1.0f, +1.0f, +1.0f}, // Bottom Right B (CCW)
    {-1.0f, -1.0f, +1.0f}, // Top Left D (CCW)
    {-1.0f, +1.0f, -1.0f}, // Bottom Left C (CW)
};

#endif // MOTOR_MAPPING_H
