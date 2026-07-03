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

// Default placeholder mapping for an X-quad; tune with mapping test logs.
// Order must match motor_id_t order: TR, BR, TL, BL.
// Yaw column from measured spin directions (2026-07-03 ESC config):
// TR+BL spin CCW, TL+BR spin CW (viewed from above). A CCW motor's drag
// reaction torques the frame CW looking down => positive yaw influence.
static const MotorInfluence DEFAULT_MOTOR_INFLUENCE[4] = {
    //  TR         BR         TL         BL
    // roll, pitch, yaw
    {+1.0f, -1.0f, +1.0f}, // Top Right (CCW)
    {+1.0f, +1.0f, -1.0f}, // Bottom Right (CW)
    {-1.0f, -1.0f, -1.0f}, // Top Left (CW)
    {-1.0f, +1.0f, +1.0f}, // Bottom Left (CCW)
};

#endif // MOTOR_MAPPING_H
