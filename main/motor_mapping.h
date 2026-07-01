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
static const MotorInfluence DEFAULT_MOTOR_INFLUENCE[4] = {
    //  TR         BR         TL         BL
    // roll, pitch, yaw
    {+1.0f, -1.0f, 0.0f}, // Top Right
    {+1.0f, +1.0f, 0.0f}, // Bottom Right
    {-1.0f, -1.0f, 0.0f}, // Top Left
    {-1.0f, +1.0f, 0.0f}, // Bottom Left
};

#endif // MOTOR_MAPPING_H
