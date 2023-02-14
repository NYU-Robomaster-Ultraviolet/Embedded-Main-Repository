#include "ballistics_subsystem.hpp"
#include "modm/math/geometry/angle.hpp"
#include <math.h>


namespace ballistics{
void BallisticsSubsystem::initialize(){
    pastTime = tap::arch::clock::getTimeMilliseconds();
    yawMotor.initialize();
    yawMotor.setDesiredOutput(0);
    pitchMotor.initialize();
    pitchMotor.setDesiredOutput(0);
    uint16_t currentPitchEncoder = pitchMotor.getEncoderUnwrapped();
    uint16_t currentYawEncoder = yawMotor.getEncoderUnwrapped();
    startingPitch = wrappedEncoderValueToRadians(currentPitchEncoder);
    startingYaw = wrappedEncoderValueToRadians(currentYawEncoder);
}

void BallisticsSubsystem::refresh(){ 
    if(yawMotor.isMotorOnline()){
        uint16_t currentYawEncoder = yawMotor.getEncoderUnwrapped();
        currYaw = wrappedEncoderValueToRadians(currentYawEncoder);

    }
    if(pitchMotor.isMotorOnline()){
        uint16_t currentPitchEncoder = pitchMotor.getEncoderUnwrapped();
        currPitch = wrappedEncoderValueToRadians(currentPitchEncoder);

    }
}

inline float BallisticsSubsystem::wrappedEncoderValueToRadians(int64_t encoderValue) {
    return (M_TWOPI * static_cast<float>(encoderValue)) / tap::motor::DjiMotor::ENC_RESOLUTION;
}

void BallisticsSubsystem::make_angles(float currentPitch, float currentYaw, std::vector<std::vector<float>> our1_info, vector<float> &pitchYawVec){

    float yaw = atan2(our1_info[0][0], -our1_info[0][1]) * (180/M_PI);
    float pitch = atan2(sqrt(pow(our1_info[0][0],2) + pow(our1_info[0][1],2)), -our1_info[0][2]) * (180/M_PI);

    pitchYawVec.push_back(pitch);
    pitchYawVec.push_back(yaw);

}


void BallisticsSubsystem::calc_pitch_yaw(vector<vector<float>> target_info, vector<vector<float>> our1_info,float velocity,float barrel_len,float& returnPitch,float& returnYaw){
    float x = target_info[0][0] - our1_info[0][0];
    float y = target_info[0][1] - our1_info[0][1];
    float z = target_info[0][2] - our1_info[0][2];
    
    float vx = target_info[1][0] - our1_info[1][0];
    float vy = target_info[1][1] - our1_info[1][1];
    float vz = target_info[1][2] - our1_info[1][2];
    
    float ax = target_info[1][0] - our1_info[1][0];
    float ay = target_info[1][1] - our1_info[1][1];
    float az = target_info[1][2] - our1_info[1][2];
    
    vector<float> possible_ts;
    possible_ts.push_back(0.25*(ax*ax + ay*ay + az*az));
    possible_ts.push_back(vx*ax + vy*ay + vz*az);
    possible_ts.push_back(vx*vx + vy*vy + vz*vz + ax*x + ay*y + az*z - velocity*velocity);
    possible_ts.push_back(2*(-velocity*barrel_len + x*vx + y*vy + z*vz));

    //TODO with libraries cause its cringe otherwise
    //derive roots of each elements in possible ts(representing points in time where it hits)
    //select the smallest non negative time(represented by i)
    int item = 0;
    float time = possible_ts[item];

    float final_x = x + vx*time + 0.5*target_info[2][0]*(time*time);
    float final_y = y + vy*time + 0.5*target_info[2][0]*(time*time);
    float final_z = z + vz*time + 0.5*target_info[2][0]*(time*time);

    //get sin value of the ptich/yaw

    float final_x_hat = final_x - 0.5*0*(time*time);
    float final_y_hat = final_y - 0.5*0*(time*time);

    /*
    GET PITCH AND YAW USING MATH FUNCTIONS
    yaw_offset_rad = -math.pi / 2
    yaw_offset_rad = math.pi / 2
    yaw_offset_rad = math.atan2(final_y_hat, final_x_hat)
    pitch_rad = math.pi / 2 + pitch_offset_rad
    yaw_rad = math.pi / 2 + yaw_offset_rad

    CONVERT TO DEGREES

    */
   float pitch = 0.0f;
   float yaw = 0.0f;

   returnPitch = pitch;
   returnYaw = yaw;
}


}//namespace ballistics