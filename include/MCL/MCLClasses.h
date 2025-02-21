#include "vector"
#include "auto.h"
#pragma once

#define FIELD_SIZE_MM 3657.6

namespace MCL {

// Error Codes
enum ErrorCodes { // Default error code
    E_MCLERR = -1,
    E_MCLINVALID,
    E_MCLNOPARTICLES,
    E_MCLSUCCESS = 0,
    E_MCLLOOPEXIT
};
// Distance sensors Definitions
enum DistancesPosition { // Enum to declare distance side of the robot.
    E_DISTANCE_LEFT,
    E_DISTANCE_RIGHT,
    E_DISTANCE_FRONT,
    E_DISTANCE_BACK
};

class DistanceSensors {
private:
    pros::Distance *Port;
    double TrackedDistance;
    double TrackingOffset;
    double TrackingErrorSmall;
    double TrackingErrorLarge;
    double TrackingErrorCutoff;
    DistancesPosition DistanceDirection;
    double OffsetX;
    double OffsetY;

public:
    DistanceSensors(pros::Distance &Sensor, DistancesPosition DistDirection, double DistanceOffsetX, double DistanceOffsetY, double DistanceErrorSmall, double DistanceErrorLarge, double DistanceErrorLargeCutoff);
    ~DistanceSensors();
    int ChangeOffSetX(double Change);
    int ChangeOffSetY(double Change);
    double GetOffSetX();
    double GetOffSetY();
    double GetTrackingError();
    DistancesPosition GetDistancePosition();
    double CalculateMeasureDistance(double Heading);
    double SimulateDistance(lemlib::Pose particlePose);
};

class Particle {
private:
    lemlib::Pose ParticlePose;
    double SimulatedDistance;
    double ParticleWeight;

public:
    Particle();
    ~Particle();
    int UpdateWeight(double Weight);
    double SimulateDistance(std::vector<DistanceSensors>& Sensors);
    int PoseMove(lemlib::Pose ChangeInPose, lemlib::Pose Error);
    double GetWeight();
    lemlib::Pose GetPose();
};

class MCLController {
private:
    lemlib::Pose OldPose;
    lemlib::Pose ChangeInPose;
    std::vector<Particle> Particles;
    std::vector<DistanceSensors> Sensors;
    lemlib::Pose MovementNoise;

    lemlib::Pose EstimatePose();
    int ParticleGeraration(int numParticles);
    lemlib::Pose CalculateChangedPose();
    int ResampleParticles();
    int UpdateRobotPose();
    int MCLLoop();

    bool IsValidSensorReading(DistanceSensors& sensor, lemlib::Pose particlePose);
    lemlib::Pose EstimatePoseUsingSensors(lemlib::Pose particlePose);

public:
    MCLController();
    ~MCLController();
};

} // namespace MCL