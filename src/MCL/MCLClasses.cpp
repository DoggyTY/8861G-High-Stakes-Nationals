#include "main.h"
#include "MCL/MCLClasses.h"
#include <cmath>
#include <vector>
#include <random>
#include <numeric>
#include <algorithm>

namespace MCL {

// Gaussian function for weight calculation
double Gaussian(double error, double sigma) {
    return exp(-0.5 * (error * error) / (sigma * sigma)) / (sigma * sqrt(2 * M_PI));
}

double GenerateRandomWeight(double maxWeight) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, maxWeight);
    return dist(gen);
}


int DistanceSensors::ChangeOffSetX(double Change) {
    OffsetX = Change;
    return E_MCLSUCCESS;
}

int DistanceSensors::ChangeOffSetY(double Change) {
    OffsetY = Change;
    return E_MCLSUCCESS;
}

double DistanceSensors::GetOffSetX() {
    return OffsetX;
}

double DistanceSensors::GetOffSetY() {
    return OffsetY;
}

double DistanceSensors::GetTrackingError() {
    if (TrackedDistance <= TrackingErrorCutoff) {
        return TrackingErrorSmall;
    } else {
        return TrackedDistance * TrackingErrorLarge;
    }
}

DistancesPosition DistanceSensors::GetDistancePosition() {
    return DistanceDirection;
}

double DistanceSensors::CalculateMeasureDistance(double Heading) {
    double sensorReading = Port->get();  // Raw sensor reading
    double adjustedDistance = 0.0;

    // Normalize heading to [0, 360)t
    while (Heading < 0) Heading += 360;
    while (Heading >= 360) Heading -= 360;

    // Convert heading to radians
    double headingRad = Heading * (M_PI / 180.0);

    // Adjust distance based on sensor position
    switch (DistanceDirection) {
        case E_DISTANCE_LEFT:
            adjustedDistance = FIELD_SIZE_MM - (sensorReading - OffsetX) - (cos(headingRad) * OffsetX);
            break;
        case E_DISTANCE_RIGHT:
            adjustedDistance = (sensorReading + OffsetX) + (cos(headingRad) * OffsetX);
            break;
        case E_DISTANCE_FRONT:
            adjustedDistance = FIELD_SIZE_MM - (sensorReading - OffsetY) - (sin(headingRad) * OffsetY);
            break;
        case E_DISTANCE_BACK:
            adjustedDistance = (sensorReading + OffsetY) + (sin(headingRad) * OffsetY);
            break;
        default:
            return E_MCLERR;  // Error case
    }

    return adjustedDistance;
}

double DistanceSensors::SimulateDistance(lemlib::Pose particlePose) {
    double particleX = particlePose.x;
    double particleY = particlePose.y;
    double particleTheta = particlePose.theta * (M_PI / 180.0); // Convert to radians

    double sensorX = particleX + (cos(particleTheta) * OffsetX) - (sin(particleTheta) * OffsetY);
    double sensorY = particleY + (sin(particleTheta) * OffsetX) + (cos(particleTheta) * OffsetY);

    double expectedDistance = 0.0;

    switch (DistanceDirection) {
        case E_DISTANCE_LEFT:
            expectedDistance = sensorX;  // Distance to left wall
            break;
        case E_DISTANCE_RIGHT:
            expectedDistance = FIELD_SIZE_MM - sensorX; // Distance to right wall
            break;
        case E_DISTANCE_FRONT:
            expectedDistance = FIELD_SIZE_MM - sensorY; // Distance to front wall
            break;
        case E_DISTANCE_BACK:
            expectedDistance = sensorY; // Distance to back wall
            break;
        default:
            return E_MCLERR;  // Error case
    }

    return expectedDistance;
}

Particle::Particle() : ParticlePose({0, 0, 0}), SimulatedDistance(0), ParticleWeight(1.0) {}

Particle::~Particle() {}

int Particle::UpdateWeight(double Weight) {
    this->ParticleWeight = Weight;
    return E_MCLSUCCESS;
}

double Particle::SimulateDistance(std::vector<DistanceSensors>& Sensors) {
    static std::random_device rd;
    static std::mt19937 gen(rd());

    double minValidDistance = FIELD_SIZE_MM; // Start with max field size
    bool validReadingFound = false;

    for (DistanceSensors& sensor : Sensors) {
        double expectedDistance = sensor.SimulateDistance(ParticlePose);

        double baseError = sensor.GetTrackingError();
        std::normal_distribution<double> errorDist(baseError, std::abs(baseError) * 0.1);
        double randomizedError = errorDist(gen);
        double simulatedReading = expectedDistance + randomizedError;

        if (simulatedReading < expectedDistance) {
            continue; // Ignore sensor as it is blocked
        }

        minValidDistance = std::min(minValidDistance, simulatedReading);
        validReadingFound = true;
    }

    return validReadingFound ? minValidDistance : E_MCLERR;
}

int Particle::PoseMove(lemlib::Pose ChangeInPose, lemlib::Pose Error) {
    static std::random_device rd;
    static std::mt19937 gen(rd());  // Random number generator

    // Convert percentage noise into absolute values
    double noiseX_mean = ChangeInPose.x * Error.x;
    double noiseY_mean = ChangeInPose.y * Error.y;
    double noiseTheta_mean = ChangeInPose.theta * Error.theta;

    // Define Gaussian distributions for noise
    std::normal_distribution<double> noiseX(noiseX_mean, std::abs(noiseX_mean) * 0.1);  // 10% variation
    std::normal_distribution<double> noiseY(noiseY_mean, std::abs(noiseY_mean) * 0.1);
    std::normal_distribution<double> noiseTheta(noiseTheta_mean, std::abs(noiseTheta_mean) * 0.1);

    // Apply movement with converted noise
    ParticlePose.x += ChangeInPose.x + noiseX(gen);
    ParticlePose.y += ChangeInPose.y + noiseY(gen);
    ParticlePose.theta += ChangeInPose.theta + noiseTheta(gen);

    return E_MCLSUCCESS;
}

double Particle::GetWeight() {
    return ParticleWeight;
}

lemlib::Pose Particle::GetPose() {
    return ParticlePose;
}

MCLController::MCLController() : OldPose({0, 0, 0}), ChangeInPose({0, 0, 0}), MovementNoise({0.05, 0.05, 0.005}) {}

MCLController::~MCLController() {}

lemlib::Pose MCLController::EstimatePose() {
    float estimatedX = 0.0, estimatedY = 0.0;
    int validReadings = 0;

    // Iterate through each sensor to collect distance-based pose estimations
    for (DistanceSensors& sensor : Sensors) {
        double distance = sensor.CalculateMeasureDistance(OldPose.theta);  // Use last known heading
        DistancesPosition direction = sensor.GetDistancePosition();

        if (distance == E_MCLERR) continue;  // Skip invalid readings

        switch (direction) {
            case E_DISTANCE_LEFT:
                estimatedX += distance;
                break;
            case E_DISTANCE_RIGHT:
                estimatedX += FIELD_SIZE_MM - distance;  // Field width - distance
                break;
            case E_DISTANCE_FRONT:
                estimatedY += FIELD_SIZE_MM - distance;  // Field height - distance
                break;
            case E_DISTANCE_BACK:
                estimatedY += distance;
                break;
        }
        validReadings++;
    }

    // Ensure we don’t divide by zero
    if (validReadings > 0) {
        estimatedX /= validReadings;
        estimatedY /= validReadings;
    } else {
        return OldPose;  // Return previous pose if no valid readings
    }

    // Return estimated pose
    return {estimatedX, estimatedY, OldPose.theta};
}

int MCLController::ParticleGeraration(int numParticles) {
    Particles.clear();
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<double> xDist(0.0, FIELD_SIZE_MM);
    std::uniform_real_distribution<double> yDist(0.0, FIELD_SIZE_MM);

    for (int i = 0; i < numParticles; i++) {
        double x = xDist(gen);
        double y = yDist(gen);

        lemlib::Pose initialPose = {static_cast<float>(x), static_cast<float>(y), 0};
        Particle newParticle;
        newParticle.PoseMove(initialPose, {0.1, 0.1, 1.0}); // Adding small noise

        Particles.push_back(newParticle);
    }
    return E_MCLERR;
}

lemlib::Pose MCLController::CalculateChangedPose() {
    lemlib::Pose Current = chassis.getPose();
    ChangeInPose = Current - OldPose;
    OldPose = Current;
    return ChangeInPose;
}

int MCLController::ResampleParticles() {
    if (Particles.empty()) return E_MCLNOPARTICLES;

    // Create cumulative weight array
    std::vector<double> cumulativeWeights(Particles.size());
    cumulativeWeights[0] = Particles[0].GetWeight();
    for (size_t i = 1; i < Particles.size(); i++) {
        cumulativeWeights[i] = cumulativeWeights[i - 1] + Particles[i].GetWeight();
    }

    double maxWeight = cumulativeWeights.back();
    std::vector<Particle> newParticles;
    newParticles.reserve(Particles.size());

    for (size_t i = 0; i < Particles.size(); i++) {
        double randomWeight = GenerateRandomWeight(maxWeight); // Use separate function
        auto it = std::lower_bound(cumulativeWeights.begin(), cumulativeWeights.end(), randomWeight);
        size_t index = std::distance(cumulativeWeights.begin(), it);
        newParticles.push_back(Particles[index]); // Copy the selected particle
    }

    Particles = std::move(newParticles);
    return E_MCLSUCCESS;  // Success
}

int MCLController::UpdateRobotPose() {
    double sumX = 0, sumY = 0, sumTheta = 0;
    for (Particle& p : Particles) {
        sumX += p.GetPose().x;
        sumY += p.GetPose().y;
        sumTheta += p.GetPose().theta;
    }
    chassis.setPose({ static_cast<float>(sumX / Particles.size()), static_cast<float>(sumY / Particles.size()), chassis.getPose().theta });

    return E_MCLSUCCESS;
}

bool MCLController::IsValidSensorReading(DistanceSensors& sensor, lemlib::Pose particlePose) {
    double sensorReading = sensor.CalculateMeasureDistance(particlePose.theta);
    double expectedDistance = sensor.SimulateDistance(particlePose);

    // Add noise using the tracking error from the distance sensor
    double baseError = sensor.GetTrackingError();
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> errorDist(baseError, std::abs(baseError) * 0.1);
    double randomizedError = errorDist(gen);

    double error = sensorReading - expectedDistance;
    return std::abs(error) <= randomizedError;
}

lemlib::Pose MCLController::EstimatePoseUsingSensors(lemlib::Pose particlePose) {
    float estimatedX = 0.0, estimatedY = 0.0;
    int validReadings = 0;

    for (DistanceSensors& sensor : Sensors) {
        if (!IsValidSensorReading(sensor, particlePose)) continue;  // Skip invalid readings

        double distance = sensor.CalculateMeasureDistance(particlePose.theta);
        DistancesPosition direction = sensor.GetDistancePosition();

        switch (direction) {
            case E_DISTANCE_LEFT:
                estimatedX += distance;
                break;
            case E_DISTANCE_RIGHT:
                estimatedX += FIELD_SIZE_MM - distance;
                break;
            case E_DISTANCE_FRONT:
                estimatedY += FIELD_SIZE_MM - distance;
                break;
            case E_DISTANCE_BACK:
                estimatedY += distance;
                break;
        }
        validReadings++;
    }

    if (validReadings > 0) {
        estimatedX /= validReadings;
        estimatedY /= validReadings;
    } else {
        return particlePose;  // Return previous pose if no valid readings
    }

    return {estimatedX, estimatedY, particlePose.theta};
}

int MCLController::MCLLoop() {
    while (true) {
        lemlib::Pose deltaPose = CalculateChangedPose();

        for (Particle& p : Particles) {
            p.PoseMove(deltaPose, MovementNoise);
        }

        for (Particle& p : Particles) {
            lemlib::Pose estimatedPose = EstimatePoseUsingSensors(p.GetPose());
            double totalWeight = 1.0;

            for (DistanceSensors& sensor : Sensors) {
                double measuredDistance = sensor.CalculateMeasureDistance(estimatedPose.theta);
                double simulatedDistance = p.SimulateDistance(Sensors);

                double baseError = sensor.GetTrackingError();
                std::random_device rd;
                std::mt19937 gen(rd());
                std::normal_distribution<double> errorDist(baseError, std::abs(baseError) * 0.1);
                double randomizedError = errorDist(gen);

                double error = measuredDistance - simulatedDistance;
                totalWeight *= Gaussian(error, randomizedError);
            }

            p.UpdateWeight(totalWeight);
        }

        ResampleParticles();
        UpdateRobotPose();

        pros::delay(10);
    }
    return E_MCLLOOPEXIT;
}

} // namespace MCL