#include "../include/robot.hpp"
#include <array>
#include <cmath>
#include <algorithm>

// Helper: build rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)
// angles are in degrees
static std::array<std::array<float,3>,3> buildRotationMatrix(float yawDeg, float pitchDeg, float rollDeg) {
    const float y = yawDeg * M_PIf / 180.0f;
    const float p = pitchDeg * M_PIf / 180.0f;
    const float r = rollDeg * M_PIf / 180.0f;

    float cy = std::cos(y), sy = std::sin(y);
    float cp = std::cos(p), sp = std::sin(p);
    float cr = std::cos(r), sr = std::sin(r);

    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    std::array<std::array<float,3>,3> R{};
    R[0][0] = cy*cp;
    R[0][1] = cy*sp*sr - sy*cr;
    R[0][2] = cy*sp*cr + sy*sr;

    R[1][0] = sy*cp;
    R[1][1] = sy*sp*sr + cy*cr;
    R[1][2] = sy*sp*cr - cy*sr;

    R[2][0] = -sp;
    R[2][1] = cp*sr;
    R[2][2] = cp*cr;

    return R;
}

// Helper: multiply 3x3 matrix by 3x1 vector
static std::array<float,3> matMul(const std::array<std::array<float,3>,3>& M, const std::array<float,3>& v) {
    std::array<float,3> r{};
    for (int i=0;i<3;++i)
        r[i] = M[i][0]*v[0] + M[i][1]*v[1] + M[i][2]*v[2];
    return r;
}

// Helper: transpose a rotation matrix (inverse for orthonormal)
static std::array<std::array<float,3>,3> transposeMat(const std::array<std::array<float,3>,3>& M) {
    std::array<std::array<float,3>,3> T{};
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) T[i][j] = M[j][i];
    return T;
}

// Orient the chassis to absolute yaw/pitch/roll (degrees), keeping toe world positions fixed.
// - targetYawDeg, targetPitchDeg, targetRollDeg are absolute angles (deg).
// - steps controls interpolation smoothness. Use small steps for smoother motion.
// --- orientChassisTo corrigée ---
void Robot::orientChassisTo(float targetYawDeg, float targetPitchDeg, float targetRollDeg,
                            float targetX, float targetY, float targetZ,
                            int steps) {
    // detecter si caller a passé un targetZ valide
    const bool hasTargetZ = !std::isnan(targetZ);

    // Clamp raisonnable
    targetPitchDeg = std::clamp(targetPitchDeg, -30.0f, 30.0f);
    targetYawDeg   = std::clamp(targetYawDeg,   -45.0f, 45.0f);
    targetRollDeg  = std::clamp(targetRollDeg,  -30.0f, 30.0f);

    // états de départ
    const float startYaw   = chassisYawDeg;
    const float startPitch = chassisPitchDeg;
    const float startRoll  = chassisRollDeg;
    const float startX = chassisX;
    const float startY = chassisY;
    const float startZ = bodyHeight;

    // matrice de départ
    auto Rstart = buildRotationMatrix(startYaw, startPitch, startRoll);

    // positions de pieds dans le repère châssis au départ
    auto pStart = getLegsPositions();

    // positions world des pieds au départ : p_world = Rstart * pStart + Tstart
    std::array<std::array<float,3>,4> pWorld{};
    std::array<float,3> Tstart = { startX, startY, startZ };
    for (int i = 0; i < 4; ++i) {
        auto rotated = matMul(Rstart, pStart[i]);
        pWorld[i] = { rotated[0] + Tstart[0], rotated[1] + Tstart[1], rotated[2] + Tstart[2] };
    }

    // interpolation
    for (int s = 1; s <= steps; ++s) {
        float t = static_cast<float>(s) / steps;

        float curYaw   = startYaw   + (targetYawDeg   - startYaw)   * t;
        float curPitch = startPitch + (targetPitchDeg - startPitch) * t;
        float curRoll  = startRoll  + (targetRollDeg  - startRoll)  * t;
        float curX = startX + (targetX - startX) * t;
        float curY = startY + (targetY - startY) * t;
        float curZ = hasTargetZ ? (startZ + (targetZ - startZ) * t) : startZ;

        // --- CRUCIAL : mettre à jour l'état interne AVANT moveLegs ---
        // setPitch effectue le clamp et met pitch utilisé par computeZOffset
        chassisPitchDeg = curPitch;
        pitch = curPitch;   // si ta variable interne s'appelle "pitch"
        chassisYawDeg  = curYaw;          // pour cohérence si d'autres code lisent ça
        chassisRollDeg = curRoll;
        chassisX = curX;
        chassisY = curY;
        // note : on ne change pas bodyHeight ici, on le fera à la fin si hasTargetZ

        // calculer la rotation courante et son inverse
        auto Rcur = buildRotationMatrix(curYaw, curPitch, curRoll);
        auto RcurT = transposeMat(Rcur);

        // recalculer pour chaque jambe la position en repère châssis qui laissera
        // la toe au même endroit pWorld[i], pour la pose courante (Rcur, Tcur)
        std::vector<std::pair<LegID,std::array<float,3>>> targets;
        targets.reserve(4);

        for (int i = 0; i < 4; ++i) {
            std::array<float,3> world = pWorld[i];
            std::array<float,3> diff = { world[0] - curX, world[1] - curY, world[2] - curZ };
            auto newBodyPos = matMul(RcurT, diff);
            targets.push_back({ static_cast<LegID>(i), newBodyPos });
        }

        // Passer les targets aux jambes. transformTarget=false (targets déjà dans repère châssis)
        moveLegs({}, targets, false, 0.0f, 1);
        // moveLegs contient déjà un sleep (STANDARD_DELAY),
        // donc pas besoin d'un sleep supplémentaire ici
    }

    // Mise à jour finale : appliquer bodyHeight seulement si on avait targetZ
    if (hasTargetZ) setBodyHeight(targetZ);
    // mettre à jour angles stockés
    chassisYawDeg = targetYawDeg;
    chassisPitchDeg = targetPitchDeg;
    chassisRollDeg = targetRollDeg;
    chassisX = targetX;
    chassisY = targetY;
}

// --- lookAround corrigée (mapping joystick -> orientChassisTo) ---
void Robot::lookAround(float jx, float jy, float maxYawDeg, float maxPitchDeg, int steps) {
    jx = std::clamp(jx, -1.0f, 1.0f);
    jy = std::clamp(jy, -1.0f, 1.0f);
float mag = std::sqrt(jx * jx + jy * jy);
jx /= mag; jy /= mag;
    float targetYaw   = jx * maxYawDeg;
    float targetPitch = jy * maxPitchDeg;
    float targetRoll  = 0.0f;

    // IMPORTANT: on ne passe PAS de targetZ (NaN) pour signifier "ne pas changer la translation Z"
    orientChassisTo(targetYaw, targetPitch, targetRoll,
                    chassisX, chassisY, std::numeric_limits<float>::quiet_NaN(),
                    steps);
}
