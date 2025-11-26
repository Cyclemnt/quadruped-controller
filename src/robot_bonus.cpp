#include "include/robot.hpp"
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
void Robot::orientChassisTo(float targetYawDeg, float targetPitchDeg, float targetRollDeg,
                            float targetX, float targetY, float targetZ,
                            int steps) {
    // If targetZ not provided, use current bodyHeight as Z target.
    if (std::isnan(targetZ)) targetZ = bodyHeight;

    // Clamp reasonable ranges (optionnel)
    targetPitchDeg = std::clamp(targetPitchDeg, -30.0f, 30.0f);
    targetYawDeg   = std::clamp(targetYawDeg,   -45.0f, 45.0f);
    targetRollDeg  = std::clamp(targetRollDeg,  -30.0f, 30.0f);

    // Save start states
    const float startYaw   = chassisYawDeg;
    const float startPitch = chassisPitchDeg;
    const float startRoll  = chassisRollDeg;
    const float startX = chassisX;
    const float startY = chassisY;
    const float startZ = bodyHeight;

    // Precompute start rotation matrix
    auto Rstart = buildRotationMatrix(startYaw, startPitch, startRoll);

    // read the positions of feet in *chassis frame at start*
    auto pStart = getLegsPositions(); // array[4] of {x,y,z} in chassis frame
    // compute world positions of feet at start: p_world = Rstart * pStart + Tstart
    std::array<std::array<float,3>,4> pWorld{};
    std::array<float,3> Tstart = { startX, startY, startZ };
    for (int i=0;i<4;i++) {
        auto rotated = matMul(Rstart, pStart[i]);
        pWorld[i] = { rotated[0] + Tstart[0], rotated[1] + Tstart[1], rotated[2] + Tstart[2] };
    }

    // Interpolate
    for (int s = 1; s <= steps; ++s) {
        float t = static_cast<float>(s) / steps;
        float curYaw   = startYaw   + (targetYawDeg   - startYaw)   * t;
        float curPitch = startPitch + (targetPitchDeg - startPitch) * t;
        float curRoll  = startRoll  + (targetRollDeg  - startRoll)  * t;
        float curX = startX + (targetX - startX) * t;
        float curY = startY + (targetY - startY) * t;
        float curZ = startZ + (targetZ - startZ) * t;

        auto Rcur = buildRotationMatrix(curYaw, curPitch, curRoll);
        auto RcurT = transposeMat(Rcur); // inverse rotation

        std::array<std::pair<LegID,std::array<float,3>>, 4> targetsArr{};
        for (int i=0;i<4;i++) {
            std::array<float,3> world = pWorld[i];
            // p_body_new = Rcur^T * (p_world - Tcur)
            std::array<float,3> diff = { world[0] - curX, world[1] - curY, world[2] - curZ };
            auto newBodyPos = matMul(RcurT, diff);

            targetsArr[i] = { static_cast<LegID>(i), newBodyPos };
        }

        // convert to vector<vectorpairs> for moveLegs
        std::vector<std::pair<LegID,std::array<float,3>>> targets;
        targets.reserve(4);
        for (auto &tp : targetsArr) targets.push_back(tp);

        // IMPORTANT: these targets are in chassis frame that we are commanding -> transformTarget=false
        moveLegs({}, targets, false, 0.0f, 1); // 1 step inside moveLegs (we already interpolate), no arc
        // small delay already handled in moveLegs via STANDARD_DELAY
    }

    // Update stored chassis pose to target
    chassisYawDeg = targetYawDeg;
    chassisPitchDeg = targetPitchDeg;
    chassisRollDeg = targetRollDeg;
    chassisX = targetX;
    chassisY = targetY;
    setBodyHeight(targetZ); // keep bodyHeight in sync
}

// Convenience: joystick mapper (jx,jy in [-1,1]) to look around.
// jx -> yaw (left/right), jy -> pitch (up/down). roll fixed at 0 for simplicity.
void Robot::lookAround(float jx, float jy, float maxYawDeg, float maxPitchDeg, int steps) {
    // clamp inputs
    jx = std::clamp(jx, -1.0f, 1.0f);
    jy = std::clamp(jy, -1.0f, 1.0f);

    float targetYaw = jx * maxYawDeg;
    float targetPitch = jy * maxPitchDeg;
    float targetRoll = 0.0f; // keep roll neutral; you can map another axis if needed

    // call orientChassisTo (this will interpolate smoothly)
    orientChassisTo(targetYaw, targetPitch, targetRoll, 0.0f, 0.0f, std::numeric_limits<float>::quiet_NaN(), steps);
}
