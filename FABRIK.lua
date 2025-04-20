-- FABRIK algorithm in lua for a simple 2d arm

-- TODO: add end effector angle request
-- TODO: implement a 3d version of this algorithm
-- TODO: implement a 3d version of this algorithm with angle constraints
-- refrence for different joints and their constraints: sec4.3 Applying joint limits to FABRIK https://www.researchgate.net/publication/220632147_FABRIK_A_fast_iterative_solver_for_the_Inverse_Kinematics_problem
-- TODO: implement a 3d version of this algorithm with angle constraints and bias

require("utils")
require("bone")

function FABRIK(arm, target, maxIterations, tolerance)
    local iterations = 0
    local error = math.huge

    while (error > tolerance and iterations < maxIterations) do
        -- place the end effector at the target
        if target.rotation ~= nil then
            arm[#arm].position = Add(target.position, Multiply(MatrixVectorMult(target.rotation, {0, 0, 1}), -arm[#arm].length))
        else
            arm[#arm].position = target.position
        end

        -- forwards reaching
        for i = #arm -1, 2, -1 do
            local direction = UnitVector(Subtract(arm[i].position, arm[i + 1].position))
            arm[i].position = Add(arm[i + 1].position, Multiply(direction, arm[i].length))
        end

        local rotation = RotationMatrix(0, {1, 1, 1}) -- identity matrix
        -- backwards reaching
        for i = 1, #arm - 1 do
            rotation = MatrixMult(rotation, arm[i].rotation)
            local axis = UnitVector(MatrixVectorMult(rotation, arm[i].axisOfRotation))

            -- project the next bone onto the plane perpendicular to the axis of the current bone
            v = Subtract(arm[i + 1].position, arm[i].position)
            local projection = Multiply(axis, Dot(v, axis))
            local perpendicular = Subtract(v, projection)

            local direction = UnitVector(perpendicular)
            arm[i + 1].position = Add(arm[i].position, Multiply(direction, arm[i].length))

            arm[i].initialDirection = UnitVector(Subtract(arm[i + 1].position, arm[i].position))

            local angle = Angle(arm[i].initialDirection, direction)
            local clampedAngle = Clamp(angle, arm[i].minAngle, arm[i].maxAngle)
            if angle ~= clampedAngle then
                local residualAngle = angle - clampedAngle
                arm[i + 1].position = RotateAround(arm[i + 1].position, residualAngle, axis, arm[i].position)
            end

            -- update the rotation of the current bone
            arm[i].rotation = RotationMatrix(clampedAngle, axis)
            arm[i].axisOfRotation = axis
        end

        error = Distance(arm[#arm].position, target.position)
        print("Error: " .. error)
        iterations = iterations + 1
    end

    print("FABRIK iterations: " .. iterations)
    return arm
end