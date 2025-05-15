-- FABRIK algorithm in lua for a simple 2d arm

-- TODO: add end effector angle constraints
-- TODO: fix the joint constraints
-- refrence for different joints and their constraints: sec4.3 Applying joint limits to FABRIK https://www.researchgate.net/publication/220632147_FABRIK_A_fast_iterative_solver_for_the_Inverse_Kinematics_problem

require("utils")
require("Matrix3")
require("vector3")
require("bone")

function FABRIK(arm, target, maxIterations, tolerance)
    local iterations = 0
    local previousError = 0
    local error = math.huge
    local epsilon = 0.00001

    while (error > tolerance and iterations < maxIterations) do
        -- place the end effector at the target
        if target.rotation ~= nil then
            arm[#arm].position = target.rotation:Multiply(Vector3(0, 0, 1)):Multiply(-arm[#arm].length):add(target.position)
        else
            arm[#arm].position = target.position
        end

        -- forwards reaching
        for i = #arm -1, 2, -1 do
            local direction = arm[i].position:subtract(arm[i + 1].position):normalize()
            arm[i].position = direction:scale(arm[i].length):add(arm[i + 1].position)
        end


        -- backwards reaching
        for i = 1, #arm - 1 do
            local direction = arm[i + 1].position:subtract(arm[i].position):normalize()
            arm[i + 1].position = direction:scale(arm[i].length):add(arm[i].position)
        end

        -- apply joint constraints
        -- the orientation matrix for each joint is ralative to the parent bone
        local orientation = Matrix3:identity()
        for i = 1, #arm - 1 do
            local direction = arm[i + 1].position:subtract(arm[i].position)

            -- translate axis of rotaiton and direction to local orientation of arm i
            local axis = orientation:transpose():multiply(arm[i].axisOfRotation)
            local dir = orientation:transpose():multiply(direction)

            -- project direction to the plane of rotation
            local projectedDir = dir:projectToPlane(axis)

            -- scale to the length of the bone
            local length = arm[i].length
            local newDir = projectedDir:normalize():scale(length)

            -- apply the angle constraints
            local angle = SignedAngleAroundAxis(arm[i].initalDirection, newDir, axis)
            local angleClamp = Clamp(angle, arm[i].minAngle, arm[i].maxAngle)

            if angle ~= angleClamp then
                -- rotate the direction vector around the axis of rotation
                local rotation = Matrix3:rotationFromAxisAngle(axis, angleClamp - angle)
                newDir = rotation:multiply(newDir)
            end

            -- update the position of the next joint
            arm[i + 1].position = orientation:multiply(newDir):add(arm[i].position)

            -- update the orientation matrix
            orientation = Matrix3:rotationFromAxisAngle(axis, angleClamp):multiply(orientation)
            arm[i].orientation = orientation
        end

        error = arm[#arm].position:distance(target.position)
        print("Error: " .. error)

        iterations = iterations + 1
    end

    -- Convert all angles to degrees for output
    for i = 1, #arm - 1 do
        -- Convert to degrees in range -180 to 180
        local angleDegrees = (arm[i].currentAngle or 0) * (180 / math.pi)
        arm[i].angleDegrees = angleDegrees
        print("Joint " .. i .. " angle: " .. angleDegrees .. " degrees")
    end

    print("FABRIK iterations: " .. iterations)
    return arm
end