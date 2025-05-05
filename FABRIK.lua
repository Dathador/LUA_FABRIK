-- FABRIK algorithm in lua for a simple 2d arm

-- TODO: add end effector angle constraints
-- TODO: fix the joint constraints
-- refrence for different joints and their constraints: sec4.3 Applying joint limits to FABRIK https://www.researchgate.net/publication/220632147_FABRIK_A_fast_iterative_solver_for_the_Inverse_Kinematics_problem

require("utils")
require("bone")

function FABRIK(arm, target, maxIterations, tolerance)
    local iterations = 0
    local previousError = 0
    local error = math.huge
    local epsilon = 0.00001

    -- set intial directions of each bone
    for i = 1, #arm - 1 do
        arm[i].initalDirection = UnitVector(Subtract(arm[i + 1].position, arm[i].position))
    end

    while (error > tolerance and iterations < maxIterations) do
        -- place the end effector at the target
        if target.rotation ~= nil then
            arm[#arm].position = Add(target.position, Multiply(MatrixVectorMult(target.rotation, {0, 0, 1}), -arm[#arm].length))
        else
            arm[#arm].position = target.position
        end

        -- forwards reaching
        -- print("\n Forwards step")
        for i = #arm -1, 2, -1 do
            local direction = UnitVector(Subtract(arm[i].position, arm[i + 1].position))
            arm[i].position = Add(arm[i + 1].position, Multiply(direction, arm[i].length))
        end

        -- print bone positions
        -- for i = 1, #arm do
        --     print("Bone " .. i .. " (" .. arm[i].position[1] .. ", " .. arm[i].position[2] .. ", " .. arm[i].position[3] .. ")")
        -- end


        -- backwards reaching
        -- print("\n Backwards step")
        local rotation = IdentityMatrix()
        for i = 1, #arm - 1 do
            -- local direction = UnitVector(Subtract(arm[i + 1].position, arm[i].position))
            -- arm[i + 1].position = Add(arm[i].position, Multiply(direction, arm[i].length))

            local v1 = Subtract(arm[i + 1].position, arm[i].position)
            -- print("Rotating axis " .. arm[i].axisOfRotation[1] .. ", " .. arm[i].axisOfRotation[2] .. ", " .. arm[i].axisOfRotation[3])
            local normal = MatrixVectorMult(rotation, arm[i].axisOfRotation)
            -- print("Projecting arm " .. i + 1 .. " onto plane with normal " .. normal[1] .. ", " .. normal[2] .. ", " .. normal[3])
            local v2 = UnitVector(ProjectToPlane(v1, normal))
            arm[i + 1].position = Add(arm[i].position, Multiply(v2, arm[i].length))

            local angle = SignedAngleAroundAxis(v2, MatrixVectorMult(rotation, arm[i].initalDirection), arm[i].axisOfRotation)
            local constrainedAngle = Clamp(angle, arm[i].minAngle, arm[i].maxAngle)
            
            -- If the angle was constrained, we need to recalculate v2
            if constrainedAngle ~= angle then
                -- Rotate the initial direction by the constrained angle to get the constrained direction
                local constrainedDirection = RotateVector(rotatedInitialDirection, constrainedAngle, normal)
                v2 = UnitVector(constrainedDirection)
                
                -- Update the position of the next joint based on the constrained angle
                arm[i + 1].position = Add(arm[i].position, Multiply(v2, arm[i].length))
            end
            -- print("Angle " .. i .. ": " .. angle .. " radians")
            -- print("between " .. arm[i].axisOfRotation[1] .. ", " .. arm[i].axisOfRotation[2] .. ", " .. arm[i].axisOfRotation[3] .. " and " .. v2[1] .. ", " .. v2[2] .. ", " .. v2[3] .. " on axis " .. normal[1] .. ", " .. normal[2] .. ", " .. normal[3])

            arm[i].rotation = RotationMatrix(-clampedAngle, arm[i].axisOfRotation)


            rotation = MatrixMult(rotation, arm[i].rotation)
        end

        -- print bone positions
        -- for i = 1, #arm do
        --     print("Bone " .. i .. " (" .. arm[i].position[1] .. ", " .. arm[i].position[2] .. ", " .. arm[i].position[3] .. ")")
        -- end

        previousError = error
        error = Distance(arm[#arm].position, target.position)
        print("Error: " .. error)
        if math.abs(error - previousError) < epsilon then
            print("FABRIK converged to a local minimum")
            break
        end
        -- print()
        iterations = iterations + 1
    end

    print("FABRIK iterations: " .. iterations)
    return arm
end