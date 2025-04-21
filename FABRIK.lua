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

    -- save inital positions of the arm
    local initialPositions = {}
    for i = 1, #arm do
        initialPositions[i] = {x = arm[i].position[1], y = arm[i].position[2], z = arm[i].position[3]}
    end

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

        -- backwards reaching
        for i = 1, #arm - 1 do
            local direction = UnitVector(Subtract(arm[i + 1].position, arm[i].position))
            arm[i + 1].position = Add(arm[i].position, Multiply(direction, arm[i].length))
        end

        -- Enforce joint constraints
        for i = 1, #arm - 1 do
            local a = MatrixVectorMult(arm[i].rotation, arm[i].axisOfRotation)  -- Axis of rotation for joint i
            local v = Subtract(arm[i + 1].position, arm[i].position)  -- Current bone direction
            local dot = Dot(v, a)
            local a_magnitude_sq = Dot(a, a)  -- Should be 1 if a is normalized, but compute for generality

            -- Project v onto the plane perpendicular to a
            local proj_v = Subtract(v, Multiply(a, dot / a_magnitude_sq))
            local proj_magnitude = Length(proj_v)

            if proj_magnitude > 0 then
                -- Adjust position to lie on the plane and maintain length
                local direction = UnitVector(proj_v)
                arm[i + 1].position = Add(arm[i].position, Multiply(direction, arm[i].length))
            else
                -- Rare case: v is parallel to a, projection is zero
                -- For now, leave position unchanged or set a default (e.g., along x-axis in plane)
                -- Here, we’ll skip adjustment to avoid division by zero
                print("Warning: Joint " .. i + 1 .. " constraint projection failed")
            end
        end

        error = Distance(arm[#arm].position, target.position)
        print("Error: " .. error)
        iterations = iterations + 1
    end

    print("FABRIK iterations: " .. iterations)
    return arm
end