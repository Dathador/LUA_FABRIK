
-- implimentation of the FABRIK algorithm in lua for a simple 2d arm

-- to expand this to 3d we would need to add a z value to each joint as well as an axis of rotation
-- for more realistic 3d arm movement, add a bias to the angle of rotation for each joint

-- TODO: add end effector angle request
-- TODO: implement a 3d version of this algorithm
-- TODO: implement a 3d version of this algorithm with angle constraints
-- refrence for different joints and their constraints: sec4.3 Applying joint limits to FABRIK https://www.researchgate.net/publication/220632147_FABRIK_A_fast_iterative_solver_for_the_Inverse_Kinematics_problem
-- TODO: implement a 3d version of this algorithm with angle constraints and bias

require("utils")

Joint = {}

function Joint:New(x, y, z, min_angle, max_angle, length, hinge_axis, reference_direction)
    local self = setmetatable({}, Joint)
    self.position = {x, y, z}
    self.min_angle = min_angle/180 * math.pi
    self.max_angle = max_angle/180 * math.pi
    self.length = length
    self.hinge_axis = hinge_axis
    self.angle = 0
    self.reference_direction = reference_direction
    return self
end


function FABRIK(arm, target, threshold, max_iterations)
    -- forward and backwards reaching inverse kinematics algorithm
    -- arm is a table of joints, each joint has a position, min_angle, max_angle and length
    -- target is a position and rotation

    local iterations = 0
    local current_error = math.huge

    while (current_error > threshold and iterations < max_iterations) do

        -- forward reaching
        arm[#arm].position = target.position

        for i = #arm - 1, 2, -1 do
            local v = Subtract(arm[i].position, arm[i + 1].position)
            local direction = Normalize(v)
            arm[i].position = Add(arm[i + 1].position, Multiply(direction, arm[i].length))
        end

        -- Backward Reaching + Constraints
        for i = 2, #arm do

            local v = Subtract(arm[i].position, arm[i - 1].position)
            local direction = Normalize(v)
            arm[i].position = Add(arm[i - 1].position, Multiply(direction, arm[i - 1].length))

            -- Enforce rotation constraints
            if i < #arm and arm[i].hinge_axis then
                local h = Normalize(arm[i].hinge_axis)  -- Hinge axis
                local x = Normalize(Subtract(arm[i].position, arm[i - 1].position))  -- Previous segment
                local z = h
                local y = Normalize(Cross(z, x))  -- Local y-axis
                local d = Normalize(Subtract(arm[i + 1].position, arm[i].position))  -- Next segment
                local d_proj = Subtract(d, Multiply(z, Dot(d, z)))  -- Project onto plane perpendicular to hinge
                local len_proj = Length(d_proj)
                if len_proj > 0 then
                    d_proj = Multiply(d_proj, 1 / len_proj)
                    local theta = math.atan(Dot(d_proj, y), Dot(d_proj, x))
                    local clamped_theta = Clamp(theta, arm[i].min_angle, arm[i].max_angle)
                    arm[i].angle = clamped_theta
                    if clamped_theta ~= theta then
                        local delta_theta = clamped_theta - theta
                            arm[i + 1].position = RotatePointAroundAxis(arm[i + 1].position, delta_theta, z, arm[i].position)
                    end
                end
            end
        end

        -- Enforce base joint constraint
        if arm[1].hinge_axis and arm[1].reference_direction then
            local h = Normalize(arm[1].hinge_axis)
            local ref = Normalize(arm[1].reference_direction)
            ref = Normalize(Subtract(ref, Multiply(h, Dot(ref, h))))  -- Ensure ref is perpendicular to h
            local d = Normalize(Subtract(arm[2].position, arm[1].position))
            local d_proj = Subtract(d, Multiply(h, Dot(d, h)))
            local len_proj = Length(d_proj)
            if len_proj > 0 then
                d_proj = Multiply(d_proj, 1 / len_proj)
                local cross_prod = Cross(ref, d_proj)
                local theta = math.atan(Dot(cross_prod, h), Dot(ref, d_proj))
                local clamped_theta = Clamp(theta, arm[1].min_angle, arm[1].max_angle)
                if clamped_theta ~= theta then
                    local delta_theta = clamped_theta - theta
                    -- Rotate entire chain from arm[2] onwards
                    for j = 2, #arm do
                        arm[j].position = RotatePointAroundAxis(arm[j].position, delta_theta, h, arm[1].position)
                    end
                end
            end
        end


        iterations = iterations + 1
        current_error = Distance(arm[#arm].position, target.position)
    end

    print("iterations " .. iterations)
    return arm
end


local target = {
    position = {-18, 3, 5},
    angle = 90/180 * math.pi
}


function main()
    local max_iterations = 20
    local tolerance = 0.01
    -- joint (x, y, min_angle, max_angle, length, hinge_axis)

    local arm = {
        Joint:New(   0,  0, 0,  -90,    0,   9, {0, 0, 1}, {1, 0, 0}),
        Joint:New(  -9, -9, 0, -180,  180,   9, {0, 0, 1},       nil),
        Joint:New( -10,  2, 0,  -60,   60,   1, {0, 0, 1},       nil),
        Joint:New( -18,  4, 0,    0,    0,   0,       nil,       nil)
    }


    arm = FABRIK(arm, target, tolerance, max_iterations)

    for i = 1, #arm do
        print("Joint " .. i .. ": (" .. arm[i].position[1] .. ", " .. arm[i].position[2] .. ", " .. arm[i].position[3] .. ")")
    end
end

main()