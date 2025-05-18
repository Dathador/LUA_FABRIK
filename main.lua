require "FABRIK"
require "bone"
require "Vector3"
require "Matrix3"
require "animations"
-- x pos right    neg left
-- y pos up       neg down
-- z pos forwards neg backwards

-- positive rotation clockwise, negative counter clockwise

local target = {
    position = Vector3:new(6, 5, 6), -- {2, 5, 7},
    rotation = nil
}

local arm = {
    bone:New(Vector3:new( 0, 0, 0), Matrix3:identity(), Vector3:new(0, 1, 0), -math.pi, math.pi, 4),
    bone:New(Vector3:new( 4, 0, 0), Matrix3:identity(), Vector3:new(0, 0, 1), -math.pi, math.pi, 4),
    bone:New(Vector3:new( 8, 0, 0), Matrix3:identity(), Vector3:new(0, 0, 1), -math.pi, math.pi, 4),
    bone:New(Vector3:new(12, 0, 0), Matrix3:identity(), Vector3:new(0, 0, 0), -math.pi, math.pi, 0)
    -- bone:New({16, 0, 0}, rotationFromAxisAngle(0, {0, 0, 1}), {0, 0, 0}, -math.pi, math.pi, 0),
}

function main()
    local max_iterations = 50
    local tolerance = 0.01

    -- set intial directions of each bone
    for i = 1, #arm - 1 do
        print(arm[i].position:toString())
        print(arm[i + 1].position:toString())
        arm[i].initalDirection = arm[i + 1].position:subtract(arm[i].position):normalize()
    end

    arm = FABRIK(arm, target, max_iterations, tolerance)

    for i = 1, #arm do
        print("Joint " .. i .. " " .. arm[i].position:toString())
        -- print("Axis of rotation: " .. arm[i].axisOfRotation:toString())
    end
end

main()