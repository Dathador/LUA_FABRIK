require "FABRIK"
require "animations"
-- x pos right    neg left
-- y pos up       neg down
-- z pos forwards neg backwards

-- positive rotation clockwise, negative counter clockwise

local target = {
    position = {2, 8, 5},
    rotation = nil
}

local arm = {
    bone:New({ 0, 0, 0}, RotationMatrix(0, {0, 0, 1}), {0, 1, 0}, -math.pi, math.pi, 4),
    bone:New({ 4, 0, 0}, RotationMatrix(0, {0, 0, 1}), {0, 0, 1}, -math.pi, math.pi, 4),
    bone:New({ 8, 0, 0}, RotationMatrix(0, {0, 0, 1}), {0, 1, 0}, -math.pi, math.pi, 0)
}

function main()
    local max_iterations = 20
    local tolerance = 0.01

    arm = FABRIK(arm, target, max_iterations, tolerance)

    for i = 1, #arm do
        print("Joint " .. i .. ": (" .. arm[i].position[1] .. ", " .. arm[i].position[2] .. ", " .. arm[i].position[3] .. ")")
    end
    print("Axis of rotation " .. "(" .. arm[2].axisOfRotation[1] .. ", " .. arm[2].axisOfRotation[2] .. ", " .. arm[2].axisOfRotation[3] .. ")")

    print()
end

main()