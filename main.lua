require "FABRIK"
require "animations"
-- x pos right    neg left
-- y pos up       neg down
-- z pos forwards neg backwards

-- positive rotation clockwise, negative counter clockwise

local target = {
    position = {2, 5, 7},
    rotation = nil
}

local arm = {
    bone:New({ 0, 0, 0}, RotationMatrix(0, {0, 0, 1}), {0, 1, 0}, -math.pi, math.pi, 6),
    bone:New({ 4, 0, 0}, RotationMatrix(0, {0, 0, 1}), {0, 0, 1}, -math.pi, math.pi, 6),
    bone:New({ 8, 0, 0}, RotationMatrix(0, {0, 0, 1}), {0, 1, 0}, -math.pi, math.pi, 0)
}

function main()
    local max_iterations = 20
    local tolerance = 0.01

    arm = FABRIK(arm, target, max_iterations, tolerance)

    for i = 1, #arm do
        print("Joint " .. i .. ": (" .. arm[i].position[1] .. ", " .. arm[i].position[2] .. ", " .. arm[i].position[3] .. ")")
    end
end

main()