require "FABRIK"
require "animations"
-- x pos right    neg left
-- y pos up       neg down
-- z pos forwards neg backwards

-- positive rotation clockwise, negative counter clockwise

local target = {
    position = {0, 6, 6}, -- {2, 5, 7},
    rotation = nil
}

local arm = {
    bone:New({0, 0, 0}, RotationMatrix(0, {0, 0, 1}), {0, 1, 0}, -math.pi, math.pi, 6),
    bone:New({6, 0, 0}, RotationMatrix(0, {0, 0, 1}), {1, 0, 0}, -math.pi, math.pi, 6),
    bone:New({6, 0, 6}, RotationMatrix(0, {0, 0, 1}), {0, 0, 1}, -math.pi, math.pi, 6),
    bone:New({6, 6, 6}, RotationMatrix(0, {0, 0, 1}), {0, 0, 0}, -math.pi, math.pi, 0)
    -- bone:New({16, 0, 0}, RotationMatrix(0, {0, 0, 1}), {0, 0, 0}, -math.pi, math.pi, 0),
}

function main()

    local max_iterations = 10000
    local tolerance = 0.01

    arm = FABRIK(arm, target, max_iterations, tolerance)

    for i = 1, #arm do
        print("Joint " .. i .. ": (" .. arm[i].position[1] .. ", " .. arm[i].position[2] .. ", " .. arm[i].position[3] .. ")")
    end
end

main()