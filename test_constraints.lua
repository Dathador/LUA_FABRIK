require "FABRIK"
require "bone"

-- This script tests the joint constraints by creating various arm configurations
-- and testing them against a range of target positions

function print_arm_state(arm)
    print("\nArm configuration:")
    for i = 1, #arm do
        local pos = arm[i].position
        print(string.format("Joint %d: (%.2f, %.2f, %.2f)", i, pos[1], pos[2], pos[3]))

        if i < #arm then
            local angle = arm[i].angleDegrees or 0
            print(string.format("  Angle: %.2f degrees", angle))
            print(string.format("  Constraints: [%.2f, %.2f] degrees", 
                  arm[i].minAngle * (180/math.pi), 
                  arm[i].maxAngle * (180/math.pi)))
        end
    end
end

function test_constrained_arm()
    print("\n=== Testing Constrained Arm ===")

    -- Create an arm with limited joint constraints
    local arm = {
        -- Base joint with limited rotation around Y axis
        bone:New({0, 0, 0}, RotationMatrix(0, {0, 1, 0}), {0, 1, 0}, 
                 math.rad(-45), math.rad(45), 6),

        -- Second joint with limited rotation around X axis
        bone:New({6, 0, 0}, RotationMatrix(0, {1, 0, 0}), {1, 0, 0}, 
                 math.rad(-60), math.rad(60), 6),

        -- Third joint with limited rotation around Z axis
        bone:New({6, 0, 6}, RotationMatrix(0, {0, 0, 1}), {0, 0, 1}, 
                 math.rad(-90), math.rad(90), 6),

        -- End effector (no rotation constraints needed)
        bone:New({6, 6, 6}, RotationMatrix(0, {0, 0, 1}), {0, 0, 0}, 
                 -math.pi, math.pi, 0)
    }

    -- Test case 1: Target within reach but requiring constraint activation
    local target1 = { position = {-3, 8, 12}, rotation = nil }
    print("\nTest Case 1: Target at (-3, 8, 12)")
    local result1 = FABRIK(arm, target1, 100, 0.01)
    print_arm_state(result1)

    -- Reset arm
    arm = {
        bone:New({0, 0, 0}, RotationMatrix(0, {0, 1, 0}), {0, 1, 0}, 
                 math.rad(-45), math.rad(45), 6),
        bone:New({6, 0, 0}, RotationMatrix(0, {1, 0, 0}), {1, 0, 0}, 
                 math.rad(-60), math.rad(60), 6),
        bone:New({6, 0, 6}, RotationMatrix(0, {0, 0, 1}), {0, 0, 1}, 
                 math.rad(-90), math.rad(90), 6),
        bone:New({6, 6, 6}, RotationMatrix(0, {0, 0, 1}), {0, 0, 0}, 
                 -math.pi, math.pi, 0)
    }

    -- Test case 2: Target requiring maximum constraint use
    local target2 = { position = {10, 10, 10}, rotation = nil }
    print("\nTest Case 2: Target at (10, 10, 10)")
    local result2 = FABRIK(arm, target2, 100, 0.01)
    print_arm_state(result2)

    -- Reset arm
    arm = {
        bone:New({0, 0, 0}, RotationMatrix(0, {0, 1, 0}), {0, 1, 0}, 
                 math.rad(-45), math.rad(45), 6),
        bone:New({6, 0, 0}, RotationMatrix(0, {1, 0, 0}), {1, 0, 0}, 
                 math.rad(-60), math.rad(60), 6),
        bone:New({6, 0, 6}, RotationMatrix(0, {0, 0, 1}), {0, 0, 1}, 
                 math.rad(-90), math.rad(90), 6),
        bone:New({6, 6, 6}, RotationMatrix(0, {0, 0, 1}), {0, 0, 0}, 
                 -math.pi, math.pi, 0)
    }

    -- Test case 3: Target unreachable due to constraints
    local target3 = { position = {-10, 5, 0}, rotation = nil }
    print("\nTest Case 3: Target at (-10, 5, 0) - Unreachable due to constraints")
    local result3 = FABRIK(arm, target3, 100, 0.01)
    print_arm_state(result3)
end

-- Run the test
test_constrained_arm()