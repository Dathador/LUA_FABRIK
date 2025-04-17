-- interface from my FABRIK to the game

-- TODO: add animation, keyframes will be saved as target points with rotations and inputs for the EasePosition function.

require("FABRIK")

Tick = 0

local startPosistion = {x = -4, y = 2}
local endPosition = {x = -2, y = 15}
local arm = {
    Joint:New( 0, 0, -180,  180, 9),
    Joint:New( 0, 0,    0,  180, 9),
    Joint:New( 0, 0,    0,    0, 0)
}

function Update(I)
    local target = EasePosition(startPosistion, endPosition, Tick, 0, 25, 1, 2)

    local max_iterations = 10
    local tolerance = 0.001

    arm = FABRIK(arm, target, tolerance, max_iterations)

    id1 = I:GetSubConstructIdentifier(0)
    I:SetSpinBlockRotationAngle(id1, arm[1].angle)

    id2 = I:GetSubConstructIdentifier(1)
    I:SetSpinBlockRotationAngle(id2, arm[2].angle)


    if Tick > 40 then
        Tick = 0
    end
    Tick = Tick + 1
end