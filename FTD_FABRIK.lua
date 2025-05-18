-- interface from my FABRIK to the game

-- TODO: add animation, keyframes will be saved as target points with rotations and inputs for the EasePosition function.

require("FABRIK")

Tick = 0

local startPosistion = {x = -4, y = 2}
local endPosition = {x = -2, y = 15}


function buildArm(I)
    local jointNames = {
        shoulder1 = true,
        shoulder2 = true,
        shoulder3 = true,
        elbow1 = true,
        elbow2 = true
    }

    -- get a list of the named spinBlocks
    local spinBlocks = {}
    for i = 0, I:GetAllSubconstructsCount() do
        local id = I:GetSubConstructIdentifier(i)
        if I:IsSpinBlock(id) == true then
            local info = I:GetSubConstructInfo(id)
            local name = info.CustomName
            if jointNames[name] then
                table.insert(spinBlocks, {id = id, info = info})
            end
        end
    end

    

    -- construct the arm from the spinBlocks
    I:GetSubConstructInfo()
end


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