require "Vector3"
require "Matrix3"
require "utils"


bone = {}
bone.__index = bone

function bone:New(position, orientation, axisOfRotation, minAngle, maxAngle, length)
    local self = setmetatable({}, bone)
    self.position = position or Vector3:new(0, 0, 0)
    self.orientation = orientation or RotationMatrix(0, {0, 0, 1})
    self.axisOfRotation = axisOfRotation or {0, 0, 0}
    self.initalDirection = {1, 0, 0}
    self.minAngle = minAngle or -math.pi
    self.maxAngle = maxAngle or  math.pi
    self.length = length or 1
    return self
end