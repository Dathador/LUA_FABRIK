require("utils")

local keyframe = {}
keyframe.__index = keyframe

function keyframe:New(position, rotation, time, ease)
    local self = setmetatable({}, keyframe)
    self.position = position or {0, 0, 0}
    self.rotation = rotation or RotationMatrix(0, {0, 0, 1})
    self.time = time or 0
    self.ease = ease or {1, 1}
    return self
end


Animation = {}
Animation.__index = Animation

function Animation:New(keyframes, loop, reverse)
    local self = setmetatable({}, Animation)
    self.keyframes = keyframes or {}
    self.loop = loop or false
    self.reverse = reverse or false
    self.current_time = 0
    self.current_keyframe = 1
    return self
end