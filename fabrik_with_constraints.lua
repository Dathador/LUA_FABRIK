-- Vector operations
function vec3(x, y, z)
    return {x = x, y = y, z = z}
end

function vec3_add(v1, v2)
    return vec3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z)
end

function vec3_sub(v1, v2)
    return vec3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z)
end

function vec3_mul_scalar(v, s)
    return vec3(v.x * s, v.y * s, v.z * s)
end

function vec3_dot(v1, v2)
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z
end

function vec3_cross(v1, v2)
    return vec3(
        v1.y * v2.z - v1.z * v2.y,
        v1.z * v2.x - v1.x * v2.z,
        v1.x * v2.y - v1.y * v2.x
    )
end

function vec3_norm(v)
    return math.sqrt(vec3_dot(v, v))
end

function vec3_normalize(v)
    local n = vec3_norm(v)
    if n > 0 then
        return vec3_mul_scalar(v, 1 / n)
    else
        return vec3(0, 0, 0)
    end
end

-- Matrix operations
function mat3()
    return {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    }
end

function mat3_mul(m1, m2)
    local result = mat3()
    for i = 1, 3 do
        for j = 1, 3 do
            result[i][j] = m1[i][1] * m2[1][j] + m1[i][2] * m2[2][j] + m1[i][3] * m2[3][j]
        end
    end
    return result
end

function mat3_mul_vec(m, v)
    return vec3(
        m[1][1] * v.x + m[1][2] * v.y + m[1][3] * v.z,
        m[2][1] * v.x + m[2][2] * v.y + m[2][3] * v.z,
        m[3][1] * v.x + m[3][2] * v.y + m[3][3] * v.z
    )
end

function rotation_matrix(axis, angle)
    local c = math.cos(angle)
    local s = math.sin(angle)
    local t = 1 - c
    local x, y, z = axis.x, axis.y, axis.z
    local n = vec3_norm(axis)
    if n > 0 then
        x, y, z = x / n, y / n, z / n
    end
    return {
        {t*x*x + c, t*x*y - z*s, t*x*z + y*s},
        {t*x*y + z*s, t*y*y + c, t*y*z - x*s},
        {t*x*z - y*s, t*y*z + x*s, t*z*z + c}
    }
end

function rotation_matrix_from_direction(dir, up)
    local z = vec3_normalize(dir)
    local x = vec3_normalize(vec3_cross(up, z))
    local y = vec3_cross(z, x)
    return {
        {x.x, y.x, z.x},
        {x.y, y.y, z.y},
        {x.z, y.z, z.z}
    }
end

function rotation_to_align_with_constraint(from_dir, to_dir, axis)
    local from_proj = vec3_sub(from_dir, vec3_mul_scalar(axis, vec3_dot(from_dir, axis)))
    local to_proj = vec3_sub(to_dir, vec3_mul_scalar(axis, vec3_dot(to_dir, axis)))
    if vec3_norm(from_proj) > 0 and vec3_norm(to_proj) > 0 then
        from_proj = vec3_normalize(from_proj)
        to_proj = vec3_normalize(to_proj)
        local dot_prod = vec3_dot(from_proj, to_proj)
        local cross_prod = vec3_cross(from_proj, to_proj)
        local angle = math.atan(vec3_dot(cross_prod, axis), dot_prod)
        return rotation_matrix(axis, angle)
    else
        return mat3()  -- identity matrix
    end
end

-- Utility function
function compute_lengths(positions)
    local lengths = {}
    for i = 1, #positions - 1 do
        local d = vec3_sub(positions[i+1], positions[i])
        lengths[i] = vec3_norm(d)
    end
    return lengths
end

-- Main FABRIK function with hinge joint constraints and plane projection
function FABRIK_with_constraints(positions, target, joint_constraints, max_iterations, tolerance)
    local n = #positions - 1
    local p = {}
    for i = 1, #positions do
        p[i] = vec3(positions[i].x, positions[i].y, positions[i].z)
    end
    local lengths = compute_lengths(p)
    
    -- Initialize orientations
    local orientations = {}
    for i = 1, n do
        local dir = vec3_normalize(vec3_sub(p[i+1], p[i]))
        orientations[i] = rotation_matrix_from_direction(dir, vec3(0, 1, 0))
    end
    
    local iteration = 0
    while iteration < max_iterations do
        -- Backward pass
        p[n+1] = target
        for i = n, 1, -1 do
            local direction = vec3_normalize(vec3_sub(p[i], p[i+1]))
            p[i] = vec3_add(p[i+1], vec3_mul_scalar(direction, lengths[i]))
        end
        
        -- Forward pass
        p[1] = positions[1]  -- root is fixed
        for i = 1, n do
            local direction = vec3_normalize(vec3_sub(p[i+1], p[i]))
            p[i+1] = vec3_add(p[i], vec3_mul_scalar(direction, lengths[i]))
        end
        
        -- Project each joint onto the plane of rotation of the previous joint
        for i = 1, n-1 do
            local jc = joint_constraints[i]
            local a_i = jc.axis
            local axis = mat3_mul_vec(orientations[i], a_i)
            axis = vec3_normalize(axis)
            local p_i = p[i+1]  -- joint i
            local p_ip1 = p[i+2]  -- joint i+1
            local v = vec3_sub(p_ip1, p_i)
            local dot_v_axis = vec3_dot(v, axis)
            local p_proj = vec3_sub(p_ip1, vec3_mul_scalar(axis, dot_v_axis))
            local dir = vec3_sub(p_proj, p_i)
            local dir_norm = vec3_norm(dir)
            if dir_norm > 1e-6 then
                local scale = lengths[i+1] / dir_norm
                p[i+2] = vec3_add(p_i, vec3_mul_scalar(dir, scale))
            end
        end
        
        -- Enforce hinge joint angle constraints
        for i = 2, n do
            local jc = joint_constraints[i-1]
            local a_i = jc.axis
            local min_angle = jc.min_angle
            local max_angle = jc.max_angle
            
            -- Compute global hinge axis
            local axis = mat3_mul_vec(orientations[i-1], a_i)
            axis = vec3_normalize(axis)
            
            -- Compute vectors for parent and child bones
            local u = vec3_sub(p[i], p[i-1])
            local v = vec3_sub(p[i+1], p[i])
            
            -- Project onto plane perpendicular to hinge axis
            local u_proj = vec3_sub(u, vec3_mul_scalar(axis, vec3_dot(u, axis)))
            local v_proj = vec3_sub(v, vec3_mul_scalar(axis, vec3_dot(v, axis)))
            
            if vec3_norm(u_proj) > 0 and vec3_norm(v_proj) > 0 then
                u_proj = vec3_normalize(u_proj)
                v_proj = vec3_normalize(v_proj)
                
                -- Compute current angle
                local dot_prod = vec3_dot(u_proj, v_proj)
                local cross_prod = vec3_cross(u_proj, v_proj)
                local theta_i = math.atan(vec3_dot(cross_prod, axis), dot_prod)
                
                -- Clamp angle
                local theta_clamped = math.max(min_angle, math.min(max_angle, theta_i))
                
                if theta_clamped ~= theta_i then
                    -- Compute adjustment rotation
                    local adjustment_angle = theta_clamped - theta_i
                    local R_adjust = rotation_matrix(axis, adjustment_angle)
                    
                    -- Adjust child bone vector and position
                    local v_new = mat3_mul_vec(R_adjust, v)
                    p[i+1] = vec3_add(p[i], v_new)
                end
            end
        end
        
        -- Update orientations
        orientations[1] = rotation_matrix_from_direction(vec3_normalize(vec3_sub(p[2], p[1])), vec3(0, 1, 0))
        for i = 2, n do
            local dir = vec3_normalize(vec3_sub(p[i+1], p[i]))
            local prev_dir = vec3_normalize(vec3_sub(p[i], p[i-1]))
            local axis = mat3_mul_vec(orientations[i-1], joint_constraints[i-1].axis)
            local R_rel = rotation_to_align_with_constraint(prev_dir, dir, axis)
            orientations[i] = mat3_mul(orientations[i-1], R_rel)
        end
        
        -- Check convergence
        local dist = vec3_norm(vec3_sub(p[n+1], target))
        if dist < tolerance then
            break
        end
        
        iteration = iteration + 1
    end
    
    return p
end

-- Example usage
function main()
    local target = vec3(0, 6, 6)
    local positions = {
        vec3(0, 0, 0),
        vec3(6, 0, 0),
        vec3(12, 0, 0),
        vec3(12, 0, 0)
    }
    
    local joint_constraints = {
        {axis = vec3(0, 1, 0), min_angle = -math.pi, max_angle = math.pi},
        {axis = vec3(0, 1, 0), min_angle = -math.pi, max_angle = math.pi},
        {axis = vec3(1, 0, 0), min_angle = -math.pi, max_angle = math.pi}
    }
    
    local max_iterations = 20
    local tolerance = 0.01
    
    local result_positions = FABRIK_with_constraints(positions, target, joint_constraints, max_iterations, tolerance)
    
    for i = 1, #result_positions do
        print("Joint " .. i .. ": (" .. result_positions[i].x .. ", " .. result_positions[i].y .. ", " .. result_positions[i].z .. ")")
    end
end

main()