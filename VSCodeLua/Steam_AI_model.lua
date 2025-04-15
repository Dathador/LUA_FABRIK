function Sigmoid(x)
    return 1/(1+math.exp(-x))
end

function DSigmoid(x)
    return math.exp(-x) / (1+math.exp(-x))^2
end


function ReLU(x)
	if x < 0 then
		return x/4
	else
		return x
	end
end

function DReLU(x)
    if x < 0 then
        return 0.25
    else
        return 1
    end
end


local function Narray(nl, nc, value)

	local value = value or 0
	local array = {}
	local nl = nl or 1
	local nc = nc or 1

	for i = 1, nl do
		local line = {}
		for j = 1, nc do
			if value == 'r' then
				table.insert(line, math.random())
			else
				table.insert(line, value)
			end
		end
		table.insert(array, line)
	end

	return array
end

-- two hidden layers
function NeuralNetwork:New(n_in, n_hidden1, n_hidden2, n_out)
    local NN = {}
    setmetatable(NN, self)
    self.__index = self

    NN.n_in = n_in
    NN.n_hidden1 = n_hidden1
    NN.n_hidden2 = n_hidden2
    NN.n_out = n_out

    NN.input = {{0,0},{0,1},{1,0},{1,1}}
    NN.output = {{0},{1},{1},{0}}
    NN.lr = 0.5
    NN.it = 1000000
    -- NN.bias = 0
	NN.error_training = 0.1
    NN.momentum = 0.001
    NN.fx = ReLU
    NN.dfx = DReLU
	NN.mode_training = 'sequential'

    -- arrays outputs
    NN.sum_hidden1 = Narray(1, NN.n_hidden1)
    NN.sum_hidden2 = Narray(1, NN.n_hidden2)
    NN.sum_out = Narray(1, NN.n_out)

    -- define arrays output activations
    NN.y_in = Narray(1, NN.n_in)
    NN.y_hidden1 = Narray(1, NN.n_hidden1)
    NN.y_hidden2 = Narray(1, NN.n_hidden2)
    NN.y_out = Narray(1, NN.n_out)

    -- create weights
    NN.w_in_hidden1 = Narray(NN.n_in, NN.n_hidden1, 'r')
    NN.w_hidden1_hidden2 = Narray(NN.n_hidden1, NN.n_hidden2, 'r')
    NN.w_hidden2_out = Narray(NN.n_hidden2, NN.n_out, 'r')

    -- -- create weights for bias
    -- NN.w_bias_in_hidden1 = Narray(1, NN.n_in, 'r')
    -- NN.w_bias_hidden1_hidden2 = Narray(1, NN.n_hidden1, 'r')
    -- NN.w_bias_hidden2_out = Narray(1, NN.n_hidden2, 'r')

    -- create momentum arrays
    NN.momentum_in_hidden1 = Narray(NN.n_in, NN.n_hidden1, 0)
    NN.momentum_hidden1_hidden2 = Narray(NN.n_hidden1, NN.n_hidden2, 0)
    NN.momentum_hidden2_out = Narray(NN.n_hidden2, NN.n_out, 0)



    function NN:training()
        local iteration = 1

        while iteration <= self.it do
            local error = 0

            -- loop through all input data
            for i = 1, #self.input do
                -- forward pass
                self:forward(self.input[i])

                -- calculate error
                error = error + self:backward(self.output[i])
            end
            error = error / #self.input
            if error < self.error_training then
                break
            end
            iteration = iteration + 1
        end
    end

    function NN:forward(x)
        for i = 1, self.n_in do
            self.y_in[1][i] = x[i]
        end

        for j = 1, self.n_hidden1 do
            self.sum_hidden1[1][j] = 0
            for i = 1, self.n_in do
                self.sum_hidden1[1][j] = self.sum_hidden1[1][j] + self.y_in[1][i] * self.w_in_hidden1[i][j]
            end
            self.y_hidden1[1][j] = self.ReLU(self.sum_hidden1[1][j])
        end

        for j = 1, self.n_hidden2 do
            self.sum_hidden2[1][j] = 0
            for i = 1, self.n_hidden1 do
                self.sum_hidden2[1][j] = self.sum_hidden2[1][j] + self.y_hidden1[1][i] * self.w_hidden1_hidden2[i][j]
            end
            self.y_hidden2[1][j] = self.ReLU(self.sum_hidden2[1][j])
        end

        for j = 1, self.n_out do
            self.sum_out[1][j] = 0
            for i = 1, self.n_hidden2 do
                self.sum_out[1][j] = self.sum_out[1][j] + self.y_hidden2[1][i] * self.w_hidden2_out[i][j]
            end
            self.y_out[1][j] = self.Sigmoid(self.sum_out[1][j])
        end

        return self.y_out[1]
    end

    function NN:backward(y)
        local y = y

		local output_deltas = Narray(1, self.n_out, 0)
		local nerror = 0.0
		-- value of return --
		local lms_error = 0.0 

        -- error lms algorithm (return of function)
        for i = 1, #y do
        	lms_error = lms_error + (0.5 * ((y[i] - self.y_out[i]) ^ 2))
        end

		-- outputs delts calcule		
		for i = 1, self.n_out do

			nerror = y[i] - self.y_out[i]
			output_deltas[i] = self.dfx(self.sum_out[i]) * nerror

		end

        local h2_delta = Narray(1, self.n_hidden2, 0)

        for i = 1, self.n_hidden2 do

            local nerror = 0.0

            for j = 1, self.n_out do 
                nerror = nerror + output_deltas[j] * self.w_h2_out[i][j]
            end

            h2_delta[i] = self.dfx(self.sum_h2[i]) * nerror
        end


        local h1_delta = Narray(1, self.n_hidden1, 0)

        for i = 1, self.n_hidden1 do

            local nerror = 0.0

            for j = 1, self.n_hidden2 do 
                nerror = nerror + h2_delta[j] * self.w_h1_h2[i][j]
            end

            h1_delta[i] = self.dfx(self.sum_h1[i]) * nerror
        end

        -------------- update WEIGTHS and MOMENTUM ---------------

        for i = 1, self.n_hidden2 do

            local mod = 0

            for j = 1, self.n_out do
                mod = output_deltas[j] * self.yhidden2[i]
                self.w_h2_out[i][j] = self.w_h2_out[i][j] + (self.lr * mod) +
                (self.momentum * self.mm_h2_out[i][j])
                self.mm_h2_out[i][j] = mod 
            end

        end

        for i = 1, self.n_hidden1 do

            local mod = 0

            for j = 1, self.n_hidden2 do
                mod = h2_delta[j] * self.yhidden1[i]
                self.w_h1_h2[i][j] = self.w_h1_h2[i][j] + (self.lr * mod) +
                (self.momentum * self.mm_h1_h2[i][j])
                self.mm_h1_h2[i][j] = mod 
            end

        end

        for i = 1, self.ninput do

            local mod = 0

            for j = 1, self.n_hidden1 do
                mod = h1_delta[j] * self.yinput[i]
                self.w_i_h1[i][j] = self.w_i_h1[i][j] + (self.lr * mod) +
                (self.momentum * self.mm_i_h1[i][j])
                self.mm_i_h1[i][j] = mod 
            end

        end

        return lms_error

    end
end
