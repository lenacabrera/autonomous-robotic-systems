import numpy as np
import random

class ANN:

    # Neurocontroller TODO add bias?

    def __init__(self, sensor_distances, genotype, hidden_dim, max_sensor_reach):
        # seeding for random number generation
        np.random.seed(1)

        self.input_dim = len(sensor_distances) + hidden_dim  # add hidden nodes as memory (recurrent connection)
        self.hidden_dim = hidden_dim
        self.output_dim = 2  # left and right wheel

        # weights
        self.weights_in_hid, self.weights_hid_out = self.genotype_to_weights(genotype)

        # recurrent nodes
        # self.memory = [random.randint(1,max_sensor_reach) for i in range(hidden_dim)]
        self.memory = [max_sensor_reach for i in range(hidden_dim)]

        self.max_sensor_reach = max_sensor_reach

    def tanh(self, x):
        t = (np.exp(x) - np.exp(-x)) / (np.exp(x) + np.exp(-x))
        # dt = 1 - t ** 2
        # dt = 1 - 2 / (math.exp(2 * x) + 1)
        return t

    def sigmoid(self, x):
        return 1 / (1 + np.exp(-x))

    def decode_genotype(self, sensor_distances, genotype, v_max):
        # Return velocities, decoded by neural network
        weights_in_hid, weights_hid_out = self.genotype_to_weights(genotype)

        # input layer
        sensor_distances.extend(self.memory)
        inputs = np.asarray(sensor_distances)
        # TODO normalize?
        inputs = inputs / self.max_sensor_reach

        # hidden layer
        self.memory = self.tanh(np.dot(weights_in_hid, inputs.T))

        # output layer
        outputs = self.tanh(np.dot(weights_hid_out, self.memory.T))
        # outputs = self.sigmoid(np.dot(weights_hid_out, self.memory.T))
        velocities = self.convert_outputs_to_velocities(outputs, v_max)

        return velocities

    def convert_outputs_to_velocities(self, outputs, v_max):
        # TODO
        # print(outputs)
        # print(1/(outputs[0] * 100))

        # sigmoid
        # v_wheel_l = (outputs[0] - 0.5) * 1000
        # v_wheel_r = (outputs[1] - 0.5) * 1000

        # tanh
        # v_wheel_l = outputs[0] * 200
        # v_wheel_r = outputs[1] * 200

        # latest version:
        v_wheel_l = outputs[0] * 100 * v_max
        v_wheel_r = outputs[1] * 100 * v_max

        # v_wheel_l = outputs[0] * 10 * v_max
        # v_wheel_r = outputs[1] * 10 * v_max

        # v_wheel_l = outputs[0] * 1 * v_max
        # v_wheel_r = outputs[1] * 1 * v_max

        # print(v_wheel_l, v_wheel_r)
        return (v_wheel_l, v_wheel_r)

    def genotype_to_weights(self, genotype, bin_enc_len=7, prefix_divisor=10000):
        """ Decoding genotype (binary encoding) to weights (floats) """

        n_weights = int(len(genotype)/bin_enc_len)
        weights = np.zeros(n_weights)
        enc_start = 0

        for i_weight in range(n_weights):
            enc_end = enc_start + bin_enc_len
            bin_enc = genotype[enc_start:enc_end]
            enc_start = enc_end

            weight_suffix_int = 0
            for bit in bin_enc:
                weight_suffix_int = (weight_suffix_int << 1) | bit

            # build weight from prefix and suffix
            # prefix_divisor = 100
            weight = weight_suffix_int / prefix_divisor

            weights[i_weight] = weight

        # weights as vectors
        weights_in_hid_1D = weights[:self.input_dim * self.hidden_dim]
        weights_hid_out_1D = weights[self.input_dim * self.hidden_dim:]

        # weights reshaped to matrix
        weights_in_hid = np.reshape(weights_in_hid_1D, (self.hidden_dim, self.input_dim))
        weights_hid_out = np.reshape(weights_hid_out_1D, (self.output_dim, self.hidden_dim))

        return weights_in_hid, weights_hid_out
