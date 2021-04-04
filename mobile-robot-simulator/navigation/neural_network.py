import numpy as np
import random
import math

class ANN:

    def __init__(self, conf, sensor_distances):

        self.input_dim = len(sensor_distances) + conf.dim_hidden  # add hidden nodes as memory (recurrent connection)
        self.dim_hidden = conf.dim_hidden
        self.output_dim = 2  # left and right wheel

        # recurrent nodes
        self.memory = [random.randint(1, conf.max_sensor_reach) for i in range(conf.dim_hidden)]
        # self.memory = [max_sensor_reach for i in range(dim_hidden)]

        self.max_sensor_reach = conf.max_sensor_reach

    def tanh(self, x):
        t = (np.exp(x) - np.exp(-x)) / (np.exp(x) + np.exp(-x))
        return t

    def sigmoid(self, x):
        return 1 / (1 + np.exp(-x))

    def decode_genotype(self, sensor_distances, genotype, v_max):
        # Return velocities, decoded by neural network
        weights_in_hid, weights_hid_out = self.genotype_to_weights(genotype)

        # input layer
        sensor_distances.extend(self.memory)
        inputs = np.asarray(sensor_distances)

        # normalize inputs
        inputs = inputs / self.max_sensor_reach

        # hidden layer
        self.memory = self.tanh(np.dot(weights_in_hid, inputs.T))

        # output layer
        outputs = self.tanh(np.dot(weights_hid_out, self.memory.T))
        # outputs = self.sigmoid(np.dot(weights_hid_out, self.memory.T))
        velocities = self.convert_outputs_to_velocities(outputs, v_max)

        return velocities

    def convert_outputs_to_velocities(self, outputs, v_max):

        v_wheel_l = abs((1 - outputs[0] * 1000) * 10)
        v_wheel_r = abs((1 - outputs[1] * 1000) * 10)
        # print(outputs[0] * 100 * v_max, outputs[0])

        # v_wheel_l = outputs[0] * 100 * v_max
        # v_wheel_r = outputs[1] * 100 * v_max

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
        weights_in_hid_1D = weights[:self.input_dim * self.dim_hidden]
        weights_hid_out_1D = weights[self.input_dim * self.dim_hidden:]

        # weights reshaped to matrix
        weights_in_hid = np.reshape(weights_in_hid_1D, (self.dim_hidden, self.input_dim))
        weights_hid_out = np.reshape(weights_hid_out_1D, (self.output_dim, self.dim_hidden))

        return weights_in_hid, weights_hid_out
