import numpy as np
import tensorflow as tf


# =============================================================================
#                                 DEFINE NETWORK
# =============================================================================
states = tf.placeholder(tf.float32,[None,6])
actions = tf.placeholder(tf.float32,[None,6])

layer_0 = tf.layers.dense(inputs=states,units = 128,activation=tf.nn.relu)
layer_1 = tf.layers.dense(inputs=states,units = 128,activation=tf.nn.relu)
layer_2 = tf.layers.dense(inputs=states,units = 128,activation=tf.nn.relu)
layer_3 = tf.layers.dense(inputs=states,units = 128,activation=tf.nn.relu)
output = tf.layers.dense(inputs=layer_3,units = 6,activation=tf.nn.tanh)


loss = tf.losses.mean_squared_error(labels=actions,predictions=output)
optimizer = tf.train.AdamOptimizer(learning_rate = learning_rate).minimize(loss) 

config = tf.ConfigProto(allow_soft_placement = True)
config.gpu_options.allow_growth = True

sess = tf.Session(config=config)
sess.run(tf.global_variables_initializer())
print('network generation complete!!')

saver = tf.train.Saver()


