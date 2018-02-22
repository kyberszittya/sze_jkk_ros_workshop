'''
Created on 2018. febr. 18.

@author: kyberszittya
'''
# ROS
import rospy
from ros_workshop_msgs.srv import ReinforcementSimulationStart, ReinforcementSimulationStartRequest, ReinforcementSimulationStartResponse
from ros_workshop_msgs.msg import ReinforcementObservationUpdate
# By Aurelion Geron
import gym
import numpy as np
import tensorflow as tf
from flask.globals import session

from PIL import Image, ImageDraw

import matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt

try:
    from pyglet.gl import gl_info
    openai_cart_pole_rendering = True   # no problem, let's use OpenAI gym's rendering function
except Exception:
    openai_cart_pole_rendering = False  # probably no X server available, let's use our own rendering function

def update_scene(num, frames, patch):
    patch.set_data(frames[num])
    return patch,

def plot_animation(frames, repeat=False, interval=40):
    plt.close()  # or else nbagg sometimes plots in the previous cell
    fig = plt.figure()
    patch = plt.imshow(frames[0])
    plt.axis('off')
    return animation.FuncAnimation(fig, update_scene, fargs=(frames, patch), frames=len(frames), repeat=repeat, interval=interval)

def render_cart_pole(env, obs):
    if openai_cart_pole_rendering:
        # use OpenAI gym's rendering function
        return env.render(mode="rgb_array")
    else:
        # rendering for the cart pole environment (in case OpenAI gym can't do it)
        img_w = 600
        img_h = 400
        cart_w = img_w // 12
        cart_h = img_h // 15
        pole_len = img_h // 3.5
        pole_w = img_w // 80 + 1
        x_width = 2
        max_ang = 0.2
        bg_col = (255, 255, 255)
        cart_col = 0x000000 # Blue Green Red
        pole_col = 0x669acc # Blue Green Red

        pos, vel, ang, ang_vel = obs
        img = Image.new('RGB', (img_w, img_h), bg_col)
        draw = ImageDraw.Draw(img)
        cart_x = pos * img_w // x_width + img_w // x_width
        cart_y = img_h * 95 // 100
        top_pole_x = cart_x + pole_len * np.sin(ang)
        top_pole_y = cart_y - cart_h // 2 - pole_len * np.cos(ang)
        draw.line((0, cart_y, img_w, cart_y), fill=0)
        draw.rectangle((cart_x - cart_w // 2, cart_y - cart_h // 2, cart_x + cart_w // 2, cart_y + cart_h // 2), fill=cart_col) # draw cart
        draw.line((cart_x, cart_y - cart_h // 2, top_pole_x, top_pole_y), fill=pole_col, width=pole_w) # draw pole
        return np.array(img)

def plot_cart_pole(env, obs):
    plt.close()  # or else nbagg sometimes plots in the previous cell
    img = render_cart_pole(env, obs)
    plt.imshow(img)
    plt.axis("off")
    plt.show()

def render_policy_net(model_path, action, X, n_max_steps = 1000):
    frames = []
    env = gym.make("CartPole-v0")
    obs = env.reset()
    with tf.Session() as sess:
        saver.restore(sess, model_path)
        for step in range(n_max_steps):
            img = render_cart_pole(env, obs)
            frames.append(img)
            action_val = action.eval(feed_dict={X: obs.reshape(1, n_inputs)})
            obs, reward, done, info = env.step(action_val[0][0])
            print(obs)
            if done:
                break
    env.close()
    return frames


def reset_graph(seed=42):
    tf.reset_default_graph()
    tf.set_random_seed(seed)
    np.random.seed(seed)

def discount_rewards(rewards, discount_rate):
    discounted_rewards = np.zeros(len(rewards))
    cumulative_rewards = 0
    for step in reversed(range(len(rewards))):
        cumulative_rewards = rewards[step] + cumulative_rewards * discount_rate
        discounted_rewards[step] = cumulative_rewards
    return discounted_rewards

def discount_and_normalize_rewards(all_rewards, discount_rate):
    all_discounted_rewards = [discount_rewards(rewards, discount_rate) for rewards in all_rewards]
    flat_rewards = np.concatenate(all_discounted_rewards)
    reward_mean = flat_rewards.mean()
    reward_std = flat_rewards.std()
    return [(discounted_rewards - reward_mean)/reward_std for discounted_rewards in all_discounted_rewards]

class RunSimulationServer(object):
    def __init__(self):
        self.obs_pub = rospy.Publisher('/rl_obs', ReinforcementObservationUpdate, queue_size=10)
        self.n_inputs = 4
        self.n_hidden = 4
        self.n_outputs = 1
        self.learning_rate = 0.01

        
        


    def run_simulation(self,req):
        reset_graph()
        
        initializer = tf.contrib.layers.variance_scaling_initializer()

        self.X = tf.placeholder(tf.float32, shape=[None, self.n_inputs])


        hidden = tf.layers.dense(self.X, self.n_hidden, 
            activation=tf.nn.elu, 
            kernel_initializer=initializer)
        logits = tf.layers.dense(hidden, 
            self.n_outputs)
        outputs = tf.nn.sigmoid(logits)
        p_left_and_right = tf.concat(axis=1, values=[outputs, 1 - outputs])
        self.action = tf.multinomial(tf.log(p_left_and_right), num_samples=1)

        self.y = 1- tf.to_float(self.action)

        cross_entropy = tf.nn.sigmoid_cross_entropy_with_logits(labels=self.y, logits=logits)
        optimizer = tf.train.AdamOptimizer(self.learning_rate)
        # Gradient setup
        self.grads_and_vars = optimizer.compute_gradients(cross_entropy)
        self.gradients = [grad for grad, variable in self.grads_and_vars]
        self.gradient_placeholders = []
        self.grads_and_vars_feed = []
        for grad, variable in self.grads_and_vars:
            self.gradient_placeholder = tf.placeholder(tf.float32, shape=grad.get_shape())
            self.gradient_placeholders.append(self.gradient_placeholder)
            self.grads_and_vars_feed.append((self.gradient_placeholder, variable))
            

        self.training_op = optimizer.apply_gradients(self.grads_and_vars_feed)

        self.init = tf.global_variables_initializer()
        self.saver = tf.train.Saver()

        env = gym.make("CartPole-v0")
        # n_games_per_update = 10
        # n_max_steps = 1000
        # n_iterations = 250
        # save_iterations = 10
        # discount_rate = 0.95
        n_games_per_update = req.n_games_per_update
        n_max_steps = req.n_max_steps
        n_iterations = req.n_iterations
        save_iterations = req.save_iterations
        discount_rate = req.discount_rate
        msg_obs = ReinforcementObservationUpdate()
        msg_obs.sum_grad = 0.0

        with tf.Session() as sess:
            self.init.run()
            for iteration in range(n_iterations):
                print("Iteration: {}".format(iteration))        
                all_rewards = []
                all_gradients = []
                for game in range(n_games_per_update):
                    current_rewards = []
                    current_gradients = []
                    obs = env.reset()
                    for step in range(n_max_steps):
                        action_val, gradients_val = sess.run([self.action, self.gradients], feed_dict = {self.X: obs.reshape(1, self.n_inputs)})
                        obs, reward, done, info = env.step(action_val[0][0])
                        msg_obs.obs = obs
                        #msg_obs.sum_obs += obs
                        current_rewards.append(reward)
                        current_gradients.append(gradients_val)
                        #msg_obs.grad_abs = np.sum(np.sum(gradients_val))[0]
                        s = np.sum(np.sum(gradients_val))
                        msg_obs.grad_abs = float(s)
                        msg_obs.sum_grad += float(s)
                        self.obs_pub.publish(msg_obs)
                        if done:
                            break
                        
                    all_rewards.append(current_rewards)
                    all_gradients.append(current_gradients)
                all_rewards = discount_and_normalize_rewards(all_rewards, discount_rate)
                feed_dict = {}
                for var_index, gradient_placeholder in enumerate(self.gradient_placeholders):
                    mean_gradients = np.mean([reward * all_gradients[game_index][step][var_index]
                                            for game_index, rewards in enumerate(all_rewards)
                                                for step, reward in enumerate(rewards)], axis=0)
                    feed_dict[gradient_placeholder] = mean_gradients
                sess.run(self.training_op, feed_dict = feed_dict)
                if iteration % save_iterations == 0:
                    self.saver.save(sess, "./my_policy_net_pg.ckpt")            
        env.close()
        resp = ReinforcementSimulationStartResponse()
        #resp.gradients = all_gradients
        resp.success = True
        return resp

if __name__=="__main__":
    rospy.init_node('run_simulation_node')
    r = RunSimulationServer()
    s = rospy.Service('run_simulation', ReinforcementSimulationStart, r.run_simulation)
    print("Ready to start.")
    rospy.spin()
