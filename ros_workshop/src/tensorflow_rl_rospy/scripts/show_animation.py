from ros_workshop_msgs.srv import ReinforcementSimulationStartRequest, ReinforcementSimulationStart

import sys
import rospy

import gym
import numpy as np
import tensorflow as tf
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

def run_simulation_client(n, x, c, v, a):
    rospy.wait_for_service('run_simulation')
    try:
        run_simulation = rospy.ServiceProxy('run_simulation', ReinforcementSimulationStart)
        resp1 = run_simulation(n, x, c, v, a)
        return resp1.gradients
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    n_inputs = 4
    n_hidden = 4
    n_outputs = 1
    initializer = tf.contrib.layers.variance_scaling_initializer()
    X = tf.placeholder(tf.float32, shape=[None, n_inputs])
    
    hidden = tf.layers.dense(X, n_hidden, activation=tf.nn.elu, kernel_initializer=initializer)
    logits = tf.layers.dense(hidden, n_outputs)
    outputs = tf.nn.sigmoid(logits)  # probability of action 0 (left)
    p_left_and_right = tf.concat(axis=1, values=[outputs, 1 - outputs])
    action = tf.multinomial(tf.log(p_left_and_right), num_samples=1)
    saver = tf.train.Saver()
    # n_games_per_update = 10
    # n_max_steps = 1000
    # n_iterations = 250
    # save_iterations = 10
    # discount_rate = 0.95
    n_iter = 250
    print "Requesting games_per_update: %s\n n_max_steps: %s\n n_iterations: %s\n save_iterations%s\ndiscount rate: %s"%(10, 1000, n_iter, 25, 0.95)
    print run_simulation_client(10, 1000, n_iter, 25, 0.95)
    frames = render_policy_net("./my_policy_net_pg.ckpt", action, X, n_max_steps=1000)
    video = plot_animation(frames)
    plt.show()