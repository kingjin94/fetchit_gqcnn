# -*- coding: utf-8 -*-
"""
Copyright ©2017. The Regents of the University of California (Regents). All Rights Reserved.
Permission to use, copy, modify, and distribute this software and its documentation for educational,
research, and not-for-profit purposes, without fee and without a signed licensing agreement, is
hereby granted, provided that the above copyright notice, this paragraph and the following two
paragraphs appear in all copies, modifications, and distributions. Contact The Office of Technology
Licensing, UC Berkeley, 2150 Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620, (510) 643-
7201, otl@berkeley.edu, http://ipira.berkeley.edu/industry-info for commercial licensing opportunities.

IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL,
INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF
THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF REGENTS HAS BEEN
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY, PROVIDED
HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
"""
"""
Displays robust grasps planned using a GQ-CNN-based policy on a set of saved RGB-D images.
The default configuration is cfg/examples/policy.yaml.

Author
------
Jeff Mahler

YAML Configuration File Parameters
----------------------------------
sensor/image_dir : str
    directory to the sample images, specified relative to /path/to/your/gqcnn/ (change this to try your own images!)
sensor/type : str
    type of sensor to use (virtual_primesense by default to use pre-stored images)
sensor/frame : str
    name of the sensor frame of references

calib_dir : str
    directory to the sample camera calibration files, specified relative to /path/to/your/gqcnn/

policy/gqcnn_model : str
    path to a directory containing a GQ-CNN model (change this to try your own networks!)
policy/num_seed_samples : int
    number of initial samples to take in the cross-entropy method (CEM) optimizer (smaller means faster grasp planning, lower-quality grasps)
policy/num_gmm_samples : int
    number of samples to take from the Gaussian Mixture Models on each iteration of the CEM optimizer
policy/num_iters : int
    number of sample-and-refit iterations of CEM
policy/gmm_refit_p : flota
    percentage of samples to use in the elite set on each iteration of CEM
policy/gmm_component_frac : float
    number of GMM components to use as a fraction of the sample size
policy/gmm_reg_covat : float
    regularization constant to ensure GMM sample diversity
policy/deterministic : bool
    True (1) if execution should be deterministic (via setting a random seed) and False (0) otherwise
policy/gripper_width : float
    distance between the jaws, in meters
policy/crop_height : int
    height of bounding box to use for cropping the image around a grasp candidate before passing it into the GQ-CNN
policy/crop_width : int
    width of bounding box to use for cropping the image around a grasp candidate before passing it into the GQ-CNN
policy/sampling/type : str
    grasp sampling type (use antipodal_depth to sample antipodal pairs in image space)
policy/sampling/friction_coef : float
    friction coefficient to use in sampling
policy/sampling/depth_grad_thresh : float
    threshold on depth image gradients for edge detection
policy/sampling/depth_grad_gaussian_sigma : float
    variance for gaussian filter to smooth image before taking gradients
policy/sampling/downsample_rate : float
    factor by which to downsample the image when detecting edges (larger number means edges are smaller images, which speeds up performance)
policy/sampling/max_rejection_samples : int
    maximum number of samples to take when sampling antipodal candidates (larger means potentially longer runtimes)
policy/sampling/max_dist_from_center : int
    maximum distance, in pixels, from the image center allowed in grasp sampling
policy/sampling/min_dist_from_boundary : int
    minimum distance, in pixels, of a grasp from the image boundary
policy/sampling/min_grasp_dist : float
    minimum distance between grasp vectors allowed in sampling (larger means greater sample diversity but potentially lower precision)
policy/sampling/angle_dist_weight : float
    weight for the distance between grasp axes in radians (we recommend keeping the default)
policy/sampling/depth_samples_per_grasp : int
    number of depth samples to take per independent antipodal grasp sample in image space
policy/sampling/depth_sample_win_height: int
    height of window used to compute the minimum depth for grasp depth sampling
policy/sampling/depth_sample_win_width: int
    width of window used to compute the minimum depth for grasp depth sampling
policy/sampling/min_depth_offset : float
    offset, in cm, from the min depth
policy/sampling/max_depth_offset : float
    offset, in cm, from the max depth

policy/vis/grasp_sampling : bool
    True (1) if grasp sampling should be displayed (for debugging)
policy/vis/tf_images : bool
    True (1) if transformed images should be displayed (for debugging)
policy/vis/grasp_candidates : bool
    True (1) if grasp candidates should be displayed (for debugging)
policy/vis/elite_grasps : bool
    True (1) if the elite set should be displayed (for debugging)
policy/vis/grasp_ranking : bool
    True (1) if the ranked grasps should be displayed (for debugging)
policy/vis/grasp_plan : bool
    True (1) if the planned grasps should be displayed (for debugging)
policy/vis/final_grasp : bool
    True (1) if the final planned grasp should be displayed (for debugging)
policy/vis/k : int
    number of grasps to display

inpaint_rescale_factor : float
    scale factor to resize the image by before inpainting (smaller means faster performance by less precise)

"""

"""
Modified version of gqcnn policy demo which takes in images from the fetch robot published on head_camera/depth_registered/image_raw and /head_camera/rgb/image_raw and publishes a grasp pose on /planned_grasp. The grasp pose encodes a postion to grasp at (with implict start width of 10 cm) and an angle to approach from as a quarternion
"""

from rospy import init_node, get_rostime, Publisher
init_node("gqcnn_planner")
from rospy.client import wait_for_message
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
bridge = CvBridge()
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

import argparse
import logging
import IPython
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage.morphology import binary_opening as binary_opening
import os
import sys
import time

from autolab_core import RigidTransform, YamlConfig
from perception import RgbdImage, RgbdSensorFactory, BinaryImage, DepthImage, ColorImage

from gqcnn import CrossEntropyAntipodalGraspingPolicy, RgbdImageState
from gqcnn import Visualizer as vis

if __name__ == '__main__':
    # set up logger
    logging.getLogger().setLevel(logging.DEBUG)

    # parse args
    parser = argparse.ArgumentParser(description='Run a GQ-CNN-based grasping policy')
    parser.add_argument('--config_filename', type=str, default='cfg/examples/policy.yaml', help='path to configuration file to use')
    args = parser.parse_args()
    config_filename = args.config_filename

    # make relative paths absolute
    if not os.path.isabs(config_filename):
        config_filename = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                       '..',
                                       config_filename)

    # read config
    config = YamlConfig(config_filename)
    sensor_type = config['sensor']['type']
    sensor_frame = config['sensor']['frame']
    inpaint_rescale_factor = config['inpaint_rescale_factor']
    policy_config = config['policy']

    # make relative paths absolute
    if not os.path.isabs(config['calib_dir']):
        config['calib_dir'] = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                           '..',
                                           config['calib_dir'])
    if not os.path.isabs(config['sensor']['image_dir']):
        config['sensor']['image_dir'] = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                                     '..',
                                                     config['sensor']['image_dir'])

    if not os.path.isabs(config['policy']['gqcnn_model']):
        config['policy']['gqcnn_model'] = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                                       '..',
                                                       config['policy']['gqcnn_model'])

    # read camera calib
    tf_filename = '%s_to_world.tf' %(sensor_frame)
    T_camera_world = RigidTransform.load(os.path.join(config['calib_dir'], sensor_frame, tf_filename))

    # setup sensor
    sensor = RgbdSensorFactory.sensor(sensor_type, config['sensor'])
    sensor.start()
    camera_intr = sensor.ir_intrinsics
  
    # read images, NEW: from ROS
    # color_im, depth_im, _ = sensor.frames()
    color_msg = wait_for_message("/head_camera/rgb/image_raw", Image)
    color_im = bridge.imgmsg_to_cv2(color_msg, "bgr8") # for the color images 
    color_im = ColorImage(cv2.cvtColor(color_im, cv2.COLOR_BGR2RGB)) # change color order for pyplot if color image
    depth_msg = wait_for_message("head_camera/depth_registered/image_raw", Image)
    depth_im = DepthImage(bridge.imgmsg_to_cv2(depth_msg)) # Returns a 480x640 float image
    color_im = color_im.inpaint(rescale_factor=inpaint_rescale_factor)
    depth_im = depth_im.inpaint(rescale_factor=inpaint_rescale_factor)
    rgbd_im = RgbdImage.from_color_and_depth(color_im, depth_im)
    
    # Find highest peak in histogram --> assumed to be the table surface
    histo = np.histogram(np.hstack(depth_im.data), bins='auto')
    maximizing_bin = np.argmax(histo[0])
    # print(maximizing_bin) # maximizer of all bin counts
    mostCommonDepth = histo[1][maximizing_bin]
    # print(mostCommonDepth) # maximizer depth
    # print(max(histo[0])) # maximizer count
    plt.title("Depth histogram")
    plt.hist(np.hstack(depth_im.data), bins='auto')
    plt.show()
    
    # Create raw segmask by thresholding about the table height
    segmask = np.logical_and(
				np.greater_equal(depth_im.data, (mostCommonDepth-0.1)*np.ones((480, 640))),
				np.less_equal(depth_im.data, (mostCommonDepth+0.1)*np.ones((480, 640)))
              )
    plt.subplot(1,3,1)
    plt.imshow(segmask)
    plt.title("Raw segmentation mask")
    plt.colorbar()
    plt.subplot(1,3,2)
    plt.imshow(depth_im.data)
    plt.colorbar()
    plt.title("Raw depth image")

    # refine to remove artifacts from segmentation
    plt.subplot(1,3,3)
    segmask = binary_opening(segmask, np.ones((50,50)))
    plt.imshow(segmask)
    plt.title("Segmantation mask after opening")
    plt.show()
    segmask = BinaryImage(255*segmask.astype(np.uint8))
    vis.figure(size=(10,10))
    vis.subplot(1,3,1)
    vis.imshow(rgbd_im.color)
    vis.title("Color image")
    vis.subplot(1,3,2)
    vis.imshow(rgbd_im.depth)
    vis.title("Depth image")
    vis.subplot(1,3,3)
    vis.imshow(segmask)
    vis.title("Segmentation mask")
    vis.show()
    state = RgbdImageState(rgbd_im, camera_intr, segmask=segmask)

    # init policy
    policy = CrossEntropyAntipodalGraspingPolicy(policy_config)
    policy_start = time.time()
    action = policy(state)
    logging.info('Planning took %.3f sec' %(time.time() - policy_start))
    print("Goal pose:")
    print(action.grasp.pose())
    print(action.grasp.pose().pose_msg)
    
    # Into world frame --> Get current camera to base, transform action.grasp.pose()
    tf_listener_ = tf.TransformListener()
    i = 0
    while(not (tf_listener_.frameExists("base_link") and tf_listener_.frameExists("head_camera_depth_optical_frame"))):
        i += 1

    now = get_rostime()
    while(not tf_listener_.canTransform("base_link", "head_camera_depth_optical_frame", now)):
        i += 1
        now = get_rostime()
    # print tf_listener_.lookupTransform("base_link", "head_camera_depth_optical_frame", now)
    hdr = Header(stamp=now, frame_id='head_camera_depth_optical_frame')
    pose_msg = PoseStamped(header=hdr, pose=action.grasp.pose().pose_msg)
    # print(pose_msg)
    # print(tf_listener_.transformPose("base_link", pose_msg))
    pub = Publisher('/planned_grasp', PoseStamped, queue_size=1, latch=True)
    pub.publish(pose_msg)


    # vis final grasp
    if policy_config['vis']['final_grasp']:
        vis.figure(size=(10,10))
        vis.subplot(1,2,1)
        vis.imshow(rgbd_im.color)
        vis.grasp(action.grasp, scale=1.5, show_center=False, show_axis=True)
        vis.title('Planned grasp on color (Q=%.3f)' %(action.q_value))
        vis.subplot(1,2,2)
        vis.imshow(rgbd_im.depth)
        vis.grasp(action.grasp, scale=1.5, show_center=False, show_axis=True)
        vis.title('Planned grasp on depth (Q=%.3f)' %(action.q_value))
        vis.show()

