#!/usr/bin/env python
import roslib; roslib.load_manifest('stereo_slam')
import pylab
import numpy as np
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
import tf.transformations as tf
import scipy.optimize as optimize
import collections
from odometry_evaluation import utils

class Error(Exception):
  """ Base class for exceptions in this module. """
  pass

def rebase(base_vector, rebased_vector):
  min_idx_vec = []
  for i in range(len(base_vector)):
    # Search the best coincidence in time with ground truth
    min_dist = 99999
    min_idx = -1
    for j in range(len(rebased_vector)):
      dist = abs(rebased_vector[j,0] - base_vector[i,0])
      if dist < min_dist:
        min_dist = dist
        min_idx = j
    min_idx_vec.append(min_idx)
  min_idx_vec = np.array(min_idx_vec)
  return rebased_vector[min_idx_vec,:]


def apply_tf_to_matrix(tf_delta, data):
  corrected_data = []
  for i in range(len(data)):
    point = utils.to_transform(data[i,:])
    point_d = tf.concatenate_matrices(point, tf_delta)
    t_d = tf.translation_from_matrix(point_d)
    q_d = tf.quaternion_from_matrix(point_d)
    corrected_data.append([data[i,0], t_d[0], t_d[1], t_d[2], q_d[0], q_d[1], q_d[2], q_d[3]])
  return np.array(corrected_data)

def apply_tf_to_vector(tf_delta, data):
  corrected_data = []
  for i in range(len(data)):
    point = utils.to_transform(data[i,:])
    t_d = tf.translation_from_matrix(point)
    q_d = tf.quaternion_from_matrix(point)
    t_d = np.array([t_d[0], t_d[1], t_d[2], 1.0])
    point_mod = tf.concatenate_matrices(tf_delta, t_d)
    corrected_data.append([data[i,0], point_mod[0], point_mod[1], point_mod[2], t_d[2], q_d[0], q_d[1], q_d[2], q_d[3]])
  return np.array(corrected_data)

def quaternion_from_rpy(roll, pitch, yaw):
  q = []
  q.append( np.cos(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)*np.sin(pitch/2)*np.sin(yaw/2))
  q.append( np.sin(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) - np.cos(roll/2)*np.sin(pitch/2)*np.sin(yaw/2))
  q.append( -np.cos(roll/2)*np.sin(pitch/2)*np.cos(yaw/2) - np.sin(roll/2)*np.cos(pitch/2)*np.sin(yaw/2))
  q.append( np.cos(roll/2)*np.cos(pitch/2)*np.sin(yaw/2) - np.sin(roll/2)*np.sin(pitch/2)*np.cos(yaw/2))
  q = np.array(q)
  return q

def sigmoid(p, vertices, gt_rebased):
  # Get the current parameter set and build the transform
  roll, pitch, yaw = p
  q = quaternion_from_rpy(roll, pitch, yaw)
  cur_delta = utils.to_transform([0.0, 0.0, 0.0, 0.0, q[0], q[1], q[2], q[3]])
  
  # Compute the quadratic error for the current delta transformation
  err = 0.0
  for i in range(len(vertices)):
    # Compute the corrected ground truth
    tf_gt = utils.to_transform(gt_rebased[i])
    tf_gt_t = tf.translation_from_matrix(tf_gt)
    tf_gt_t = np.array([tf_gt_t[0], tf_gt_t[1], tf_gt_t[2], 1.0])
    tf_gt_corrected = tf.concatenate_matrices(cur_delta, tf_gt_t)
    tf_gt_corr_vect = [0.0, tf_gt_corrected[0], tf_gt_corrected[1], tf_gt_corrected[2], 0.0, 0.0, 0.0, 1.0]

    # Compute the error
    err += np.power(utils.calc_dist(vertices[i], tf_gt_corr_vect), 2)

  return np.sqrt(err)

def calc_errors(vector_1, vector_2):
  # Compute the errors between vectors
  assert(len(vector_1) == len(vector_1))
  output = []
  for i in range(len(vector_1)):
    output.append(utils.calc_dist(vector_1[i,:], vector_2[i,:]))
  return np.array(output)

def calc_time_vector(data):
  output = []
  start_time = data[0,0]
  for i in range(len(data)):
    output.append((data[i,0] - start_time))
  return np.array(output)

if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(
          description='Plot 3D graphics of SLAM.',
          formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('ground_truth_file', 
          help='file with ground truth')
  parser.add_argument('visual_odometry_file', 
          help='file with visual odometry')
  parser.add_argument('graph_vertices_file', 
          help='file the vertices of stereo slam')
  parser.add_argument('graph_edges_file', 
          help='file the edges of stereo slam')
  args = parser.parse_args()
  colors = ['g','r','b']
  angles = [-6, 6, -4, 4, -12, 12]

  # Log
  print "Adjusting curves, please wait..."

  # Init figure
  fig = pylab.figure(1)
  ax = Axes3D(fig)
  ax.grid(True)
  ax.set_title("Graph Viewer")
  ax.set_xlabel("X")
  ax.set_ylabel("Y")
  ax.set_zlabel("Z")

  # Load ground truth (gt) data.
  # Check the file type
  f = open(args.ground_truth_file)
  lines = f.readlines()
  f.close()
  size = lines[1].split(",")

  if (len(size) > 12):
    gt = pylab.loadtxt(args.ground_truth_file, delimiter=',', skiprows=1, usecols=(0,5,6,7,8,9,10,11))
    gt[:,0] = gt[:,0] / 1000000000
  else:
    gt = pylab.loadtxt(args.ground_truth_file, delimiter=',', usecols=(0,1,2,3,4,5,6,7))

  # Load the visual odometry
  odom = pylab.loadtxt(args.visual_odometry_file, delimiter=',', skiprows=1, usecols=(0,5,6,7,8,9,10,11))
  odom[:,0] = odom[:,0] / 1000000000
  
  # Load the graph vertices
  vertices = pylab.loadtxt(args.graph_vertices_file, delimiter=',', skiprows=0, usecols=(0,5,6,7,8,9,10,11))
  ax.plot(vertices[:,1], vertices[:,2], vertices[:,3], colors[2], label='Stereo slam', marker='o')

  # Load the graph edges
  edges = pylab.loadtxt(args.graph_edges_file, delimiter=',', skiprows=0, usecols=(1,2,3,4,5,6))
  for i in range(len(edges)):
    vect = []
    vect.append([edges[i,0], edges[i,1], edges[i,2]])
    vect.append([edges[i,3], edges[i,4], edges[i,5]])
    vect =  np.array(vect)
    ax.plot(vect[:,0], vect[:,1], vect[:,2], colors[2], linestyle='--')

  # Get the gt indeces for all graph vertices
  gt_rebased = rebase(vertices, gt)
  odom_rebased = rebase(vertices, odom)

  # Compute the translation to make the same origin for all curves
  first_vertice = utils.to_transform(vertices[0,:])
  first_gt_coincidence = utils.to_transform(gt_rebased[0,:])
  tf_delta = tf.concatenate_matrices(tf.inverse_matrix(first_gt_coincidence), first_vertice)

  # Move all the gt and odometry points with the correction
  gt_moved = apply_tf_to_matrix(tf_delta, gt)
  gt_rb_moved = apply_tf_to_matrix(tf_delta, gt_rebased)
  odom_moved = apply_tf_to_matrix(tf_delta, odom)
  odom_rb_moved = apply_tf_to_matrix(tf_delta, odom_rebased)

  # Transform optimization
  Param = collections.namedtuple('Param','roll pitch yaw')
  rranges = ((angles[0]*np.pi/180, angles[1]*np.pi/180, 0.04), (angles[2]*np.pi/180, angles[3]*np.pi/180, 0.02), (angles[4]*np.pi/180, angles[5]*np.pi/180, 0.04))
  p = optimize.brute(sigmoid, rranges, args=(vertices, gt_rebased))
  p = Param(*p)

  # Build the rotation matrix
  roll, pitch, yaw = p
  q = quaternion_from_rpy(roll, pitch, yaw)
  tf_correction = utils.to_transform([0.0, 0.0, 0.0, 0.0, q[0], q[1], q[2], q[3]])

  # Correct ground truth and odometry
  gt_corrected = apply_tf_to_vector(tf_correction, gt_moved)
  gt_rb_corrected = apply_tf_to_vector(tf_correction, gt_rb_moved)
  odom_corrected = apply_tf_to_vector(tf_correction, odom_moved)
  odom_rb_corrected = apply_tf_to_vector(tf_correction, odom_rb_moved)

  # Compute the errors
  print "Computing errors, please wait..."
  odom_dist = utils.trajectory_distances(odom_rb_corrected)
  vertices_dist = utils.trajectory_distances(vertices)
  odom_errors = calc_errors(gt_rb_corrected, odom_rb_corrected)
  vertices_errors = calc_errors(gt_rb_corrected, vertices)
  time = calc_time_vector(gt_rb_corrected)
  odom_mae = np.average(np.abs(odom_errors), 0)
  vertices_mae = np.average(np.abs(vertices_errors), 0)

  rows = []
  rows.append(['Odometry'] + [len(odom_errors)] + [odom_dist[-1]] + [odom_mae])
  rows.append(['SLAM'] + [len(vertices_errors)] + [vertices_dist[-1]] + [vertices_mae])

  # Build the header for the output table
  header = [  "Input", "Data Points", "Traj. Distance (m)", "Trans. MAE (m)"]
  utils.toRSTtable([header] + rows)

  # Plot graph
  ax.plot(gt_corrected[:,1], gt_corrected[:,2], gt_corrected[:,3], colors[0], label='Ground Truth')
  ax.plot(odom_corrected[:,1], odom_corrected[:,2], odom_corrected[:,3], colors[1], label='Visual Odometry')
  ax.legend()

  # Plot errors
  fig1 = pylab.figure()
  ax1 = fig1.gca()
  ax1.plot(time, odom_errors, label='Odometry', marker='o')
  ax1.plot(time, vertices_errors, label='SLAM', marker='o')
  ax1.grid(True)
  ax1.set_title("Error: Odometry vs SLAM")
  ax1.set_xlabel("Time (s)")
  ax1.set_ylabel("Error (m)")
  ax1.legend(loc=2)
  fig2 = pylab.figure()
  ax2 = fig2.gca()
  ax2.plot(odom_dist, odom_errors, label='Odometry', marker='o')
  ax2.plot(vertices_dist, vertices_errors, label='SLAM', marker='o')
  ax2.grid(True)
  ax2.set_title("Error: Odometry vs SLAM")
  ax2.set_xlabel("Distance (m)")
  ax2.set_ylabel("Error (m)")
  ax2.legend(loc=2)
  
  pyplot.draw()
  pylab.show()