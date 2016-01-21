#!/usr/bin/env python
import roslib; roslib.load_manifest('stereo_slam')
import pylab
import numpy as np
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
import tf.transformations as tf
import scipy.optimize as optimize
import collections
import math
import string

class Error(Exception):
  """ Base class for exceptions in this module. """
  pass

def trajectory_distances(data):
    dist = []
    dist.append(0)
    for i in range(len(data) - 1):
        dist.append(dist[i] + calc_dist(data[i, :], data[i + 1, : ]))
    return dist

def calc_dist_xyz(data_point1, data_point2):
    xdiff = data_point1[1] - data_point2[1]
    ydiff = data_point1[2] - data_point2[2]
    zdiff = data_point1[3] - data_point2[3]
    return xdiff, ydiff, zdiff

def calc_dist(data_point1, data_point2):
    xdiff, ydiff, zdiff = calc_dist_xyz(data_point1, data_point2)
    return math.sqrt(xdiff*xdiff + ydiff*ydiff + zdiff*zdiff)

def to_transform(data_point):
    t = [data_point[1], data_point[2], data_point[3]]
    q = [data_point[4], data_point[5], data_point[6], data_point[7]]
    rot_mat = tf.quaternion_matrix(q)
    trans_mat = tf.translation_matrix(t)
    return tf.concatenate_matrices(trans_mat, rot_mat)

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
    point = to_transform(data[i,:])
    point_d = tf.concatenate_matrices(point, tf_delta)
    t_d = tf.translation_from_matrix(point_d)
    q_d = tf.quaternion_from_matrix(point_d)
    corrected_data.append([data[i,0], t_d[0], t_d[1], t_d[2], q_d[0], q_d[1], q_d[2], q_d[3]])
  return np.array(corrected_data)

def apply_tf_to_vector(tf_delta, data):
  corrected_data = []
  for i in range(len(data)):
    point = to_transform(data[i,:])
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
  cur_delta = to_transform([0.0, 0.0, 0.0, 0.0, q[0], q[1], q[2], q[3]])

  # Compute the quadratic error for the current delta transformation
  err = 0.0
  for i in range(len(vertices)):
    # Compute the corrected ground truth
    tf_gt = to_transform(gt_rebased[i])
    tf_gt_t = tf.translation_from_matrix(tf_gt)
    tf_gt_t = np.array([tf_gt_t[0], tf_gt_t[1], tf_gt_t[2], 1.0])
    tf_gt_corrected = tf.concatenate_matrices(cur_delta, tf_gt_t)
    tf_gt_corr_vect = [0.0, tf_gt_corrected[0], tf_gt_corrected[1], tf_gt_corrected[2], 0.0, 0.0, 0.0, 1.0]

    # Compute the error
    err += np.power(calc_dist(vertices[i], tf_gt_corr_vect), 2)

  return np.sqrt(err)

def calc_errors(vector_1, vector_2):
  # Compute the errors between vectors
  assert(len(vector_1) == len(vector_1))
  output = []
  for i in range(len(vector_1)):
    output.append(calc_dist(vector_1[i,:], vector_2[i,:]))
  return np.array(output)

def calc_time_vector(data):
  output = []
  start_time = data[0,0]
  for i in range(len(data)):
    output.append((data[i,0] - start_time))
  return np.array(output)

def toRSTtable(rows, header=True, vdelim="  ", padding=1, justify='right'):
    """
    Outputs a list of lists as a Restructured Text Table
    - rows - list of lists
    - header - if True the first row is treated as a table header
    - vdelim - vertical delimiter between columns
    - padding - nr. of spaces are left around the longest element in the column
    - justify - may be left, center, right
    """
    border="=" # character for drawing the border
    justify = {'left':string.ljust,'center':string.center,'right':string.rjust}[justify.lower()]

    # calculate column widhts (longest item in each col
    # plus "padding" nr of spaces on both sides)
    cols = zip(*rows)
    colWidths = [max([len(str(item))+2*padding for item in col]) for col in cols]

    # the horizontal border needed by rst
    borderline = vdelim.join([w*border for w in colWidths])

    # outputs table in rst format
    output = ""
    output += borderline + "\n"
    for row in rows:
        output += vdelim.join([justify(str(item),width) for (item,width) in zip(row,colWidths)])
        output += "\n"
        if header: output += borderline + "\n"; header=False
    output += borderline + "\n"
    print output
    return output


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
  parser.add_argument('orb_file',
          help='file corresponding to the ORB-SLAM trajectory')
  args = parser.parse_args()
  colors = ['g','r','b']
  angles = [-6, 6, -4, 4, -12, 12]

  # Setup the font for the graphics
  font = {'family' : 'Sans',
          'weight' : 'normal',
          'size'   : 35}
  pylab.rc('font', **font)
  linewidth = 3.0

  # Log
  print "Adjusting curves, please wait..."

  # Init figures
  fig0 = pylab.figure()
  ax0 = Axes3D(fig0)
  ax0.grid(True)
  ax0.set_xlabel("x (m)")
  ax0.set_ylabel("y (m)")
  ax0.set_zlabel("z (m)")

  fig1 = pylab.figure()
  ax1 = fig1.gca()
  ax1.grid(True)
  ax1.set_xlabel("x (m)")
  ax1.set_ylabel("y (m)")

  fig2 = pylab.figure()
  ax2 = fig2.gca()
  ax2.grid(True)
  ax2.set_xlabel("x (m)")
  ax2.set_ylabel("y (m)")

  fig3 = pylab.figure()
  ax3 = fig3.gca()
  ax3.grid(True)
  ax3.set_xlabel("x (m)")
  ax3.set_ylabel("y (m)")

  fig4 = pylab.figure()
  ax4 = fig4.gca()
  ax4.grid(True)
  ax4.set_xlabel("x (m)")
  ax4.set_ylabel("y (m)")

  # Load ground truth (gt) data.
  # Check the file type
  f = open(args.ground_truth_file)
  lines = f.readlines()
  f.close()
  size = lines[1].split(",")

  if (len(size) == 8 or len(size) >= 12):
    gt = pylab.loadtxt(args.ground_truth_file, delimiter=',', comments='%', usecols=(0,5,6,7,8,9,10,11))
    gt[:,0] = gt[:,0] / 1000000000
  else:
    gt = pylab.loadtxt(args.ground_truth_file, delimiter=',', comments='%', usecols=(0,1,2,3,4,5,6,7))

  # Load the visual odometry
  odom = pylab.loadtxt(args.visual_odometry_file, delimiter=',', comments='%', usecols=(0,5,6,7,8,9,10,11))
  # odom = pylab.loadtxt(args.visual_odometry_file, delimiter=',', comments='%', usecols=(0,4,5,6,7,8,9,10))
  odom[:,0] = odom[:,0] / 1000000000

  # Load the graph vertices
  vertices = pylab.loadtxt(args.graph_vertices_file, delimiter=',', usecols=(0,2,3,4,5,6,7,8))

  # Load the ORB-SLAM trajectory
  orb = pylab.loadtxt(args.orb_file, delimiter=',', usecols=(0,2,3,4,5,6,7,8))

  # Get the gt indeces for all graph vertices
  gt_rebased = rebase(vertices, gt)
  odom_rebased = rebase(vertices, odom)
  orb_rebased = rebase(vertices, orb)

  # Compute the translation to make the same origin for all curves
  first_vertice = to_transform(vertices[0,:])
  first_gt_coincidence = to_transform(gt_rebased[0,:])
  tf_delta = tf.concatenate_matrices(tf.inverse_matrix(first_gt_coincidence), first_vertice)

  # Move all the gt and odometry points with the correction
  gt_moved = apply_tf_to_matrix(tf_delta, gt)
  gt_rb_moved = apply_tf_to_matrix(tf_delta, gt_rebased)
  odom_moved = apply_tf_to_matrix(tf_delta, odom)
  odom_rb_moved = apply_tf_to_matrix(tf_delta, odom_rebased)
  orb_moved = apply_tf_to_matrix(tf_delta, orb)
  orb_rb_moved = apply_tf_to_matrix(tf_delta, orb_rebased)

  # Transform optimization
  Param = collections.namedtuple('Param','roll pitch yaw')
  rranges = ((angles[0]*np.pi/180, angles[1]*np.pi/180, 0.04), (angles[2]*np.pi/180, angles[3]*np.pi/180, 0.02), (angles[4]*np.pi/180, angles[5]*np.pi/180, 0.04))
  p = optimize.brute(sigmoid, rranges, args=(vertices, gt_rebased))
  p = Param(*p)

  # Build the rotation matrix
  roll, pitch, yaw = p
  q = quaternion_from_rpy(roll, pitch, yaw)
  tf_correction = to_transform([0.0, 0.0, 0.0, 0.0, q[0], q[1], q[2], q[3]])

  # Correct ground truth and odometry
  gt_corrected = apply_tf_to_vector(tf_correction, gt_moved)
  gt_rb_corrected = apply_tf_to_vector(tf_correction, gt_rb_moved)
  odom_corrected = apply_tf_to_vector(tf_correction, odom_moved)
  odom_rb_corrected = apply_tf_to_vector(tf_correction, odom_rb_moved)
  orb_corrected = apply_tf_to_vector(tf_correction, orb_moved)
  orb_rb_corrected = apply_tf_to_vector(tf_correction, orb_rb_moved)

  # Compute the errors
  print "Computing errors, please wait..."
  gt_dist = trajectory_distances(gt_rb_corrected)
  odom_dist = trajectory_distances(odom_rb_corrected)
  vertices_dist = trajectory_distances(vertices)
  orb_dist = trajectory_distances(orb_rb_corrected)
  odom_errors = calc_errors(gt_rb_corrected, odom_rb_corrected)
  vertices_errors = calc_errors(gt_rb_corrected, vertices)
  orb_errors = calc_errors(gt_rb_corrected, orb_rb_corrected)
  time = calc_time_vector(gt_rb_corrected)
  odom_mae = np.average(np.abs(odom_errors), 0)
  vertices_mae = np.average(np.abs(vertices_errors), 0)
  orb_mae = np.average(np.abs(orb_errors), 0)

  rows = []
  rows.append(['Viso2'] + [len(odom_errors)] + [odom_dist[-1]] + [odom_mae])
  rows.append(['ORB-SLAM'] + [len(orb_errors)] + [orb_dist[-1]] + [orb_mae])
  rows.append(['Stereo-SLAM'] + [len(vertices_errors)] + [vertices_dist[-1]] + [vertices_mae])

  # Build the header for the output table
  header = [  "Input", "Data Points", "Traj. Distance (m)", "Trans. MAE (m)"]
  toRSTtable([header] + rows)

  print "Ground truth distance: ", gt_dist[-1], "m"

  # Plot graph (3D)
  ax0.plot(gt_corrected[:,1], gt_corrected[:,2], gt_corrected[:,3], colors[0], linewidth=linewidth, label='Ground Truth')
  ax0.plot(odom_corrected[:,1], odom_corrected[:,2], odom_corrected[:,3], colors[1], linewidth=linewidth, label='Viso2')
  ax0.plot(orb_corrected[:,1], orb_corrected[:,2], orb_corrected[:,3], 'y', linewidth=linewidth, label='ORB-SLAM')
  ax0.plot(vertices[:,1], vertices[:,2], vertices[:,3], colors[2], linewidth=linewidth, label='Stereo-SLAM', marker='o')

  # Plot graph (2D)
  ax1.plot(gt_corrected[:,1], gt_corrected[:,2], colors[0], linewidth=linewidth, label='Ground truth')
  ax1.plot(odom_corrected[:,1], odom_corrected[:,2], colors[1], linewidth=linewidth, label='Viso2')
  ax1.plot(orb_corrected[:,1], orb_corrected[:,2], 'y', linewidth=linewidth, label='ORB-SLAM')
  ax1.plot(vertices[:,1], vertices[:,2], colors[2], linewidth=linewidth, label='Stereo-SLAM', marker='o')

  # Plot the graph edges
  f = open(args.graph_edges_file)
  lines = f.readlines()
  f.close()
  if (len(lines) > 0):
    edges = pylab.loadtxt(args.graph_edges_file, delimiter=',', usecols=(3,4,5,10,11,12))
    for i in range(len(edges)):
      vect = []
      vect.append([edges[i,0], edges[i,1], edges[i,2]])
      vect.append([edges[i,3], edges[i,4], edges[i,5]])
      vect =  np.array(vect)
      ax0.plot(vect[:,0], vect[:,1], vect[:,2], colors[2], linewidth=linewidth-1, linestyle='--')
      ax1.plot(vect[:,0], vect[:,1], colors[2], linewidth=linewidth-1, linestyle='--')

  ax0.legend(loc=2)
  ax1.legend(loc=2)

  # Plot individual graphs (2D)
  ax2.plot(gt_corrected[:,1], gt_corrected[:,2], 'g', linewidth=linewidth, label='Ground truth')
  ax2.plot(odom_corrected[:,1], odom_corrected[:,2], 'b', linewidth=linewidth, label='Viso2')
  ax2.legend(loc=2)

  ax3.plot(gt_corrected[:,1], gt_corrected[:,2], 'g', linewidth=linewidth, label='Ground truth')
  ax3.plot(orb_corrected[:,1], orb_corrected[:,2], 'b', linewidth=linewidth, label='ORB-SLAM')
  ax3.legend(loc=2)

  ax4.plot(gt_corrected[:,1], gt_corrected[:,2], 'g', linewidth=linewidth, label='Ground truth')
  ax4.plot(vertices[:,1], vertices[:,2], 'b', linewidth=linewidth, label='Stereo-SLAM')
  ax4.legend(loc=2)


  # Plot errors
  fig5 = pylab.figure()
  ax5 = fig5.gca()
  ax5.plot(odom_dist, odom_errors, colors[1], linewidth=linewidth, label='Viso2')
  ax5.plot(orb_dist, orb_errors, 'g', linewidth=linewidth, label='ORB-SLAM')
  ax5.plot(vertices_dist, vertices_errors, colors[2], linewidth=linewidth, label='Stereo-SLAM')
  ax5.grid(True)
  ax5.set_xlabel("Distance (m)")
  ax5.set_ylabel("Error (m)")
  ax5.legend(loc=2)
  ax5.tick_params(axis='both', which='major', labelsize=40);
  ax5.set_xlim(0, 52)

  pyplot.draw()
  pylab.show()