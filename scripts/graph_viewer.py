#!/usr/bin/env python
import roslib; roslib.load_manifest('stereo_slam')
import sys
import pylab
import math
import numpy as np
import weakref
import string
import random
import time
import ntpath
import os
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
import tf.transformations as tf

# Global variables
blocking_file = ""
graph_edges_file = ""
first_iter = True
colors = ['g','r','b']
ax_gt = None
ax_odom = None
ax_vertices = None
ax_edges = []
edges_shown = True
correct_graph = False
gt_data = []

class Error(Exception):
  """ Base class for exceptions in this module. """
  pass

def to_transform(data_point):
  t = [data_point[1], data_point[2], data_point[3]]
  q = [data_point[4], data_point[5], data_point[6], data_point[7]]
  rot_mat = tf.quaternion_matrix(q)
  trans_mat = tf.translation_matrix(t)
  return tf.concatenate_matrices(trans_mat, rot_mat)

def real_time_plot(odom_file, graph_vertices_file):
  """
  Function to plot the data saved into the files in real time
  """
  global blocking_file, first_iter, colors, ax_gt, ax_odom, ax_vertices, edges_shown, correct_graph, gt_data

  # Load visual odometry data (saved with rostopic echo -p /stereo_odometer/odometry > file.txt)
  if (odom_file != "" and os.path.exists(odom_file)):
    try:
      data = pylab.loadtxt(odom_file, delimiter=',', skiprows=1, usecols=(5,6,7,8,9,10,11))
      # Remove old line collection
      if (first_iter == False):
        l = ax_odom.pop(0)
        wl = weakref.ref(l)
        l.remove()
        del l
      ax_odom = ax.plot(data[:,0], data[:,1], data[:,2], colors[1], label='Visual Odometry')
    except:
      print "No data in ", odom_file

  # Load stereo slam vertices (file saved with node stereo_slam)
  if (graph_vertices_file != "" and os.path.exists(graph_vertices_file)):

    # Check if file is blocked
    while (os.path.exists(blocking_file) and os.path.isfile(blocking_file)):
      time.sleep(0.5)

    try:
      data = pylab.loadtxt(graph_vertices_file, delimiter=',', skiprows=0, usecols=(0,5,6,7,8,9,10,11))
      # Remove old line collection
      if (first_iter == False):
        l = ax_vertices.pop(0)
        wl = weakref.ref(l)
        l.remove()
        del l

      # Plot vertices
      ax_vertices = ax.plot(data[:,1], data[:,2], data[:,3], colors[2], label='Stereo slam', marker='o')
      
      # Correct data if needed
      if (correct_graph == True):

        # Correct vertices time if needed
        if (data[0,0] < 9999999999):
          data[:,0] = data[:,0] * 1000000000

        # Search the best matching between graph vertices and ground truth
        min_time_diff = 999999999999
        min_time_idx = -1
        for i in range(len(gt_data)):
          time_diff = np.fabs(data[0,0] - gt_data[i,0])
          if (time_diff < min_time_diff):
            min_time_diff = time_diff
            min_time_idx = i

        # Sanity check
        if (min_time_idx == -1):
          print "Impossible to find time matchings between ground truth poses and graph vertices"
        else:
          # Build the tf for the first graph vertice
          tf_vertice_1 = to_transform(data[0,:])
          tf_gt = to_transform(gt_data[min_time_idx,:])
          tf_delta = tf.concatenate_matrices(tf.inverse_matrix(tf_gt), tf_vertice_1)

          print "Transformation: "
          print tf_delta
          
          # Apply the transformation to all gt poses
          gt_data_cor = []
          for i in range(len(gt_data)):
            gt_pose = to_transform(gt_data[i,:])
            gt_pose_cor = tf.concatenate_matrices(gt_pose, tf_delta)
            translation_vec = tf.translation_from_matrix(gt_pose_cor)
            gt_data_cor.append([gt_data[i,0], translation_vec[0], translation_vec[1], translation_vec[2]])
          gt_data_cor =  np.array(gt_data_cor)

          # Remove old line collection
          if (first_iter == False):
            l = ax_gt.pop(0)
            wl = weakref.ref(l)
            l.remove()
            del l

          # Draw the corrected gt
          ax_gt = ax.plot(gt_data_cor[:,1], gt_data_cor[:,2], gt_data_cor[:,3], colors[0], label='Ground Truth')

    except:
      print "Error while plotting ", graph_vertices_file


  # Show the edges
  if (edges_shown == True):
    draw_edges()
  else:
    remove_edges()

  # Update the plot
  pyplot.draw()

  # Show legend only once
  if (first_iter == True):
    ax.legend()
    first_iter = False

def draw_edges():
  global blocking_file, graph_edges_file, ax_edges, edges_shown

  # Load stereo slam edges (file saved with node stereo_slam)
  if (graph_edges_file != "" and os.path.exists(graph_edges_file)):
    
    # Check if file is blocked
    while (os.path.exists(blocking_file) and os.path.isfile(blocking_file)):
      time.sleep(0.5)

    try:
      data = pylab.loadtxt(graph_edges_file, delimiter=',', skiprows=0, usecols=(1,2,3,4,5,6))
      # First, remove previous edges
      remove_edges()
      # Plot current
      ax_edges = []
      for i in range(len(data)):
        vect = []
        vect.append([data[i,0], data[i,1], data[i,2]])
        vect.append([data[i,3], data[i,4], data[i,5]])
        vect =  np.array(vect)
        ax_edge = ax.plot(vect[:,0], vect[:,1], vect[:,2], colors[2], linestyle='--')
        ax_edges.append(ax_edge)
    except:
      print "Error while plotting ", graph_edges_file
  edges_shown = True

def remove_edges():
  global ax_edges, edges_shown
  # Remove old lines
  for i in range(len(ax_edges)):
    l = ax_edges[i].pop(0)
    wl = weakref.ref(l)
    l.remove()
    del l
  ax_edges = []
  edges_shown = False

def onclick(event):
  global edges_shown, correct_graph
 
  if (event.button == 3):
    if (edges_shown):
      remove_edges()
    else:
      draw_edges()
    # Update the plot
    pyplot.draw()
  elif (event.button == 2):
    correct_graph = not correct_graph


if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(
          description='Plot 3D graphics of odometry data files in real time.',
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

  print "GRAPH VIEWER MOUSE INPUTS:"
  print " - Right button: activates/deactivates the visualization of graph edges."
  print " - Wheel button: activates/deactivates the initialization of nodes path to (x, y, z, q) to (0, 0, 0, 0)."

  # Save blocking file into global
  blocking_file = os.path.dirname(args.graph_vertices_file) + "/.block.txt"

  # Save graph edges file into global
  graph_edges_file = args.graph_edges_file

  # Init figure
  fig = pylab.figure(1)
  ax = Axes3D(fig)
  ax.grid(True)
  ax.set_title("Graph Viewer")
  ax.set_xlabel("X")
  ax.set_ylabel("Y")
  ax.set_zlabel("Z")

  # Load ground truth (gt) data. Note that gt comes from standard ros odometry message.
  if (args.ground_truth_file != "" and os.path.exists(args.ground_truth_file)):
    
    # Check the file type
    f = open(args.ground_truth_file)
    lines = f.readlines()
    f.close()
    size = lines[1].split(",")

    if (len(size) > 12):
      gt_data = pylab.loadtxt(args.ground_truth_file, delimiter=',', skiprows=1, usecols=(0,5,6,7,8,9,10,11))
    else:
      gt_data = pylab.loadtxt(args.ground_truth_file, delimiter=',', usecols=(0,1,2,3,4,5,6,7))

    try:
      ax_gt = ax.plot(gt_data[:,1], gt_data[:,2], gt_data[:,3], colors[0], label='Ground Truth')
    except:
      pass

  # Handle on click callback
  fig.canvas.mpl_connect('button_press_event', onclick)

  # Start timer for real time plot
  timer = fig.canvas.new_timer(2500)
  real_time_plot(args.visual_odometry_file, args.graph_vertices_file)
  timer.add_callback( real_time_plot, 
                      args.visual_odometry_file, 
                      args.graph_vertices_file)
  timer.start()  
  pylab.show()