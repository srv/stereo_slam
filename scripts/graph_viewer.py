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
from scipy.misc import imread

# Global variables
lock_file = ""
graph_edges_file = ""
legend_edited = False
ax_gt = None
ax_odom = None
ax_vertices = None
ax_edges = []
edges_shown = True
gt_data = []
plot_dim = 3

class Error(Exception):
  """ Base class for exceptions in this module. """
  pass

def check_file_len(file):
  """ Check if the file length is > 0 """
  f = open(file)
  lines = f.readlines()
  f.close()
  return len(lines) > 0

def rm_ax(ax_id):
  """ Remove the axes lines """
  if (ax_id is not None and ax_id):
    l = ax_id.pop(0)
    wl = weakref.ref(l)
    l.remove()
    del l

def real_time_plot(gt_file, odom_file, graph_vertices_file):
  """ Function to plot the data saved into the files in real time """

  global lock_file, legend_edited, ax_gt, ax_odom, ax_vertices, edges_shown, gt_data, plot_dim

  # Remove the main axes
  rm_ax(ax_gt)
  rm_ax(ax_odom)
  rm_ax(ax_vertices)

  # Ground truth
  if (gt_file != "" and os.path.exists(gt_file) and check_file_len(gt_file) and gt_file != "none"):

    # Check gt file type
    f = open(gt_file)
    lines = f.readlines()
    f.close()
    size = lines[1].split(",")
    if (len(size) > 12):
      data = pylab.loadtxt(gt_file, delimiter=',', skiprows=1, usecols=(0,5,6,7,8,9,10,11))
    else:
      data = pylab.loadtxt(gt_file, delimiter=',', usecols=(0,1,2,3,4,5,6,7))

    # Plot
    if (len(data.shape) == 1):
      data = [data]
      data = np.array(data)
    if (plot_dim == 3):
      ax_gt = ax.plot(data[:,1], data[:,2], data[:,3],'k', label='Ground Truth')
    else:
      ax_gt = ax.plot(data[:,1], data[:,2], 'k', label='Ground Truth')

  # Load visual odometry data (saved with rostopic echo -p /stereo_odometer/odometry > file.txt)
  if (odom_file != "" and os.path.exists(odom_file) and check_file_len(odom_file)):

    # Read the data
    data = pylab.loadtxt(odom_file, delimiter=',', skiprows=1, usecols=(5,6,7,8,9,10,11))
    # data = pylab.loadtxt(odom_file, delimiter=',', skiprows=1, usecols=(4,5,6,7,8,9,10))

    # Plot
    if (len(data.shape) == 1):
      data = np.array([data])
    if (plot_dim == 3):
      ax_odom = ax.plot(data[:,0], data[:,1], data[:,2], 'c', label='Visual Odometry')
    else:
      ax_odom = ax.plot(data[:,0], data[:,1], 'c', label='Visual Odometry')

  # Load stereo slam vertices (file saved with node stereo_slam)
  if (graph_vertices_file != "" and os.path.exists(graph_vertices_file) and check_file_len(graph_vertices_file)):

    # Check if file is blocked
    while (os.path.exists(lock_file)):
      time.sleep(0.5)

    # Read the data
    try:
      data = pylab.loadtxt(graph_vertices_file, delimiter=',', skiprows=0, usecols=(2,3,4,5,6,7,8))
    except:
      return;

    # Plot
    if (len(data.shape) == 1):
      data = np.array([data])
    if (plot_dim == 3):
      ax_vertices = ax.plot(data[:,0], data[:,1], data[:,2], 'b', label='Stereo slam', marker='o', zorder=1)
    else:
      ax_vertices = ax.plot(data[:,0], data[:,1], 'b', label='Stereo slam', marker='o', zorder=1)

  # Show the edges
  if (edges_shown == True):
    draw_edges()
  else:
    remove_edges()

  # Update the plot
  pyplot.draw()

  # Show legend only once
  if (ax_vertices is not None and legend_edited is False):
    #ax.legend()
    legend_edited = True

def draw_edges():
  """ Draw the edges """
  global lock_file, graph_edges_file, ax_edges, edges_shown, plot_dim

  # First, remove previous edges
  remove_edges()

  # Load stereo slam edges (file saved with node stereo_slam)
  if (graph_edges_file != "" and os.path.exists(graph_edges_file) and check_file_len(graph_edges_file)):

    # Check if file is blocked
    while (os.path.exists(lock_file)):
      time.sleep(0.5)

    # Read the data
    try:
      data = pylab.loadtxt(graph_edges_file, delimiter=',', skiprows=0, usecols=(0,1,2,3,4,5,10,11,12))
    except:
      return;

    # Sanity check
    if (len(data.shape) == 1):
      edges_shown = True
      return

    # Inliers column
    inliers = data[:,2]
    max_inliers = max(inliers)
    min_inliers = min(inliers)

    valid_color = True
    if (min_inliers - max_inliers == 0):
      valid_color = False

    if (valid_color):
      # red color
      m_red = (255) / (min_inliers - max_inliers)
      n_red = -m_red * max_inliers

      # blue color
      m_blue = (255) / (max_inliers - min_inliers)
      n_blue = -m_blue * min_inliers

    # Plot current
    if (len(data.shape) == 1):
      data = np.array([data])
    ax_edges = []
    for i in range(len(data)):

      # Get the color depending on the inlier value
      if (valid_color):
        red = hex(int(m_red * data[i,2] + n_red))
        blue = hex(int(m_blue * data[i,2] + n_blue))
        red = red[2:]
        blue = blue[2:]
        if (len(red) == 1):
          red += '0'
        if (len(blue) == 1):
          blue += '0'
        color = '#' + red + '00' + blue
      else:
        color = 'b';

      vect = []
      vect.append([data[i,3], data[i,4], data[i,5]])
      vect.append([data[i,6], data[i,7], data[i,8]])
      vect =  np.array(vect)
      if (plot_dim == 3):
        ax_edge = ax.plot(vect[:,0], vect[:,1], vect[:,2], color, linestyle='-', zorder=1)
      else:
        ax_edge = ax.plot(vect[:,0], vect[:,1], color, linestyle='-', zorder=1)
      ax_edges.append(ax_edge)
  edges_shown = True
  return

def remove_edges():
  """ Remove the edges """
  global ax_edges, edges_shown

  for i in range(len(ax_edges)):
    l = ax_edges[i].pop(0)
    wl = weakref.ref(l)
    l.remove()
    del l

  ax_edges = []
  edges_shown = False

def onclick(event):
  """ Handle the click event """
  global edges_shown

  if (event.button == 3):
    if (edges_shown):
      remove_edges()
    else:
      draw_edges()
    pyplot.draw()


if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(
          description='Plot 3D graphics of odometry data files in real time.',
          formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('-d','--d',
          help='directory with the files: gt.txt (optional), odom.txt, graph_vertices.txt, graph_edges.txt',
          default=".")
  parser.add_argument('-gt','--gt',
          help='file with ground truth',
          default="")
  parser.add_argument('-o','--o',
          help='file with visual odometry',
          default="")
  parser.add_argument('-v','--v',
          help='file with the vertices of stereo slam',
          default="")
  parser.add_argument('-e','--e',
          help='file with the edges of stereo slam',
          default="")
  parser.add_argument('-dim','--dim', type=int,
          help='defines the plot dimensions: 2 for xy and 3 for xyz',
          default=3)
  args = parser.parse_args()
  plot_dim = args.dim

  # Some hardcode parameters
  # font = {'family' : 'Sans',
  #         'weight' : 'normal',
  #         'size'   : 30}
  # pylab.rc('font', **font)

  print "GRAPH VIEWER MOUSE INPUTS:"
  print " - Right button: activates/deactivates the visualization of graph edges."

  # Set parameters
  global_dir            = args.d
  ground_truth_file     = args.gt
  visual_odometry_file  = args.o
  graph_vertices_file   = args.v
  graph_edges_file      = args.e

  # Default parameters
  if (global_dir != ""):
    if (global_dir[:-1] != "/"):
      global_dir += "/"
    visual_odometry_file = global_dir + "odom.txt"
    graph_vertices_file = global_dir + "graph_vertices.txt"
    graph_edges_file = global_dir + "graph_edges.txt"
    ground_truth_file = global_dir + "gt.txt"
  if not os.path.exists(ground_truth_file):
    ground_truth_file = "none"

  # Save blocking file into global
  lock_file = os.path.dirname(graph_vertices_file) + "/graph.lock"

  # Init figure
  fig = pylab.figure(1)
  if (plot_dim == 3):
    ax = Axes3D(fig)
    ax.set_zlabel("z (m)")
  else:
    ax = fig.gca()
    img = imread("/home/plnegre/Downloads/mosaic1.jpg")
    ax.imshow(img, zorder=0, extent=[-13.5, 34.5, -17, 9])
    ax.set_axis_bgcolor('black')

  ax.grid(True)
  # ax.set_title("Graph Viewer")
  ax.set_xlabel("x (m)")
  ax.set_ylabel("y (m)")
  # ax.set_xlim([0, 28])
  # ax.set_ylim([-10, 23])

  # Handle on click callback
  fig.canvas.mpl_connect('button_press_event', onclick)

  # Start timer for real time plot
  timer = fig.canvas.new_timer(2500)
  real_time_plot(ground_truth_file, visual_odometry_file, graph_vertices_file)
  timer.add_callback( real_time_plot,
                      ground_truth_file,
                      visual_odometry_file,
                      graph_vertices_file)
  timer.start()
  pylab.show()
