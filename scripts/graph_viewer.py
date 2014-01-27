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
legend_edited = False
colors = ['g','r','b']
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

  global blocking_file, legend_edited, colors, ax_gt, ax_odom, ax_vertices, edges_shown, gt_data, plot_dim

  # Remove the main axes
  rm_ax(ax_gt)
  rm_ax(ax_odom)
  rm_ax(ax_vertices)

  # Load visual odometry data (saved with rostopic echo -p /stereo_odometer/odometry > file.txt)
  if (gt_file != "" and os.path.exists(gt_file) and check_file_len(gt_file)):

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
      ax_gt = ax.plot(data[:,1], data[:,2], data[:,3], colors[0], label='Ground Truth')
    else:
      ax_gt = ax.plot(data[:,1], data[:,2], colors[0], label='Ground Truth')

  # Load visual odometry data (saved with rostopic echo -p /stereo_odometer/odometry > file.txt)
  if (odom_file != "" and os.path.exists(odom_file) and check_file_len(odom_file)):

    # Read the data
    data = pylab.loadtxt(odom_file, delimiter=',', skiprows=1, usecols=(5,6,7,8,9,10,11))
    
    # Plot
    if (len(data.shape) == 1):
      data = np.array([data])
    if (plot_dim == 3):
      ax_odom = ax.plot(data[:,0], data[:,1], data[:,2], colors[1], label='Visual Odometry')
    else:
      ax_odom = ax.plot(data[:,0], data[:,1], colors[1], label='Visual Odometry')

  # Load stereo slam vertices (file saved with node stereo_slam)
  if (graph_vertices_file != "" and os.path.exists(graph_vertices_file) and check_file_len(graph_vertices_file)):

    # Check if file is blocked
    while (os.path.exists(blocking_file) and os.path.isfile(blocking_file)):
      time.sleep(0.5)

    # Read the data
    data = pylab.loadtxt(graph_vertices_file, delimiter=',', skiprows=0, usecols=(0,5,6,7,8,9,10,11))
    
    # Plot
    if (len(data.shape) == 1):
      data = np.array([data])
    if (plot_dim == 3):
      ax_vertices = ax.plot(data[:,1], data[:,2], data[:,3], colors[2], label='Stereo slam', marker='o')
    else:
      ax_vertices = ax.plot(data[:,1], data[:,2], colors[2], label='Stereo slam', marker='o')

  # Show the edges
  if (edges_shown == True):
    draw_edges()
  else:
    remove_edges()

  # Update the plot
  pyplot.draw()

  # Show legend only once
  if (ax_gt is not None and ax_odom is not None and ax_vertices is not None and legend_edited is False):
    ax.legend()
    legend_edited = True

def draw_edges():
  """ Draw the edges """
  global blocking_file, graph_edges_file, ax_edges, edges_shown, plot_dim

  # First, remove previous edges
  remove_edges()

  # Load stereo slam edges (file saved with node stereo_slam)
  if (graph_edges_file != "" and os.path.exists(graph_edges_file) and check_file_len(graph_edges_file)):

    # Check if file is blocked
    while (os.path.exists(blocking_file) and os.path.isfile(blocking_file)):
      time.sleep(0.5)

    # Read the data
    data = pylab.loadtxt(graph_edges_file, delimiter=',', skiprows=0, usecols=(1,2,3,4,5,6))

    # Plot current
    if (len(data.shape) == 1):
      data = np.array([data])
    ax_edges = []
    for i in range(len(data)):
      vect = []
      vect.append([data[i,0], data[i,1], data[i,2]])
      vect.append([data[i,3], data[i,4], data[i,5]])
      vect =  np.array(vect)
      if (plot_dim == 3):
        ax_edge = ax.plot(vect[:,0], vect[:,1], vect[:,2], colors[2], linestyle='--')
      else:
        ax_edge = ax.plot(vect[:,0], vect[:,1], colors[2], linestyle='--')
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
  parser.add_argument('ground_truth_file', 
          help='file with ground truth')
  parser.add_argument('visual_odometry_file', 
          help='file with visual odometry')
  parser.add_argument('graph_vertices_file', 
          help='file the vertices of stereo slam')
  parser.add_argument('graph_edges_file', 
          help='file the edges of stereo slam')
  parser.add_argument('-dim','--dim', type=int,
            help='defines the plot dimensions: 2 for xy and 3 for xyz',
            default=3)
  args = parser.parse_args()
  plot_dim = args.dim

  # Some hardcode parameters
  font = {'family' : 'Sans',
          'weight' : 'normal',
          'size'   : 14}
  pylab.rc('font', **font)

  print "GRAPH VIEWER MOUSE INPUTS:"
  print " - Right button: activates/deactivates the visualization of graph edges."

  # Save blocking file into global
  blocking_file = os.path.dirname(args.graph_vertices_file) + "/.block.txt"

  # Save graph edges file into global
  graph_edges_file = args.graph_edges_file

  # Init figure
  fig = pylab.figure(1)
  if (plot_dim == 3):
    ax = Axes3D(fig)
    ax.set_zlabel("Z")
  else:
    ax = fig.gca()
  ax.grid(True)
  ax.set_title("Graph Viewer")
  ax.set_xlabel("X")
  ax.set_ylabel("Y")

  # Handle on click callback
  fig.canvas.mpl_connect('button_press_event', onclick)

  # Start timer for real time plot
  timer = fig.canvas.new_timer(2500)
  real_time_plot(args.ground_truth_file, args.visual_odometry_file, args.graph_vertices_file)
  timer.add_callback( real_time_plot,
                      args.ground_truth_file, 
                      args.visual_odometry_file, 
                      args.graph_vertices_file)
  timer.start()
  pylab.show()