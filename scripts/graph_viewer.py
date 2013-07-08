#!/usr/bin/env python
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

# Global variables
blocking_file = ""
graph_edges_file = ""
first_iter = True
colors = ['g','r','b']
ax_odom = None
ax_vertices = None
ax_edges = []
edges_shown = True
correct_graph = False

class Error(Exception):
  """ Base class for exceptions in this module. """
  pass

def real_time_plot(odom_file, graph_vertices_file):
  """
  Function to plot the data saved into the files in real time
  """
  global blocking_file, first_iter, colors, ax_odom, ax_vertices, edges_shown, correct_graph

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
      data = pylab.loadtxt(graph_vertices_file, delimiter=',', skiprows=0, usecols=(5,6,7,8,9,10,11))
      # Remove old line collection
      if (first_iter == False):
        l = ax_vertices.pop(0)
        wl = weakref.ref(l)
        l.remove()
        del l

      # Correct data if needed
      if (correct_graph == True):
        initial = []
        initial.append(data[0,:])
        initial =  np.array(initial)
        initial[0,6] = initial[0,6] - 1
        data = data - initial[0,:]

      ax_vertices = ax.plot(data[:,0], data[:,1], data[:,2], colors[2], label='Stereo slam', marker='o')
    except:
      print "No data in ", graph_vertices_file

  # Show the edges
  if (edges_shown == True and correct_graph == False):
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
      print "No data in ", graph_edges_file
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
  parser.add_argument('-ts','--time-step',
          help='update frequency (in milliseconds)',
          default='2500')
  args = parser.parse_args()

  print "GRAPH VIEWER MOUSE INPUTS:"
  print " - Right button: activates/deactivates the visualization of graph edges."
  print " - Wheel button: activates/deactivates the initialization of nodes path to (x, y, z, q) to (0, 0, 0, 0). Note that edges are NOT modified!"

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
      data = pylab.loadtxt(args.ground_truth_file, delimiter=',', skiprows=1, usecols=(5,6,7))
    else:
      data = pylab.loadtxt(args.ground_truth_file, delimiter=',', usecols=(1,2,3))

    ax.plot(data[:,0], data[:,1], data[:,2], colors[0], label='Ground Truth')

  # Handle on click callback
  fig.canvas.mpl_connect('button_press_event', onclick)

  # Start timer for real time plot
  timer = fig.canvas.new_timer(interval=args.time_step)
  real_time_plot(args.visual_odometry_file, args.graph_vertices_file)
  timer.add_callback( real_time_plot, 
                      args.visual_odometry_file, 
                      args.graph_vertices_file)
  timer.start()  
  pylab.show()